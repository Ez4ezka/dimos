# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""PX4 Offboard flight supervisor state machine. Pure logic: no socket, no port, no pymavlink.

States::

  IDLE -> PREFLIGHT -> STREAMING -> OFFBOARD_REQ -> ARMING -> TAKEOFF -> HOVER
  HOVER <-> YAW_TRACK / FOLLOW / TELEOP       (operator selects the guidance mode)
  any armed state -> LANDING -> IDLE          (operator land)
  any armed state -> ABORT -> IDLE            (safety rule: PX4 put in Hold, setpoints stop)
  pilot leaves Offboard -> IDLE               (PILOT_OVERRIDE: we never touch the mode)

Every tick the owner takes a :class:`VehicleSnapshot`, calls :meth:`SupervisorCore.step`,
then :meth:`SupervisorCore.stream`. All vehicle side effects go through the
:class:`Px4Actuator` protocol so the whole machine runs against a fake in tests.

Two clocks: ``now`` is the tick clock used for state timers and integration; staleness
comes from the snapshot's own receive-time ages. Keep them apart (the ported tests
simulate seconds of tick time in milliseconds of wall time).

The guidance laws the states use sit above the core in this file: YAW_TRACK unwinds the
gimbal toward the airframe centre-line by yawing the vehicle, FOLLOW holds a standoff
distance and altitude to the estimated target with velocity feed-forward. Neither ever
commands from raw pixel error.

Ported from drone-autonomy ``flight_supervisor.py:43-303`` and ``common/guidance.py``
(flown 2026-09-09), maths unchanged. Additions over the flown code: the TELEOP guidance
mode, the E-STOP latch, the closed rejection enum, and ``set_hold``/``set_land`` gated on
PX4 being in OFFBOARD (safety invariant 1).
"""

from __future__ import annotations

from dataclasses import dataclass
import enum
import math
import time
from typing import Any, Literal, Protocol

from dimos.robot.px4.config import FollowConfig, GuidanceConfig, SupervisorLimits, YawTrackConfig
from dimos.robot.px4.mavlink import (
    LANDED_ON_GROUND,
    MAIN_AUTO,
    MAIN_OFFBOARD,
    SUB_AUTO_LAND,
    SUB_AUTO_LOITER,
    VehicleSnapshot,
    body_flu_velocity_to_ned,
    mode_name,
)
from dimos.utils.angles import clamp, wrap180

ARMED_STATES = frozenset({"ARMING", "TAKEOFF", "HOVER", "YAW_TRACK", "FOLLOW", "TELEOP", "LANDING"})
GUIDANCE_STATES = frozenset({"HOVER", "YAW_TRACK", "FOLLOW", "TELEOP"})
GuidanceMode = Literal["HOVER", "YAW_TRACK", "FOLLOW", "TELEOP"]
GUIDANCE_MODES: tuple[GuidanceMode, ...] = ("HOVER", "YAW_TRACK", "FOLLOW", "TELEOP")

# Time constants of the flown supervisor (flight_supervisor.py:189,194).
_ABORT_DWELL_S = 2.0
_PREFLIGHT_TIMEOUT_S = 10.0
_MAX_STEP_DT_S = 0.2


class Rejection(enum.Enum):
    """Why a command or input was refused. Closed set: the command tracker classifies on it."""

    NOT_TELEOP = "not_teleop"
    ESTOP_LATCHED = "estop_latched"
    ENABLE_SWITCH_OFF = "enable_switch_off"
    STALE_INPUT = "stale_input"
    PREFLIGHT_FAILED = "preflight_failed"
    NOT_ARMED = "not_armed"
    MODE_NOT_OFFBOARD = "mode_not_offboard"
    FENCE = "fence"
    CEILING = "ceiling"
    BATTERY = "battery"
    # Not in the kickoff list: a command that makes no sense in the current state
    # (takeoff while flying, estop_clear while not IDLE). The tracker treats it as a
    # plain refusal.
    WRONG_STATE = "wrong_state"


@dataclass(frozen=True)
class TakeoffPoint:
    """Captured before arming; never changed after (it is the geofence origin)."""

    n: float
    e: float
    d0: float
    yaw: float


@dataclass(frozen=True)
class HoverPoint:
    n: float
    e: float
    d: float


@dataclass(frozen=True)
class Setpoint:
    """What :meth:`SupervisorCore.stream` sends. ``pos`` uses n/e/d, ``vel`` uses vn/ve/vd."""

    kind: Literal["pos", "vel"]
    yaw: float  # degrees, NED heading
    n: float = 0.0
    e: float = 0.0
    d: float = 0.0
    vn: float = 0.0
    ve: float = 0.0
    vd: float = 0.0
    yaw_rate: float | None = None  # deg/s; when set, a vel setpoint commands rate not heading
    range_m: float | None = None
    bearing_deg: float | None = None


@dataclass(frozen=True)
class TeleopCommand:
    """A body-frame FLU velocity request (dimOS ``Twist``) with its receive time."""

    forward: float
    left: float
    up: float
    yaw_rate_ccw: float  # rad/s
    t: float

    @property
    def is_zero(self) -> bool:
        return (
            self.forward == 0.0 and self.left == 0.0 and self.up == 0.0 and self.yaw_rate_ccw == 0.0
        )


# Guidance laws. Pure functions; the config dicts of the flown stack became the
# dataclasses in config.py.


@dataclass
class TargetEstimate:
    """One ``target_state_v1`` packet as the supervisor sees it (NED, metres, m/s)."""

    valid: bool = False
    n: float | None = None
    e: float | None = None
    vn: float = 0.0
    ve: float = 0.0
    los_valid: bool = False
    gimbal_yaw_body_deg: float | None = None
    track_id: int | None = None
    range_m: float | None = None
    bearing_deg: float | None = None
    reason: str = ""


@dataclass(frozen=True)
class FollowCommand:
    vn: float
    ve: float
    vd: float
    yaw_deg: float
    range_m: float
    bearing_deg: float


def yaw_track_rate(
    gimbal_yaw_body_deg: float | None, target_visible: bool, cfg: YawTrackConfig
) -> float:
    """deg/s of vehicle yaw to command. Positive = clockwise (right), toward the gimbal."""
    if not target_visible or gimbal_yaw_body_deg is None:
        return 0.0
    g = gimbal_yaw_body_deg
    db = cfg.deadband_deg
    if abs(g) <= db:
        return 0.0
    err = g - math.copysign(db, g)
    return clamp(cfg.k_yaw * err, -cfg.max_yaw_rate_dps, cfg.max_yaw_rate_dps)


def follow_velocity(
    veh_n: float,
    veh_e: float,
    veh_d: float,
    d_takeoff: float,
    target: TargetEstimate,
    cfg: FollowConfig,
) -> FollowCommand:
    """Velocity + yaw command to keep standoff geometry to the target."""
    assert target.n is not None and target.e is not None, "follow_velocity needs a position"
    dn, de = target.n - veh_n, target.e - veh_e
    rng = math.hypot(dn, de)
    bearing = (math.degrees(math.atan2(de, dn)) + 360.0) % 360.0
    if rng < 1e-3:
        un, ue = 1.0, 0.0
    else:
        un, ue = dn / rng, de / rng
    err = rng - cfg.standoff_m
    if abs(err) <= cfg.range_deadband_m:
        v_rad = 0.0
    else:
        v_rad = cfg.k_range * (err - math.copysign(cfg.range_deadband_m, err))
    vn = v_rad * un + cfg.ff_gain * target.vn
    ve = v_rad * ue + cfg.ff_gain * target.ve
    speed = math.hypot(vn, ve)
    if speed > cfg.v_max_mps:
        vn, ve = vn * cfg.v_max_mps / speed, ve * cfg.v_max_mps / speed
    d_des = d_takeoff - cfg.altitude_m
    vd = clamp(cfg.k_alt * (d_des - veh_d), -cfg.vz_max_mps, cfg.vz_max_mps)
    return FollowCommand(vn=vn, ve=ve, vd=vd, yaw_deg=bearing, range_m=rng, bearing_deg=bearing)


def rate_limit_yaw(current_deg: float, desired_deg: float, max_rate_dps: float, dt: float) -> float:
    step = clamp(wrap180(desired_deg - current_deg), -max_rate_dps * dt, max_rate_dps * dt)
    return wrap180(current_deg + step)


class Px4Actuator(Protocol):
    """The only way the core touches the aircraft."""

    def set_mode(self, main: int, sub: int = 0) -> None: ...

    def arm(self, value: bool) -> None: ...

    def send_position_setpoint(self, n: float, e: float, d: float, yaw_rad: float) -> None: ...

    def send_velocity_setpoint(
        self,
        vn: float,
        ve: float,
        vd: float,
        yaw_rad: float | None = None,
        yaw_rate_rad: float | None = None,
    ) -> None: ...


class SupervisorCore:
    def __init__(
        self,
        limits: SupervisorLimits,
        guidance: GuidanceConfig,
        sitl: bool = False,
    ) -> None:
        self.cfg = limits
        self.gcfg = guidance
        self.sitl = sitl
        self.state = "IDLE"
        self.state_since = time.time()
        self.reason = "startup"
        self.guidance_mode: GuidanceMode = "HOVER"
        self.fake_enable = False
        self.target: TargetEstimate | None = None
        self.target_rx_t = 0.0  # any target_state_v1 packet
        self.target_valid_t = 0.0  # last packet with a usable (valid, n/e) target
        self.sp: Setpoint | None = None
        self.yaw_cmd: float = 0.0
        self.takeoff: TakeoffPoint | None = None
        self.hover: HoverPoint | None = None
        self.entered_offboard = False
        self.last_sp = 0.0
        self.last_step: float | None = None
        self.estop_latched = False
        self.last_rejection: Rejection | None = None
        self.teleop: TeleopCommand | None = None
        self.setpoint_count = 0
        self.transitions: list[tuple[str, str, float]] = []

    def goto(self, state: str, reason: str = "", now: float | None = None) -> None:
        t = time.time() if now is None else now
        if state != self.state:
            self.transitions.append((state, reason, t))
        self.state, self.state_since, self.reason = state, t, reason

    def enable_switch(self, st: VehicleSnapshot) -> bool:
        if self.sitl:
            return self.fake_enable
        if st.rc is None or st.rc_age > self.cfg.rc_stale_s:
            return False
        ch = self.cfg.enable_channel
        if ch < 1 or ch > len(st.rc.chan):
            return False
        return st.rc.chan[ch - 1] >= self.cfg.enable_threshold_us

    @staticmethod
    def _px4_age(st: VehicleSnapshot) -> float:
        # HEARTBEAT is 1 Hz and px4_stale_s is 1.0 s, so heartbeat age alone sits on the
        # threshold and a few ms of jitter would abort a flight (seen in SITL). Any message
        # from 1/1 proves the link; the flown code tracked last_px4_msg but never used it.
        return min(st.heartbeat_age, st.px4_msg_age)

    def preflight_failures(self, st: VehicleSnapshot) -> list[str]:
        c, f = self.cfg, []
        if self._px4_age(st) > c.px4_stale_s:
            f.append("PX4 heartbeat stale")
        if st.local is None or st.local_age > 0.5:
            f.append("local position stale")
        if st.gps is None or st.gps.fix < c.min_fix_type:
            f.append("GPS fix")
        elif not math.isnan(st.gps.eph) and st.gps.eph > c.max_eph_m:
            f.append(f"eph {st.gps.eph:.1f}")
        batt = st.batt_pct
        if batt >= 0 and batt < c.min_batt_pct:
            f.append(f"battery {batt}%")
        if not self.sitl and (st.rc is None or st.rc_age > c.rc_stale_s):
            f.append("RC stale")
        if not self.enable_switch(st):
            f.append("enable switch off")
        if st.landed_state is None or st.landed_state != LANDED_ON_GROUND:
            f.append("not on ground")
        if st.armed:
            f.append("already armed")
        # PX4 refuses to arm for Offboard without an absolute position estimate; saying so
        # here beats "arming refused" three seconds later.
        if st.estimator is not None and not st.estimator.position_valid:
            f.append("position estimate not valid")
        return f

    def _px4_said(self, st: VehicleSnapshot, hint: str) -> str:
        """PX4's own warning since the current state began, else the operator's hint."""
        text = st.statustext
        if text is not None and text.t >= self.state_since:
            return f": {text.text}"
        return f" ({hint})"

    def abort_reason(self, st: VehicleSnapshot) -> tuple[str, Rejection] | None:
        c = self.cfg
        if self._px4_age(st) > c.px4_stale_s:
            return "PX4 heartbeat lost", Rejection.STALE_INPUT
        if not self.enable_switch(st):
            return "enable switch off / RC lost", Rejection.ENABLE_SWITCH_OFF
        batt = st.batt_pct
        if 0 <= batt < c.min_batt_pct - 10:
            return f"battery {batt}%", Rejection.BATTERY
        if st.local and self.takeoff:
            dist = math.hypot(st.local.n - self.takeoff.n, st.local.e - self.takeoff.e)
            alt = self.takeoff.d0 - st.local.d
            if dist > c.geofence_radius_m:
                return f"geofence {dist:.0f} m", Rejection.FENCE
            if alt > c.max_alt_m:
                return f"altitude {alt:.1f} m", Rejection.CEILING
        return None

    def target_fresh(self, now: float | None = None) -> bool:
        t = time.time() if now is None else now
        return (
            self.target is not None
            and self.target.valid
            and self.target.n is not None
            and self.target.e is not None
            and t - self.target_rx_t <= self.cfg.target_stale_s
        )

    # Operator commands. Each returns None when accepted or the rejection reason.

    def sitl_enable(self, value: bool) -> Rejection | None:
        if not self.sitl:
            return Rejection.WRONG_STATE
        self.fake_enable = bool(value)
        return None

    def takeoff_cmd(self, st: VehicleSnapshot, now: float | None = None) -> Rejection | None:
        if self.estop_latched:
            return self._reject(Rejection.ESTOP_LATCHED)
        if self.state != "IDLE":
            return self._reject(Rejection.WRONG_STATE)
        self.goto("PREFLIGHT", "operator takeoff", now)
        return None

    def land_cmd(
        self, st: VehicleSnapshot, m: Px4Actuator, now: float | None = None
    ) -> Rejection | None:
        if not (self.state in ARMED_STATES or st.armed):
            return self._reject(Rejection.NOT_ARMED)
        if self.state == "LANDING":
            return self._reject(Rejection.WRONG_STATE)
        if not st.in_offboard:
            # The pilot owns the aircraft; we never send a mode command.
            return self._reject(Rejection.MODE_NOT_OFFBOARD)
        self.sp = None
        m.set_mode(MAIN_AUTO, SUB_AUTO_LAND)
        self.goto("LANDING", "operator land", now)
        return None

    def hold_cmd(
        self, st: VehicleSnapshot, m: Px4Actuator, now: float | None = None
    ) -> Rejection | None:
        if (self.state in ARMED_STATES or st.armed) and st.in_offboard:
            m.set_mode(MAIN_AUTO, SUB_AUTO_LOITER)
        self.sp = None
        self.teleop = None
        self.entered_offboard = False
        self.goto("IDLE", "operator hold", now)
        return None

    def set_guidance_mode(self, mode: GuidanceMode, now: float | None = None) -> Rejection | None:
        if mode not in GUIDANCE_MODES:
            raise ValueError(f"unknown guidance mode {mode!r}")
        if self.estop_latched:
            return self._reject(Rejection.ESTOP_LATCHED)
        self.guidance_mode = mode
        if self.state in GUIDANCE_STATES:
            self.goto(mode, "operator mode", now)
        return None

    def estop(self, st: VehicleSnapshot, m: Px4Actuator, now: float | None = None) -> None:
        """Hold plus latch. Synchronous; nothing can move the aircraft until estop_clear."""
        self.estop_latched = True
        self.sp = None
        self.teleop = None
        if (self.state in ARMED_STATES or st.armed) and st.in_offboard:
            m.set_mode(MAIN_AUTO, SUB_AUTO_LOITER)
        self.entered_offboard = False
        self.goto("IDLE", "ESTOP", now)

    def estop_land(self, st: VehicleSnapshot, m: Px4Actuator, now: float | None = None) -> None:
        self.estop_latched = True
        self.sp = None
        self.teleop = None
        if (self.state in ARMED_STATES or st.armed) and st.in_offboard:
            m.set_mode(MAIN_AUTO, SUB_AUTO_LAND)
            self.goto("LANDING", "ESTOP land", now)
            return
        self.entered_offboard = False
        self.goto("IDLE", "ESTOP land (pilot has the aircraft)", now)

    def estop_clear(self) -> Rejection | None:
        if self.state != "IDLE":
            return self._reject(Rejection.WRONG_STATE)
        self.estop_latched = False
        return None

    def on_target(self, target: TargetEstimate, now: float) -> None:
        self.target, self.target_rx_t = target, now

    def on_cmd_vel(self, cmd: TeleopCommand, now: float) -> Rejection | None:
        """Accept a teleop velocity request. Honoured only in TELEOP; clamped to the limits."""
        if self.estop_latched:
            return self._reject(Rejection.ESTOP_LATCHED)
        if self.state != "TELEOP":
            return self._reject(Rejection.NOT_TELEOP)
        values = (cmd.forward, cmd.left, cmd.up, cmd.yaw_rate_ccw, cmd.t)
        if not all(math.isfinite(v) for v in values) or now - cmd.t > self.cfg.teleop_stale_s:
            return self._reject(Rejection.STALE_INPUT)
        c = self.cfg
        self.teleop = TeleopCommand(
            forward=clamp(cmd.forward, -c.teleop_v_xy_mps, c.teleop_v_xy_mps),
            left=clamp(cmd.left, -c.teleop_v_xy_mps, c.teleop_v_xy_mps),
            up=clamp(cmd.up, -c.teleop_v_z_mps, c.teleop_v_z_mps),
            yaw_rate_ccw=clamp(cmd.yaw_rate_ccw, -c.teleop_yaw_rate_rps, c.teleop_yaw_rate_rps),
            t=cmd.t,
        )
        return None

    def _reject(self, why: Rejection) -> Rejection:
        self.last_rejection = why
        return why

    # Main step

    def step(self, st: VehicleSnapshot, m: Px4Actuator, now: float) -> None:
        s = self.state
        dt = 0.0 if self.last_step is None else max(0.0, min(_MAX_STEP_DT_S, now - self.last_step))
        self.last_step = now
        armed = st.armed
        in_offboard = st.in_offboard

        # Pilot override: we asked for Offboard, PX4 is no longer in it -> stop, never touch modes.
        if s in ARMED_STATES and s != "LANDING" and self.entered_offboard and not in_offboard:
            self.sp = None
            self.teleop = None
            self.entered_offboard = False
            self.last_rejection = Rejection.MODE_NOT_OFFBOARD
            hb = st.heartbeat
            name = mode_name(hb.main if hb else None, hb.sub if hb else 0)
            self.goto("IDLE", f"PILOT_OVERRIDE (mode now {name})", now)
            return
        if s in ARMED_STATES and s != "LANDING":
            r = self.abort_reason(st)
            if r:
                text, why = r
                self.sp = None
                self.teleop = None
                self.last_rejection = why
                m.set_mode(MAIN_AUTO, SUB_AUTO_LOITER)
                self.goto("ABORT", text, now)
                return

        if s == "IDLE":
            self.sp = None
            self.entered_offboard = False
        elif s == "ABORT":
            if now - self.state_since > _ABORT_DWELL_S:
                self.goto("IDLE", "after abort", now)
        elif s == "PREFLIGHT":
            fails = self.preflight_failures(st)
            if fails:
                if now - self.state_since > _PREFLIGHT_TIMEOUT_S:
                    self.last_rejection = (
                        Rejection.ENABLE_SWITCH_OFF
                        if "enable switch off" in fails
                        else Rejection.PREFLIGHT_FAILED
                    )
                    self.goto("IDLE", "preflight failed: " + ", ".join(fails), now)
                self.reason = "waiting: " + ", ".join(fails)
            else:
                assert st.local is not None
                self.takeoff = TakeoffPoint(
                    n=st.local.n,
                    e=st.local.e,
                    d0=st.local.d,
                    yaw=st.yaw_deg if st.yaw_deg is not None else 0.0,
                )
                self.yaw_cmd = self.takeoff.yaw
                self.sp = Setpoint(
                    kind="pos",
                    n=self.takeoff.n,
                    e=self.takeoff.e,
                    d=self.takeoff.d0,
                    yaw=self.yaw_cmd,
                )
                self.goto(
                    "STREAMING",
                    f"takeoff point N{self.takeoff.n:.1f} E{self.takeoff.e:.1f} D{self.takeoff.d0:.1f}",
                    now,
                )
        elif s == "STREAMING":
            if now - self.state_since >= self.cfg.prestream_s:
                m.set_mode(MAIN_OFFBOARD)
                self.goto("OFFBOARD_REQ", "requested OFFBOARD", now)
        elif s == "OFFBOARD_REQ":
            if in_offboard:
                self.entered_offboard = True
                m.arm(True)
                self.goto("ARMING", "arm requested", now)
            elif now - self.state_since > self.cfg.ack_timeout_s:
                self.sp = None
                self.last_rejection = Rejection.MODE_NOT_OFFBOARD
                hint = "check COM_RC_OVERRIDE / preflight in QGC"
                self.goto("IDLE", "PX4 did not enter OFFBOARD" + self._px4_said(st, hint), now)
        elif s == "ARMING":
            if armed:
                self.goto("TAKEOFF", "armed, climbing", now)
            elif now - self.state_since > self.cfg.ack_timeout_s:
                self.sp = None
                self.last_rejection = Rejection.NOT_ARMED
                m.set_mode(MAIN_AUTO, SUB_AUTO_LOITER)
                self.goto("IDLE", "arming refused" + self._px4_said(st, "see QGC messages"), now)
        elif s == "TAKEOFF":
            assert self.takeoff is not None
            d_goal = self.takeoff.d0 - self.cfg.takeoff_alt_m
            d_ramp = self.takeoff.d0 - self.cfg.climb_rate_mps * (now - self.state_since)
            self.sp = Setpoint(
                kind="pos",
                n=self.takeoff.n,
                e=self.takeoff.e,
                d=max(d_goal, d_ramp),
                yaw=self.yaw_cmd,
            )
            if (
                st.local
                and abs(st.local.d - d_goal) < self.cfg.hover_tolerance_m
                and d_ramp <= d_goal
            ):
                self.hover = HoverPoint(n=self.takeoff.n, e=self.takeoff.e, d=d_goal)
                self.goto("HOVER", f"at {self.cfg.takeoff_alt_m} m", now)
        elif s == "HOVER":
            self.hover = self.hover or self._here(st)
            self.sp = Setpoint(
                kind="pos", n=self.hover.n, e=self.hover.e, d=self.hover.d, yaw=self.yaw_cmd
            )
            if self.guidance_mode != "HOVER" and now - self.state_since > self.cfg.hover_settle_s:
                self.goto(self.guidance_mode, "settled", now)
        elif s == "YAW_TRACK":
            self.hover = self.hover or self._here(st)
            fresh = self.target_fresh(now) or (
                self.target is not None
                and self.target.los_valid
                and now - self.target_rx_t <= self.cfg.target_stale_s
            )
            g = self.target.gimbal_yaw_body_deg if self.target else None
            rate = yaw_track_rate(g, fresh, self.gcfg.yaw_track)
            self.yaw_cmd = wrap180(self.yaw_cmd + rate * dt)
            self.sp = Setpoint(
                kind="pos",
                n=self.hover.n,
                e=self.hover.e,
                d=self.hover.d,
                yaw=self.yaw_cmd,
                yaw_rate=rate,
            )
            if self.guidance_mode != "YAW_TRACK":
                self.goto("HOVER", "mode change", now)
        elif s == "FOLLOW":
            self._step_follow(st, now, dt)
        elif s == "TELEOP":
            self._step_teleop(st, now)
        elif s == "LANDING":
            self.sp = None
            if st.landed_state == LANDED_ON_GROUND and not armed:
                self.entered_offboard = False
                self.goto("IDLE", "landed and disarmed", now)

    def _step_follow(self, st: VehicleSnapshot, now: float, dt: float) -> None:
        fc = self.gcfg.follow
        if self.target_fresh(now):
            self.target_valid_t = now
        # Loss clock starts at the later of: last valid target, entering FOLLOW.
        age = now - max(self.target_valid_t, self.state_since)
        if self.target_fresh(now):
            assert st.local is not None and self.takeoff is not None and self.target is not None
            self.reason = "following"
            cmd = follow_velocity(
                st.local.n, st.local.e, st.local.d, self.takeoff.d0, self.target, fc
            )
            self.yaw_cmd = rate_limit_yaw(
                self.yaw_cmd, cmd.yaw_deg, self.gcfg.yaw_track.max_yaw_rate_dps, dt
            )
            self.sp = Setpoint(
                kind="vel",
                vn=cmd.vn,
                ve=cmd.ve,
                vd=cmd.vd,
                yaw=self.yaw_cmd,
                range_m=cmd.range_m,
                bearing_deg=cmd.bearing_deg,
            )
            self.hover = None
        elif age <= fc.loss_hover_s:
            self.hover = self.hover or self._here(st)
            self.sp = Setpoint(
                kind="pos", n=self.hover.n, e=self.hover.e, d=self.hover.d, yaw=self.yaw_cmd
            )
            self.reason = f"target lost {age:.1f}s: holding position"
        else:
            self.hover = self.hover or self._here(st)
            self.guidance_mode = "HOVER"
            self.goto("HOVER", f"target lost {age:.1f}s, holding here", now)
            return
        if self.guidance_mode != "FOLLOW":
            self.hover = self._here(st)
            self.goto("HOVER", "mode change", now)

    def _step_teleop(self, st: VehicleSnapshot, now: float) -> None:
        cmd = self.teleop
        moving = cmd is not None and now - cmd.t <= self.cfg.teleop_stale_s and not cmd.is_zero
        if moving:
            assert cmd is not None
            yaw_rad = math.radians(st.yaw_deg if st.yaw_deg is not None else self.yaw_cmd)
            up = 0.0 if self.cfg.teleop_lock_altitude else cmd.up
            vn, ve, vd = body_flu_velocity_to_ned(cmd.forward, cmd.left, up, yaw_rad)
            # dimOS yaw rate is counter-clockwise positive; NED heading rate is clockwise.
            rate_dps = -math.degrees(cmd.yaw_rate_ccw)
            self.yaw_cmd = st.yaw_deg if st.yaw_deg is not None else self.yaw_cmd
            self.sp = Setpoint(kind="vel", vn=vn, ve=ve, vd=vd, yaw=self.yaw_cmd, yaw_rate=rate_dps)
            self.hover = None
            self.reason = "teleop"
        else:
            if self.hover is None:
                self.hover = self._here(st)
                self.yaw_cmd = st.yaw_deg if st.yaw_deg is not None else self.yaw_cmd
            self.sp = Setpoint(
                kind="pos", n=self.hover.n, e=self.hover.e, d=self.hover.d, yaw=self.yaw_cmd
            )
            self.reason = "teleop idle: holding position"
        if self.guidance_mode != "TELEOP":
            self.teleop = None
            self.hover = self._here(st)
            self.goto("HOVER", "mode change", now)

    @staticmethod
    def _here(st: VehicleSnapshot) -> HoverPoint:
        assert st.local is not None, "no local position"
        return HoverPoint(n=st.local.n, e=st.local.e, d=st.local.d)

    def stream(self, m: Px4Actuator, now: float) -> bool:
        """Send the current setpoint at ``setpoint_hz``. Returns True when one was sent."""
        if self.sp is None or now - self.last_sp < 0.95 / self.cfg.setpoint_hz:
            return False
        self.last_sp = now
        sp = self.sp
        if sp.kind == "pos":
            m.send_position_setpoint(sp.n, sp.e, sp.d, math.radians(sp.yaw))
        elif sp.yaw_rate is not None:
            m.send_velocity_setpoint(sp.vn, sp.ve, sp.vd, yaw_rate_rad=math.radians(sp.yaw_rate))
        else:
            m.send_velocity_setpoint(sp.vn, sp.ve, sp.vd, yaw_rad=math.radians(sp.yaw))
        self.setpoint_count += 1
        return True

    def status(self, st: VehicleSnapshot, now: float | None = None) -> dict[str, Any]:
        """What the supervisor knows, for the status RPC and the skills."""
        t = time.time() if now is None else now
        hb = st.heartbeat
        sp = self.sp
        out: dict[str, Any] = dict(
            t=t,
            state=self.state,
            reason=self.reason,
            guidance_mode=self.guidance_mode,
            armed=st.armed,
            px4_mode=mode_name(hb.main if hb else None, hb.sub if hb else 0),
            enable=self.enable_switch(st),
            sitl=self.sitl,
            setpoint=None if sp is None else _setpoint_dict(sp),
            local=None if st.local is None else dict(n=st.local.n, e=st.local.e, d=st.local.d),
            batt_pct=st.sys_status.batt_pct if st.sys_status else None,
            gps=None if st.gps is None else dict(fix=st.gps.fix, sats=st.gps.sats, eph=st.gps.eph),
            target_fresh=self.target_fresh(t),
            target=None
            if self.target is None
            else dict(
                track_id=self.target.track_id,
                range_m=self.target.range_m,
                bearing_deg=self.target.bearing_deg,
                gimbal_yaw_body_deg=self.target.gimbal_yaw_body_deg,
            ),
            position_valid=None if st.estimator is None else st.estimator.position_valid,
            estop_latched=self.estop_latched,
            last_rejection=None if self.last_rejection is None else self.last_rejection.value,
            setpoint_count=self.setpoint_count,
            teleop_age_s=None if self.teleop is None else t - self.teleop.t,
        )
        if st.local and self.takeoff:
            out["alt_m"] = self.takeoff.d0 - st.local.d
            out["dist_m"] = math.hypot(st.local.n - self.takeoff.n, st.local.e - self.takeoff.e)
        return out


def _setpoint_dict(sp: Setpoint) -> dict[str, Any]:
    if sp.kind == "pos":
        return dict(kind="pos", n=sp.n, e=sp.e, d=sp.d, yaw=sp.yaw, yaw_rate=sp.yaw_rate)
    return dict(
        kind="vel",
        vn=sp.vn,
        ve=sp.ve,
        vd=sp.vd,
        yaw=sp.yaw,
        yaw_rate=sp.yaw_rate,
        range_m=sp.range_m,
        bearing_deg=sp.bearing_deg,
    )
