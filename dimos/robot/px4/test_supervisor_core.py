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

"""Guidance laws and the supervisor state machine against a fake vehicle and actuator.

Ported 1:1 from drone-autonomy ``tests/test_guidance.py`` (the suite that gated the
2026-09-09 flight). Same cases, same numbers, same assertions; only the call shapes changed
(typed commands, snapshots, the actuator protocol).
"""

from __future__ import annotations

import math
import time
from typing import Any

from dimos.robot.px4 import mavlink as px
from dimos.robot.px4.config import FollowConfig, GuidanceConfig, SupervisorLimits
from dimos.robot.px4.mavlink import VehicleSnapshot, VehicleState
from dimos.robot.px4.supervisor_core import (
    SupervisorCore,
    TakeoffPoint,
    TargetEstimate,
    follow_velocity,
    rate_limit_yaw,
    yaw_track_rate,
)

CFG = SupervisorLimits()
GCFG = GuidanceConfig()
YT = GCFG.yaw_track
FO = FollowConfig()


def close(a: float, b: float, tol: float = 0.05) -> None:
    assert abs(a - b) <= tol, (a, b)


def test_yaw_track_deadband_and_limit() -> None:
    assert yaw_track_rate(10.0, True, YT) == 0.0
    assert yaw_track_rate(40.0, False, YT) == 0.0
    assert yaw_track_rate(None, True, YT) == 0.0
    r = yaw_track_rate(40.0, True, YT)
    close(r, YT.k_yaw * 25.0)
    assert yaw_track_rate(-89.0, True, YT) == -YT.max_yaw_rate_dps


def test_follow_geometry() -> None:
    tgt = TargetEstimate(valid=True, n=30.0, e=0.0, vn=0.0, ve=0.0)
    c = follow_velocity(0.0, 0.0, -10.0, 0.0, tgt, FO)
    close(c.range_m, 30.0)
    close(c.bearing_deg, 0.0)
    assert c.vn > 0 and abs(c.ve) < 1e-9  # closes toward target
    assert math.hypot(c.vn, c.ve) <= FO.v_max_mps + 1e-9
    close(c.vd, 0.0)  # already at 10 m
    tgt = TargetEstimate(valid=True, n=12.5, e=0.0, vn=0.0, ve=0.0)
    c = follow_velocity(0.0, 0.0, -5.0, 0.0, tgt, FO)
    close(c.vn, 0.0)  # inside deadband
    assert c.vd < 0  # climb (negative D velocity) to 10 m
    tgt = TargetEstimate(valid=True, n=5.0, e=0.0, vn=0.0, ve=0.0)
    assert follow_velocity(0, 0, -10, 0, tgt, FO).vn < 0  # too close: back away
    tgt = TargetEstimate(valid=True, n=12.0, e=0.0, vn=0.0, ve=1.0)
    close(follow_velocity(0, 0, -10, 0, tgt, FO).ve, FO.ff_gain * 1.0)  # feed-forward


def test_rate_limit_yaw_wraps() -> None:
    close(rate_limit_yaw(170.0, -170.0, 30.0, 0.5), -175.0)
    close(rate_limit_yaw(0.0, 90.0, 30.0, 1.0), 30.0)


class FakeActuator:
    """Records every vehicle side effect the core requests."""

    def __init__(self) -> None:
        self.calls: list[tuple[Any, ...]] = []

    def set_mode(self, main: int, sub: int = 0) -> None:
        self.calls.append(("mode", main, sub))

    def arm(self, value: bool) -> None:
        self.calls.append(("arm", value))

    def send_position_setpoint(self, n: float, e: float, d: float, yaw_rad: float) -> None:
        self.calls.append(("sp", "pos", (n, e, d), (0.0, 0.0, 0.0), yaw_rad))

    def send_velocity_setpoint(
        self,
        vn: float,
        ve: float,
        vd: float,
        yaw_rad: float | None = None,
        yaw_rate_rad: float | None = None,
    ) -> None:
        self.calls.append(("sp", "vel", (0.0, 0.0, 0.0), (vn, ve, vd), yaw_rad))

    def modes(self) -> list[tuple[int, int]]:
        return [(c[1], c[2]) for c in self.calls if c[0] == "mode"]

    def arms(self) -> list[bool]:
        return [c[1] for c in self.calls if c[0] == "arm"]

    def setpoints(self) -> list[tuple[Any, ...]]:
        return [c for c in self.calls if c[0] == "sp"]


class Msg:
    """Duck-typed pymavlink message from PX4 (1/1)."""

    def __init__(self, typ: str, **kw: Any) -> None:
        self._t = typ
        self.__dict__.update(kw)

    def get_type(self) -> str:
        return self._t

    def get_srcSystem(self) -> int:
        return 1

    def get_srcComponent(self) -> int:
        return 1


def make_state(
    armed: bool = False,
    main: int = px.MAIN_POSCTL,
    landed: int = px.LANDED_ON_GROUND,
    enable: bool = True,
    n: float = 0.0,
    e: float = 0.0,
    d: float = 0.0,
    batt: int = 80,
) -> VehicleSnapshot:
    st = VehicleState()
    cm = main << 16
    st.handle(Msg("HEARTBEAT", base_mode=128 if armed else 0, custom_mode=cm))
    st.handle(Msg("LOCAL_POSITION_NED", x=n, y=e, z=d, vx=0, vy=0, vz=0))
    st.handle(Msg("GPS_RAW_INT", fix_type=3, satellites_visible=20, eph=80, epv=120))
    st.handle(
        Msg(
            "SYS_STATUS",
            onboard_control_sensors_present=0,
            onboard_control_sensors_enabled=0,
            onboard_control_sensors_health=0,
            battery_remaining=batt,
            voltage_battery=16000,
        )
    )
    st.handle(Msg("EXTENDED_SYS_STATE", landed_state=landed, vtol_state=0))
    st.handle(Msg("ATTITUDE", roll=0.0, pitch=0.0, yaw=math.radians(45.0)))
    chans = {f"chan{i}_raw": 1500 for i in range(1, 19)}
    chans[f"chan{CFG.enable_channel}_raw"] = 2000 if enable else 1000
    st.handle(Msg("RC_CHANNELS", chancount=8, rssi=100, **chans))
    return st.snapshot()


def run(
    sup: SupervisorCore, st: VehicleSnapshot, m: FakeActuator, seconds: float, dt: float = 0.05
) -> None:
    t = time.time()
    for i in range(int(seconds / dt)):
        sup.step(st, m, t + i * dt)
        sup.stream(m, t + i * dt)


def test_supervisor_nominal_takeoff_and_pilot_override() -> None:
    m = FakeActuator()
    sup = SupervisorCore(CFG, GCFG)
    st = make_state()
    assert sup.takeoff_cmd(st) is None
    assert sup.state == "PREFLIGHT"
    run(sup, st, m, 0.1)
    assert sup.state == "STREAMING", sup.reason
    run(sup, st, m, CFG.prestream_s + 0.2)
    assert sup.state == "OFFBOARD_REQ"
    assert (px.MAIN_OFFBOARD, 0) in m.modes()  # OFFBOARD requested
    n_sp = len(m.setpoints())
    assert n_sp >= CFG.setpoint_hz * CFG.prestream_s * 0.8  # streamed before the request
    st = make_state(main=px.MAIN_OFFBOARD)
    run(sup, st, m, 0.1)
    assert sup.state == "ARMING" and m.arms() == [True]
    st = make_state(armed=True, main=px.MAIN_OFFBOARD, landed=px.LANDED_IN_AIR)
    run(sup, st, m, 0.1)
    assert sup.state == "TAKEOFF"
    st = make_state(
        armed=True, main=px.MAIN_OFFBOARD, landed=px.LANDED_IN_AIR, d=-CFG.takeoff_alt_m
    )
    run(sup, st, m, CFG.takeoff_alt_m / CFG.climb_rate_mps + 1.0)
    assert sup.state == "HOVER", sup.reason
    sp = m.setpoints()[-1]
    close(sp[2][2], -CFG.takeoff_alt_m)  # D setpoint = -3 m
    # Pilot flicks to Position: supervisor stops without commanding any mode.
    before = len(m.modes())
    st = make_state(armed=True, main=px.MAIN_POSCTL, landed=px.LANDED_IN_AIR, d=-3.0)
    run(sup, st, m, 0.1)
    assert sup.state == "IDLE" and "PILOT_OVERRIDE" in sup.reason
    assert len(m.modes()) == before and sup.sp is None


def test_supervisor_abort_on_enable_switch_and_geofence() -> None:
    m = FakeActuator()
    sup = SupervisorCore(CFG, GCFG)
    sup.state, sup.entered_offboard = "HOVER", True
    sup.takeoff = TakeoffPoint(n=0.0, e=0.0, d0=0.0, yaw=0.0)
    sup.yaw_cmd = 0.0
    st = make_state(
        armed=True, main=px.MAIN_OFFBOARD, landed=px.LANDED_IN_AIR, d=-3.0, enable=False
    )
    run(sup, st, m, 0.1)
    assert sup.state == "ABORT" and "enable" in sup.reason
    assert (px.MAIN_AUTO, px.SUB_AUTO_LOITER) in m.modes(), m.calls
    sup2 = SupervisorCore(CFG, GCFG)
    sup2.state, sup2.entered_offboard = "HOVER", True
    sup2.takeoff = TakeoffPoint(n=0.0, e=0.0, d0=0.0, yaw=0.0)
    sup2.yaw_cmd = 0.0
    st = make_state(
        armed=True,
        main=px.MAIN_OFFBOARD,
        landed=px.LANDED_IN_AIR,
        n=CFG.geofence_radius_m + 1,
        d=-3.0,
    )
    run(sup2, st, m, 0.1)
    assert sup2.state == "ABORT" and "geofence" in sup2.reason


def test_supervisor_preflight_refuses_without_enable() -> None:
    m = FakeActuator()
    sup = SupervisorCore(CFG, GCFG)
    st = make_state(enable=False)
    assert sup.takeoff_cmd(st) is None
    run(sup, st, m, 0.5)
    assert sup.state == "PREFLIGHT" and "enable switch" in sup.reason
    assert not m.setpoints()  # nothing streamed


def test_supervisor_yaw_track_and_follow_loss() -> None:
    m = FakeActuator()
    sup = SupervisorCore(CFG, GCFG)
    sup.state, sup.entered_offboard = "HOVER", True
    sup.takeoff = TakeoffPoint(n=0.0, e=0.0, d0=0.0, yaw=0.0)
    sup.yaw_cmd = 0.0
    st = make_state(armed=True, main=px.MAIN_OFFBOARD, landed=px.LANDED_IN_AIR, d=-3.0)
    assert sup.set_guidance_mode("YAW_TRACK") is None
    run(sup, st, m, CFG.hover_settle_s + 0.2)
    assert sup.state == "YAW_TRACK"
    sup.target = TargetEstimate(valid=False, los_valid=True, gimbal_yaw_body_deg=60.0)
    sup.target_rx_t = time.time()
    y0 = sup.yaw_cmd
    t = time.time()
    for i in range(21):
        sup.step(st, m, t + i * 0.05)  # 1 s at 20 Hz
    expected = min(YT.max_yaw_rate_dps, YT.k_yaw * (60.0 - YT.deadband_deg))
    assert 0.6 * expected <= sup.yaw_cmd - y0 <= 1.2 * expected, (sup.yaw_cmd - y0, expected)
    # FOLLOW with a fresh target, then loss -> hold, then long loss -> HOVER.
    assert sup.set_guidance_mode("FOLLOW") is None
    sup.step(st, m, time.time())
    assert sup.state == "FOLLOW" and sup.sp is not None and sup.sp.kind == "pos"
    sup.target = TargetEstimate(valid=True, n=30.0, e=0.0, vn=0.0, ve=0.0)
    sup.target_rx_t = time.time()
    sup.step(st, m, time.time())
    assert sup.state == "FOLLOW"
    sup.step(st, m, time.time())
    assert sup.sp is not None and sup.sp.kind == "vel" and sup.sp.vn > 0
    # Estimator keeps sending packets, but they are invalid: age counts from the last VALID one.
    sup.target = TargetEstimate(valid=False, reason="no measurement")
    sup.target_rx_t = time.time()
    sup.target_valid_t = time.time() - 2.0
    sup.step(st, m, time.time())
    assert sup.sp is not None and sup.sp.kind == "pos" and "holding" in sup.reason
    sup.target_valid_t = time.time() - 6.0
    sup.state_since = time.time() - 10.0  # entered FOLLOW long ago
    sup.step(st, m, time.time())
    assert sup.state == "HOVER" and "lost" in sup.reason, (sup.state, sup.reason)
    assert sup.hover is not None
    assert abs(sup.hover.d - (-3.0)) < 1e-6  # holds current altitude, no descent to takeoff alt
