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

"""CommandTracker: did that operator command take effect, and if not, why not.

Read-only. It taps the connection's streams and exposes query RPCs. It never commands the
aircraft, never opens a MAVLink socket, is never imported by ``connection.py``, and sits
in the path of nothing flight-critical: if it dies, flight is unaffected.

Per operator command (one ``command_event`` from the connection) it scores one
:class:`TrackedCommand`: the verdict, the SupervisorCore rejection value when refused,
the supervisor state before and after, and three latency segments measured separately:

    request  -> verdict            (both stamps are in the command_event: ``mux``)
    verdict  -> first setpoint     (``offboard_setpoint`` stamped at the write: ``onboard``)
    setpoint -> odometry response  (body speed crosses the threshold: ``response``)

The connection publishes a command_event on the motion edges of teleop, so a held key is
one event, not twenty a second. Verdicts: ``ok``, ``rejected`` (carries the enum),
``clamped``, ``no_setpoint``, ``no_motion``, ``mode_not_offboard``,
``not_expected_to_move``, ``held``. ``not_expected_to_move`` is derived from
``vehicle_status`` (disarmed or on the ground), not a config flag, so bench and flight use
the identical blueprint. ``mode_not_offboard`` is checked before ``no_motion`` so a PX4
refusal never reads as an unresponsive airframe.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
import statistics
import threading
import time
from typing import Any

from pydantic import Field
from reactivex.disposable import Disposable

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.px4_msgs.CommandEvent import CommandEvent
from dimos.msgs.px4_msgs.TrackedCommand import SCORING_VERSION, TrackedCommand
from dimos.msgs.px4_msgs.VehicleStatus import VehicleStatus
from dimos.msgs.std_msgs.Float32 import Float32
from dimos.msgs.std_msgs.String import String
from dimos.robot.px4.mavlink import LANDED_IN_AIR, LANDED_TAKEOFF, MAIN_OFFBOARD
from dimos.robot.px4.supervisor_core import Rejection
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# Commands that produce a setpoint stream when accepted, and the odometry sign that
# proves they took effect. Stop commands are scored on the stream ending instead.
_MOVE_COMMANDS = frozenset({"cmd_vel", "takeoff"})
_STOP_COMMANDS = frozenset({"land", "hold", "estop", "estop_land"})
_NO_MOTION_COMMANDS = frozenset(
    {"set_guidance_mode", "estop_clear", "sitl_enable", "cmd_vel_release"}
)
# A setpoint gap this long means PX4's Offboard-loss failsafe is about to fire.
_STREAM_ENDED_S = 0.25


class CommandTrackerConfig(ModuleConfig):
    # Measured body speed must reach this fraction of the commanded speed, and at least
    # response_min_mps, before the command counts as taken effect.
    response_frac: float = Field(default=0.3)
    response_min_mps: float = Field(default=0.15)
    # How long after the first setpoint the airframe gets to respond. Takeoff streams
    # setpoints for prestream_s, then waits for the OFFBOARD and arm acks before it climbs,
    # so it gets its own window.
    response_timeout_s: float = Field(default=1.5)
    takeoff_response_timeout_s: float = Field(default=15.0)
    # How long after the verdict a setpoint must appear.
    setpoint_timeout_s: float = Field(default=1.0)
    # A teleop key held longer than this closes as "held" rather than waiting forever.
    event_max_s: float = Field(default=10.0)
    # Commanded speed below this fraction of the request reads as "clamped".
    clamp_frac: float = Field(default=0.98)
    keep_events: int = Field(default=500)
    # Rolling window for the summary RPC's rates.
    health_window_s: float = Field(default=30.0)
    sweep_hz: float = Field(default=5.0)


@dataclass
class _Vehicle:
    armed: bool = False
    landed_state: int = -1
    main_mode: int = 0
    t: float = 0.0


@dataclass
class _Open:
    """An event still collecting evidence."""

    event: CommandEvent
    axis: str
    requested: float
    verdict_wall: float
    vehicle: _Vehicle
    commanded_ts: float = math.nan
    moved_ts: float = math.nan
    commanded_peak: float = 0.0
    measured_peak: float = 0.0
    setpoint_count: int = 0
    last_setpoint_ts: float = math.nan
    setpoint_gap_max_ms: float = 0.0
    closed_ts: float = math.nan


def _parse_cmd_vel(argument: str) -> tuple[str, float]:
    """Dominant axis and its requested magnitude from the connection's argument string."""
    try:
        fwd, left, up, yaw = (float(v) for v in argument.split(","))
    except ValueError:
        return "forward", 0.0
    axes = {"forward": fwd, "left": left, "up": up, "yaw": yaw}
    axis = max(axes, key=lambda k: abs(axes[k]))
    return axis, axes[axis]


def _body_component(msg: Odometry, axis: str) -> float:
    """Signed velocity of an FLU-frame odometry or setpoint along a body axis."""
    yaw = msg.orientation.to_euler().z
    vx, vy, vz = msg.vx, msg.vy, msg.twist.linear.z
    c, s = math.cos(yaw), math.sin(yaw)
    if axis == "forward":
        return vx * c + vy * s
    if axis == "left":
        return -vx * s + vy * c
    if axis == "up":
        return vz
    return msg.twist.angular.z


class CommandTracker(Module):
    """Scores every operator command from the connection's streams. Read-only."""

    config: CommandTrackerConfig

    command_event: In[CommandEvent]
    offboard_setpoint: In[Odometry]
    odometry: In[Odometry]
    vehicle_status: In[VehicleStatus]
    supervisor_state: In[String]
    supervisor_status: In[String]

    tracked_command: Out[TrackedCommand]
    command_report: Out[String]
    # Commanded versus measured body-forward speed, for a rerun scalar overlay.
    cmd_forward: Out[Float32]
    meas_forward: Out[Float32]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = threading.Lock()
        self._open: list[_Open] = []
        self._done: deque[TrackedCommand] = deque(maxlen=self.config.keep_events)
        self._vehicle = _Vehicle()
        self._last_odom: Odometry | None = None
        self._last_state = ""
        self._stop_event = threading.Event()
        self._sweep_thread: threading.Thread | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.command_event.subscribe(self._on_command_event)))
        self.register_disposable(
            Disposable(self.offboard_setpoint.subscribe(self._on_offboard_setpoint))
        )
        self.register_disposable(Disposable(self.odometry.subscribe(self._on_odometry)))
        self.register_disposable(Disposable(self.vehicle_status.subscribe(self._on_vehicle_status)))
        self.register_disposable(Disposable(self.supervisor_state.subscribe(self._on_state)))
        self._stop_event.clear()
        self._sweep_thread = threading.Thread(
            target=self._sweep_loop, name="tracker-sweep", daemon=True
        )
        self._sweep_thread.start()

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        if self._sweep_thread is not None:
            self._sweep_thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
            self._sweep_thread = None
        super().stop()

    # Stream handlers

    def _on_vehicle_status(self, msg: VehicleStatus) -> None:
        with self._lock:
            self._vehicle = _Vehicle(
                armed=msg.armed, landed_state=msg.landed_state, main_mode=msg.main_mode, t=msg.ts
            )

    def _on_state(self, msg: String) -> None:
        with self._lock:
            self._last_state = msg.data

    def _on_command_event(self, ev: CommandEvent, now: float | None = None) -> None:
        wall = time.time() if now is None else now
        with self._lock:
            if ev.command == "cmd_vel_release":
                # The key came up: the event's duration ends here, but the airframe answers
                # after the key, so the response window stays open until the sweep scores it.
                for o in self._open:
                    if o.event.command == "cmd_vel" and math.isnan(o.closed_ts):
                        o.closed_ts = ev.ts
                        verdict = self._score(o, wall)
                        if verdict is not None:
                            self._close(o, verdict, wall)
                return
            axis, requested = ("forward", 0.0)
            if ev.command == "cmd_vel":
                axis, requested = _parse_cmd_vel(ev.argument)
            elif ev.command == "takeoff":
                axis, requested = "up", math.inf
            elif ev.command in _STOP_COMMANDS:
                axis, requested = "forward", 0.0
            o = _Open(
                event=ev,
                axis=axis,
                requested=abs(requested),
                verdict_wall=wall,
                vehicle=self._vehicle,
            )
            if not ev.accepted:
                self._close(o, "rejected", wall)
                return
            if ev.command in _NO_MOTION_COMMANDS:
                self._close(o, "ok", wall)
                return
            self._open.append(o)

    def _on_offboard_setpoint(self, sp: Odometry) -> None:
        with self._lock:
            commanded = _body_component(sp, "forward")
            self.cmd_forward.publish(Float32(commanded))
            for o in self._open:
                if sp.ts < o.event.verdict_ts:
                    continue
                if not math.isnan(o.last_setpoint_ts):
                    o.setpoint_gap_max_ms = max(
                        o.setpoint_gap_max_ms, (sp.ts - o.last_setpoint_ts) * 1e3
                    )
                o.last_setpoint_ts = sp.ts
                o.setpoint_count += 1
                if o.event.command in _STOP_COMMANDS:
                    continue
                if math.isnan(o.commanded_ts):
                    o.commanded_ts = sp.ts
                o.commanded_peak = max(o.commanded_peak, abs(_body_component(sp, o.axis)))

    def _on_odometry(self, od: Odometry) -> None:
        with self._lock:
            self._last_odom = od
            self.meas_forward.publish(Float32(_body_component(od, "forward")))
            cfg = self.config
            for o in self._open:
                if math.isnan(o.commanded_ts) or od.ts < o.commanded_ts:
                    continue
                measured = abs(_body_component(od, o.axis))
                o.measured_peak = max(o.measured_peak, measured)
                threshold = max(
                    cfg.response_min_mps, cfg.response_frac * min(o.requested, o.commanded_peak)
                )
                if math.isnan(o.moved_ts) and measured >= threshold:
                    o.moved_ts = od.ts

    # Scoring

    def _sweep_loop(self) -> None:
        period = 1.0 / self.config.sweep_hz
        while not self._stop_event.wait(period):
            self.sweep()

    def sweep(self, now: float | None = None) -> None:
        """Close open events whose evidence window has passed. Called by the sweep thread."""
        wall = time.time() if now is None else now
        with self._lock:
            for o in list(self._open):
                verdict = self._score(o, wall)
                if verdict is not None:
                    self._close(o, verdict, wall)

    def _score(self, o: _Open, wall: float) -> str | None:
        """Verdict for an open event, or None while evidence can still arrive."""
        cfg = self.config
        ev = o.event
        if ev.command in _STOP_COMMANDS:
            # Taking effect means the setpoint stream ended.
            if math.isnan(o.last_setpoint_ts):
                return "ok" if wall - o.verdict_wall >= _STREAM_ENDED_S else None
            if wall - o.last_setpoint_ts >= _STREAM_ENDED_S:
                o.commanded_ts = o.last_setpoint_ts
                return "ok"
            return "no_setpoint" if wall - o.verdict_wall > cfg.setpoint_timeout_s else None
        if not math.isnan(o.moved_ts):
            if (
                o.requested > 0
                and not math.isinf(o.requested)
                and o.commanded_peak < cfg.clamp_frac * o.requested
            ):
                return "clamped"
            return "ok"
        if math.isnan(o.commanded_ts):
            return "no_setpoint" if wall - o.verdict_wall > cfg.setpoint_timeout_s else None
        takeoff = ev.command == "takeoff"
        timeout = cfg.takeoff_response_timeout_s if takeoff else cfg.response_timeout_s
        if wall - o.commanded_ts > timeout:
            v = o.vehicle
            airborne = v.landed_state in (-1, LANDED_IN_AIR, LANDED_TAKEOFF)
            # A takeoff starts disarmed on the ground by definition; only teleop is judged
            # on the vehicle state at the time of the command.
            if not takeoff and (not v.armed or not airborne):
                return "not_expected_to_move"
            if not takeoff and v.main_mode != MAIN_OFFBOARD:
                return "mode_not_offboard"
            return "no_motion"
        if (
            ev.command == "cmd_vel"
            and math.isnan(o.closed_ts)
            and wall - o.verdict_wall > cfg.event_max_s
        ):
            return "held"
        return None

    def _close(self, o: _Open, verdict: str, wall: float) -> None:
        if o in self._open:
            self._open.remove(o)
        ev = o.event
        mux = (ev.verdict_ts - ev.ts) * 1e3
        onboard = (
            (o.commanded_ts - ev.verdict_ts) * 1e3 if not math.isnan(o.commanded_ts) else math.nan
        )
        response = (o.moved_ts - o.commanded_ts) * 1e3 if not math.isnan(o.moved_ts) else math.nan
        clamp = (
            o.commanded_peak / o.requested
            if o.requested > 0 and not math.isinf(o.requested) and o.commanded_peak > 0
            else math.nan
        )
        end = o.closed_ts if not math.isnan(o.closed_ts) else wall
        tc = TrackedCommand(
            event_id=ev.request_id,
            command=ev.command,
            argument=ev.argument,
            source=ev.source,
            verdict=verdict,
            rejection=ev.rejection if verdict == "rejected" else "",
            request_ts=ev.ts,
            verdict_ts=ev.verdict_ts,
            commanded_ts=o.commanded_ts,
            moved_ts=o.moved_ts,
            mux_ms=mux,
            onboard_ms=onboard,
            response_ms=response,
            link_ms=math.nan,
            setpoint_count=o.setpoint_count,
            setpoint_gap_max_ms=o.setpoint_gap_max_ms if o.setpoint_count > 1 else math.nan,
            commanded_peak=o.commanded_peak if o.setpoint_count else math.nan,
            measured_peak=o.measured_peak if not math.isnan(o.commanded_ts) else math.nan,
            clamp_ratio=clamp,
            duration_s=end - ev.ts,
            scoring_version=SCORING_VERSION,
            state_before=ev.state_before,
            state_after=ev.state_after,
            ts=wall,
        )
        self._done.append(tc)
        self.tracked_command.publish(tc)
        self.command_report.publish(String(tc.report_line()))

    # Query RPCs. Nothing here can touch the aircraft.

    @rpc
    def recent(self, n: int = 20) -> list[dict[str, Any]]:
        """The last ``n`` scored commands, newest last."""
        with self._lock:
            return [_as_dict(tc) for tc in list(self._done)[-n:]]

    @rpc
    def by_verdict(self, verdict: str) -> list[dict[str, Any]]:
        """Scored commands with this verdict (``ok``, ``rejected``, ``no_motion``, ...)."""
        with self._lock:
            return [_as_dict(tc) for tc in self._done if tc.verdict == verdict]

    @rpc
    def summary(self) -> dict[str, Any]:
        """Counts per verdict and per rejection reason, latency percentiles, open events."""
        with self._lock:
            done = list(self._done)
            open_n = len(self._open)
        cutoff = time.time() - self.config.health_window_s
        recent = [tc for tc in done if tc.ts >= cutoff]
        verdicts: dict[str, int] = {}
        rejections: dict[str, int] = {r.value: 0 for r in Rejection}
        for tc in done:
            verdicts[tc.verdict] = verdicts.get(tc.verdict, 0) + 1
            if tc.rejection:
                rejections[tc.rejection] = rejections.get(tc.rejection, 0) + 1
        return {
            "total": len(done),
            "open": open_n,
            "recent_window_s": self.config.health_window_s,
            "recent": len(recent),
            "verdicts": verdicts,
            "rejections": rejections,
            "latency_ms": {
                "mux": _percentiles([tc.mux_ms for tc in done]),
                "onboard": _percentiles([tc.onboard_ms for tc in done]),
                "response": _percentiles([tc.response_ms for tc in done]),
            },
            "setpoint_gap_max_ms": max(
                (tc.setpoint_gap_max_ms for tc in done if not math.isnan(tc.setpoint_gap_max_ms)),
                default=None,
            ),
        }


def _as_dict(tc: TrackedCommand) -> dict[str, Any]:
    return {k: (None if isinstance(v, float) and math.isnan(v) else v) for k, v in vars(tc).items()}


def _percentiles(values: list[float]) -> dict[str, float | None]:
    xs = sorted(v for v in values if not math.isnan(v))
    if not xs:
        return {"p50": None, "p90": None, "max": None, "n": 0}
    return {
        "p50": statistics.median(xs),
        "p90": xs[min(len(xs) - 1, int(0.9 * len(xs)))],
        "max": xs[-1],
        "n": len(xs),
    }
