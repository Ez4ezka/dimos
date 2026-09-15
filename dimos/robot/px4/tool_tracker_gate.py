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

"""N1 gate: four scripted operator commands through the real transports, no simulator.

A fake connection module replays what Px4DroneConnection would publish for the
kickoff's sequence: W held 2 s, A tapped, W while in HOVER, W while E-STOP is latched.
CommandTracker runs beside it through the coordinator, and the gate expects exactly four
scored events with verdicts ok, ok, rejected/not_teleop, rejected/estop_latched, every
latency segment populated and non-negative on the accepted ones::

    python dimos/robot/px4/tool_tracker_gate.py
"""

from __future__ import annotations

import math
import threading
import time
from typing import Any

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import Out
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.px4_msgs.CommandEvent import CommandEvent
from dimos.msgs.px4_msgs.TrackedCommand import TrackedCommand
from dimos.msgs.px4_msgs.VehicleStatus import VehicleStatus
from dimos.msgs.std_msgs.String import String
from dimos.robot.px4.blueprints import px4_transports
from dimos.robot.px4.command_tracker import CommandTracker
from dimos.robot.px4.mavlink import LANDED_IN_AIR, MAIN_OFFBOARD

EXPECTED = [("ok", ""), ("ok", ""), ("rejected", "not_teleop"), ("rejected", "estop_latched")]


def _odom(ts: float, vx: float = 0.0, vy: float = 0.0) -> Odometry:
    return Odometry(
        ts=ts,
        frame_id="odom",
        child_frame_id="base_link",
        pose=Pose(Vector3(), Quaternion()),
        twist=Twist(Vector3(vx, vy, 0.0), Vector3()),
    )


class FakePx4Feed(Module):
    """Replays the connection's streams for the four scripted commands."""

    command_event: Out[CommandEvent]
    offboard_setpoint: Out[Odometry]
    odometry: Out[Odometry]
    vehicle_status: Out[VehicleStatus]
    supervisor_state: Out[String]
    supervisor_status: Out[String]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._thread: threading.Thread | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self._thread = threading.Thread(target=self._script, name="fake-px4-feed", daemon=True)
        self._thread.start()

    @rpc
    def stop(self) -> None:
        if self._thread is not None:
            self._thread.join(timeout=10.0)
        super().stop()

    def _event(self, rid: int, arg: str, accepted: bool, rejection: str, state: str) -> None:
        t = time.time()
        self.command_event.publish(
            CommandEvent(
                request_id=rid,
                command="cmd_vel",
                argument=arg,
                source="cmd_vel",
                verdict_ts=t + 0.004,
                accepted=accepted,
                rejection=rejection,
                state_before=state,
                state_after=state,
                ts=t,
            )
        )

    def _release(self, rid: int) -> None:
        t = time.time()
        self.command_event.publish(
            CommandEvent(
                request_id=rid,
                command="cmd_vel_release",
                argument="0.00,0.00,0.00,0.00",
                source="cmd_vel",
                verdict_ts=t,
                accepted=True,
                state_before="TELEOP",
                state_after="TELEOP",
                ts=t,
            )
        )

    def _held_key(self, rid: int, axis_vx: float, axis_vy: float, seconds: float) -> None:
        arg = f"{axis_vx:.2f},{axis_vy:.2f},0.00,0.00"
        self._event(rid, arg, True, "", "TELEOP")
        t0 = time.time()
        tick = 0
        while time.time() - t0 < seconds:
            now = time.time()
            self.offboard_setpoint.publish(_odom(now, axis_vx, axis_vy))
            # The airframe ramps to the commanded speed over half a second.
            frac = min(1.0, (now - t0) / 0.5)
            if tick % 2 == 0:
                self.odometry.publish(_odom(now, axis_vx * frac, axis_vy * frac))
            tick += 1
            time.sleep(0.05)
        self._release(rid + 100)
        # Controller lag: the airframe keeps accelerating for a moment after the last
        # setpoint, then decays. A short tap peaks after the key has come up.
        for i in range(6):
            f = min(1.0, frac + 0.12 * (i + 1)) if i < 3 else max(0.0, frac + 0.36 - 0.3 * (i - 2))
            self.odometry.publish(_odom(time.time(), axis_vx * f, axis_vy * f))
            time.sleep(0.05)
        self.odometry.publish(_odom(time.time()))

    def _script(self) -> None:
        time.sleep(1.0)  # let the tracker subscribe
        self.vehicle_status.publish(
            VehicleStatus(
                armed=True, landed_state=LANDED_IN_AIR, main_mode=MAIN_OFFBOARD, state="TELEOP"
            )
        )
        self.supervisor_state.publish(String("TELEOP"))
        time.sleep(0.2)
        self._held_key(1, 1.0, 0.0, 2.0)  # W held 2 s
        time.sleep(0.5)
        self._held_key(2, 0.0, 1.0, 0.15)  # A tapped
        time.sleep(0.5)
        self.supervisor_state.publish(String("HOVER"))
        self._event(3, "1.00,0.00,0.00,0.00", False, "not_teleop", "HOVER")  # W in HOVER
        time.sleep(0.5)
        self.vehicle_status.publish(
            VehicleStatus(
                armed=False, landed_state=LANDED_IN_AIR, main_mode=MAIN_OFFBOARD, estop_latched=True
            )
        )
        self._event(4, "1.00,0.00,0.00,0.00", False, "estop_latched", "IDLE")  # W while latched


def main() -> int:
    blueprint = autoconnect(FakePx4Feed.blueprint(), CommandTracker.blueprint()).transports(
        px4_transports()
    )
    parsed = BlueprintConfigParser(blueprint).parse(
        environ={}, overrides={"g": {"viewer": "none", "transport": "zenoh", "n_workers": 1}}
    )
    coordinator = ModuleCoordinator.build(blueprint, parsed)
    scored: list[TrackedCommand] = []
    lock = threading.Lock()

    def on_tracked(msg: TrackedCommand) -> None:
        with lock:
            scored.append(msg)

    ok = True
    try:
        unsub = coordinator.transports[("tracked_command", TrackedCommand)].subscribe(on_tracked)
        deadline = time.time() + 20.0
        while time.time() < deadline:
            with lock:
                if len(scored) >= len(EXPECTED):
                    break
            time.sleep(0.2)
        unsub()
        with lock:
            events = sorted(scored, key=lambda tc: tc.event_id)
        for tc in events:
            print(tc.report_line())
        got = [(tc.verdict, tc.rejection) for tc in events]
        print("verdicts:", got)
        ok &= got == EXPECTED
        for tc in events[:2]:
            segs = (tc.mux_ms, tc.onboard_ms, tc.response_ms)
            ok &= all(not math.isnan(s) and s >= 0 for s in segs)
            ok &= tc.setpoint_count >= 2
        tracker = coordinator.get_instance(CommandTracker)
        print("summary:", tracker.summary())
    finally:
        coordinator.stop()
    print("GATE", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
