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

"""CommandTracker scoring on synthetic streams. No aircraft, no simulator, no transports."""

from __future__ import annotations

from collections.abc import Iterator
import math
from typing import Any

import pytest

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.px4_msgs.CommandEvent import CommandEvent
from dimos.msgs.px4_msgs.VehicleStatus import VehicleStatus
from dimos.robot.px4.command_tracker import CommandTracker
from dimos.robot.px4.mavlink import LANDED_IN_AIR, LANDED_ON_GROUND, MAIN_OFFBOARD, MAIN_POSCTL
from dimos.robot.px4.supervisor_core import Rejection

T0 = 1_700_000_000.0

_FLIGHT_FACING = {
    "cmd_vel",
    "gimbal_target",
    "estop_in",
    "target_state",
    "target_valid",
    "target_los",
    "offboard_setpoint",
}
_FORBIDDEN_RPCS = {
    "arm",
    "set_mode",
    "takeoff",
    "land",
    "hold",
    "estop",
    "send_position_setpoint",
    "send_velocity_setpoint",
}


@pytest.fixture
def tracker() -> Iterator[CommandTracker]:
    t = CommandTracker()
    # Outputs are exercised without transports: swap them for recorders.
    published: list[Any] = []
    for name in ("tracked_command", "command_report", "cmd_forward", "meas_forward"):
        getattr(t, name).publish = published.append
    yield t
    t.stop()


def _vehicle(
    armed: bool = True, landed: int = LANDED_IN_AIR, mode: int = MAIN_OFFBOARD
) -> VehicleStatus:
    return VehicleStatus(armed=armed, landed_state=landed, main_mode=mode, ts=T0)


def _event(
    command: str = "cmd_vel",
    argument: str = "1.00,0.00,0.00,0.00",
    accepted: bool = True,
    rejection: str = "",
    ts: float = T0,
    verdict_ts: float = T0 + 0.010,
    request_id: int = 1,
) -> CommandEvent:
    return CommandEvent(
        request_id=request_id,
        command=command,
        argument=argument,
        source="cmd_vel" if command.startswith("cmd_vel") else "rpc",
        verdict_ts=verdict_ts,
        accepted=accepted,
        rejection=rejection,
        state_before="TELEOP",
        state_after="TELEOP",
        ts=ts,
    )


def _odom(
    ts: float, vx: float = 0.0, vy: float = 0.0, vz: float = 0.0, yaw: float = 0.0
) -> Odometry:
    return Odometry(
        ts=ts,
        frame_id="odom",
        child_frame_id="base_link",
        pose=Pose(Vector3(), Quaternion.from_euler(Vector3(0.0, 0.0, yaw))),
        twist=Twist(Vector3(vx, vy, vz), Vector3()),
    )


def test_read_only_surface(tracker: CommandTracker) -> None:
    assert not (set(tracker.outputs) & _FLIGHT_FACING)
    assert not (set(tracker.rpcs) & _FORBIDDEN_RPCS)
    assert {"recent", "by_verdict", "summary"} <= set(tracker.rpcs)


@pytest.mark.parametrize("reason", list(Rejection))
def test_rejected_command_carries_the_enum_value(
    tracker: CommandTracker, reason: Rejection
) -> None:
    tracker._on_command_event(_event(accepted=False, rejection=reason.value), now=T0 + 0.02)
    (tc,) = tracker.recent()
    assert tc["verdict"] == "rejected"
    assert tc["rejection"] == reason.value
    assert tc["mux_ms"] == pytest.approx(10.0)
    assert tc["onboard_ms"] is None and tc["response_ms"] is None
    assert tracker.summary()["rejections"][reason.value] == 1


def test_takeoff_is_scored_on_the_climb_not_the_teleop_window(tracker: CommandTracker) -> None:
    # On the ground, disarmed: exactly what a takeoff looks like when it is requested.
    tracker._on_vehicle_status(_vehicle(armed=False, landed=LANDED_ON_GROUND, mode=MAIN_POSCTL))
    tracker._on_command_event(_event(command="takeoff", argument=""), now=T0 + 0.011)
    tracker._on_offboard_setpoint(_odom(T0 + 0.05))  # position setpoints, prestream
    tracker.sweep(now=T0 + 3.0)  # past the teleop window: still open, no verdict yet
    assert tracker.recent() == []
    tracker._on_odometry(_odom(T0 + 3.5, vz=0.7))  # the climb
    tracker.sweep(now=T0 + 3.6)
    (tc,) = tracker.recent()
    assert tc["command"] == "takeoff" and tc["verdict"] == "ok"
    assert tc["response_ms"] == pytest.approx(3450.0)


def test_takeoff_that_never_climbs_is_no_motion(tracker: CommandTracker) -> None:
    tracker._on_vehicle_status(_vehicle(armed=False, landed=LANDED_ON_GROUND, mode=MAIN_POSCTL))
    tracker._on_command_event(_event(command="takeoff", argument=""), now=T0 + 0.011)
    tracker._on_offboard_setpoint(_odom(T0 + 0.05))
    tracker.sweep(now=T0 + 16.0)
    (tc,) = tracker.recent()
    assert tc["verdict"] == "no_motion"


def test_accepted_teleop_completes_with_three_segments(tracker: CommandTracker) -> None:
    tracker._on_vehicle_status(_vehicle())
    tracker._on_command_event(_event(), now=T0 + 0.011)
    tracker._on_odometry(_odom(T0 + 0.02))  # before the setpoint: not a response
    tracker._on_offboard_setpoint(_odom(T0 + 0.050, vx=1.0))
    tracker._on_offboard_setpoint(_odom(T0 + 0.100, vx=1.0))
    tracker._on_odometry(_odom(T0 + 0.200, vx=0.1))  # below threshold
    tracker._on_odometry(_odom(T0 + 0.300, vx=0.5))  # crosses 0.3 * 1.0
    tracker._on_command_event(
        _event(command="cmd_vel_release", ts=T0 + 2.0, request_id=2), now=T0 + 2.0
    )
    (tc,) = tracker.recent()
    assert tc["verdict"] == "ok" and tc["rejection"] == ""
    assert tc["mux_ms"] == pytest.approx(10.0)
    assert tc["onboard_ms"] == pytest.approx(40.0)
    assert tc["response_ms"] == pytest.approx(250.0)
    assert tc["setpoint_count"] == 2
    assert tc["setpoint_gap_max_ms"] == pytest.approx(50.0)
    assert tc["measured_peak"] == pytest.approx(0.5)
    assert tc["duration_s"] == pytest.approx(2.0)
    assert tc["state_before"] == "TELEOP"


def test_response_uses_the_commanded_body_axis(tracker: CommandTracker) -> None:
    # Vehicle heading east (yaw -90 deg in FLU); "left" in the body is north (+x world).
    tracker._on_vehicle_status(_vehicle())
    tracker._on_command_event(_event(argument="0.00,1.00,0.00,0.00"), now=T0 + 0.011)
    yaw = -math.pi / 2
    tracker._on_offboard_setpoint(_odom(T0 + 0.05, vx=1.0, yaw=yaw))
    tracker._on_odometry(_odom(T0 + 0.30, vx=0.6, yaw=yaw))
    tracker.sweep(now=T0 + 5.0)
    (tc,) = tracker.recent()
    assert tc["verdict"] == "ok"
    assert tc["measured_peak"] == pytest.approx(0.6, abs=1e-9)


def test_no_setpoint_when_nothing_is_streamed(tracker: CommandTracker) -> None:
    tracker._on_vehicle_status(_vehicle())
    tracker._on_command_event(_event(), now=T0 + 0.011)
    tracker.sweep(now=T0 + 0.5)
    assert tracker.recent() == []
    tracker.sweep(now=T0 + 1.2)
    (tc,) = tracker.recent()
    assert tc["verdict"] == "no_setpoint" and tc["onboard_ms"] is None


def test_mode_not_offboard_beats_no_motion(tracker: CommandTracker) -> None:
    tracker._on_vehicle_status(_vehicle(mode=MAIN_POSCTL))
    tracker._on_command_event(_event(), now=T0 + 0.011)
    tracker._on_offboard_setpoint(_odom(T0 + 0.05, vx=1.0))
    tracker.sweep(now=T0 + 3.0)
    (tc,) = tracker.recent()
    assert tc["verdict"] == "mode_not_offboard"


def test_no_motion_when_airborne_in_offboard_but_nothing_moves(tracker: CommandTracker) -> None:
    tracker._on_vehicle_status(_vehicle())
    tracker._on_command_event(_event(), now=T0 + 0.011)
    tracker._on_offboard_setpoint(_odom(T0 + 0.05, vx=1.0))
    tracker._on_odometry(_odom(T0 + 0.5, vx=0.05))
    tracker.sweep(now=T0 + 3.0)
    (tc,) = tracker.recent()
    assert tc["verdict"] == "no_motion"
    assert tc["measured_peak"] == pytest.approx(0.05)


def test_not_expected_to_move_on_the_ground(tracker: CommandTracker) -> None:
    tracker._on_vehicle_status(_vehicle(armed=False, landed=LANDED_ON_GROUND))
    tracker._on_command_event(_event(), now=T0 + 0.011)
    tracker._on_offboard_setpoint(_odom(T0 + 0.05, vx=1.0))
    tracker.sweep(now=T0 + 3.0)
    (tc,) = tracker.recent()
    assert tc["verdict"] == "not_expected_to_move"
    assert tc["onboard_ms"] == pytest.approx(40.0)  # t_commanded is still populated


def test_clamped_when_the_setpoint_is_below_the_request(tracker: CommandTracker) -> None:
    tracker._on_vehicle_status(_vehicle())
    tracker._on_command_event(_event(argument="5.00,0.00,0.00,0.00"), now=T0 + 0.011)
    tracker._on_offboard_setpoint(_odom(T0 + 0.05, vx=1.5))
    tracker._on_odometry(_odom(T0 + 0.4, vx=0.6))
    tracker.sweep(now=T0 + 3.0)
    (tc,) = tracker.recent()
    assert tc["verdict"] == "clamped"
    assert tc["clamp_ratio"] == pytest.approx(0.3)


def test_stop_command_is_ok_once_the_stream_ends(tracker: CommandTracker) -> None:
    tracker._on_vehicle_status(_vehicle())
    tracker._on_command_event(_event(command="land", argument=""), now=T0 + 0.011)
    tracker._on_offboard_setpoint(_odom(T0 + 0.02, vx=0.3))  # last one the tick sent
    tracker.sweep(now=T0 + 0.1)
    assert tracker.recent() == []
    tracker.sweep(now=T0 + 0.5)
    (tc,) = tracker.recent()
    assert tc["verdict"] == "ok"
    assert tc["commanded_ts"] == pytest.approx(T0 + 0.02)


def test_summary_counts_and_percentiles(tracker: CommandTracker) -> None:
    tracker._on_vehicle_status(_vehicle())
    tracker._on_command_event(_event(accepted=False, rejection="not_teleop", request_id=1), now=T0)
    tracker._on_command_event(
        _event(command="set_guidance_mode", argument="TELEOP", request_id=2), now=T0
    )
    s = tracker.summary()
    assert s["total"] == 2 and s["open"] == 0
    assert s["verdicts"] == {"rejected": 1, "ok": 1}
    assert s["rejections"]["not_teleop"] == 1
    assert s["latency_ms"]["mux"]["n"] == 2 and s["latency_ms"]["response"]["n"] == 0
