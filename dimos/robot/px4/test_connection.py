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

"""Px4DroneConnection shell: the port contract, the RPC surface, start refusal, stop order.

No aircraft: the module is constructed but never started against a socket (nothing
opens in ``__init__``), and the gimbal path is exercised through a fake MavlinkIO.
"""

from __future__ import annotations

from collections.abc import Iterator
import math
import socket
import threading
import time
from unittest.mock import MagicMock

import pytest

from dimos.msgs.px4_msgs.CommandEvent import CommandEvent
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.px4.connection import Px4DroneConnection
from dimos.robot.px4.mavlink import (
    MAIN_OFFBOARD,
    MAIN_POSCTL,
    Heartbeat,
    LocalPosition,
    VehicleSnapshot,
)
from dimos.robot.px4.supervisor_core import GotoGoal, SupervisorCore, TakeoffPoint

# The port contract (Round 1). New modules bind by exact name; renaming one breaks them.
_CONTRACT_OUT = {
    "odometry",
    "odom",
    "imu",
    "motor_outputs",
    "gps",
    "battery",
    "rc",
    "gimbal_attitude",
    "global_pose",
    "tf",
    "vehicle_status",
    "statustext",
    "supervisor_state",
    "command_event",
    "offboard_setpoint",
    "robot_state",
    "stop_movement",
}
_CONTRACT_IN = {
    "cmd_vel",
    "gimbal_target",
    "target_state",
    "target_valid",
    "target_los",
    "estop_in",
}

# Anything that could move the aircraft must not be reachable over RPC.
_FORBIDDEN_RPCS = {
    "arm",
    "disarm",
    "set_mode",
    "set_px4_mode",
    "send_position_setpoint",
    "send_velocity_setpoint",
    "send_gimbal_pitchyaw",
    "claim_gimbal_control",
    "offboard_gate_acquire",
}
_REQUIRED_RPCS = {
    "takeoff",
    "go_to",
    "land",
    "hold",
    "set_guidance_mode",
    "estop",
    "estop_land",
    "estop_clear",
    "status",
    "snapshot",
    "sensor_stats",
    "sitl_enable",
}


@pytest.fixture
def module() -> Iterator[Px4DroneConnection]:
    m = Px4DroneConnection(writer_lock_port=0)
    yield m
    m.stop()


def test_port_contract(module: Px4DroneConnection) -> None:
    assert set(module.outputs) == _CONTRACT_OUT
    assert set(module.inputs) == _CONTRACT_IN


def test_rpc_surface_has_no_actuation(module: Px4DroneConnection) -> None:
    names = set(module.rpcs)
    assert not (names & _FORBIDDEN_RPCS), names & _FORBIDDEN_RPCS
    assert _REQUIRED_RPCS <= names, _REQUIRED_RPCS - names


def test_start_refuses_when_another_writer_holds_the_lock_port() -> None:
    holder = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    holder.bind(("127.0.0.1", 0))
    port = holder.getsockname()[1]
    m = Px4DroneConnection(writer_lock_port=port)
    try:
        with pytest.raises(RuntimeError, match="another Offboard writer"):
            m.start()
    finally:
        holder.close()
        m.stop()
    assert m._io is None  # the MAVLink socket was never opened


def test_stop_joins_tick_before_heartbeat_before_publish_then_closes_io(
    module: Px4DroneConnection,
) -> None:
    module._stop_event.clear()
    threads = []
    for name in ("px4-tick", "px4-heartbeat", "px4-publish", "px4-stats"):
        t = threading.Thread(target=module._stop_event.wait, name=name, daemon=True)
        t.start()
        threads.append((name, t))
    module._threads = threads
    module._io = MagicMock()
    module._writer_lock_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    module.stop()

    assert module.stop_sequence == [
        "px4-tick",
        "px4-heartbeat",
        "px4-publish",
        "px4-stats",
        "px4-io",
        "writer-lock",
    ]
    assert all(not t.is_alive() for _, t in threads)


def _snapshot(d: float = 0.0, flying: bool = False) -> VehicleSnapshot:
    now = time.time()
    main = MAIN_OFFBOARD if flying else MAIN_POSCTL
    return VehicleSnapshot(
        wall=now,
        heartbeat=Heartbeat(flying, main << 16, main, 0, now),
        heartbeat_age=0.0,
        local=LocalPosition(0.0, 0.0, d, 0.0, 0.0, 0.0, None, now),
        local_age=0.0,
        gps=None,
        sys_status=None,
        rc=None,
        rc_age=math.inf,
        landed_state=None,
        yaw_deg=0.0,
        px4_msg_age=0.0,
    )


@pytest.fixture
def commanded(module: Px4DroneConnection) -> tuple[Px4DroneConnection, list[CommandEvent]]:
    """The module with a supervisor core and a vehicle snapshot but no socket, and its events."""
    module._core = SupervisorCore(module.config.limits, module.config.guidance)
    module._io = MagicMock()
    module._state = MagicMock()
    module._state.snapshot.return_value = _snapshot()
    events: list[CommandEvent] = []
    module.command_event.subscribe(events.append)
    return module, events


def test_takeoff_rpc_passes_the_altitude_to_the_core(
    commanded: tuple[Px4DroneConnection, list[CommandEvent]],
) -> None:
    module, events = commanded
    assert module.takeoff(2.5) == {"accepted": True, "rejection": None, "state": "PREFLIGHT"}
    assert module._core is not None and module._core.takeoff_alt_m == 2.5
    assert module.takeoff(2.0) == {
        "accepted": False,
        "rejection": "wrong_state",
        "state": "PREFLIGHT",
    }
    assert [(e.command, e.argument, e.accepted) for e in events] == [
        ("takeoff", "2.50", True),
        ("takeoff", "2.00", False),
    ]


def test_go_to_rpc_sets_the_goal_and_a_refusal_leaves_it_running(
    commanded: tuple[Px4DroneConnection, list[CommandEvent]],
) -> None:
    module, events = commanded
    core = module._core
    assert core is not None
    core.state, core.takeoff = "HOVER", TakeoffPoint(n=0.0, e=0.0, d0=0.0, yaw=0.0)
    module._state.snapshot.return_value = _snapshot(d=-2.0, flying=True)

    assert module.go_to(north_m=-2.0, altitude_m=3.0)["accepted"]
    assert core.state == "GOTO" and core.goal == GotoGoal(n=-2.0, e=0.0, d=-3.0)
    assert module.go_to(north_m=100.0) == {
        "accepted": False,
        "rejection": "fence",
        "state": "GOTO",
    }
    assert core.goal == GotoGoal(n=-2.0, e=0.0, d=-3.0)
    assert [(e.command, e.argument, e.accepted, e.rejection) for e in events] == [
        ("go_to", "-2.00,0.00,3.00,nan,1", True, ""),
        ("go_to", "100.00,0.00,nan,nan,1", False, "fence"),
    ]


def _target(pitch_deg: float, yaw_deg: float) -> JointState:
    return JointState(
        name=["gimbal_pitch", "gimbal_yaw"],
        position=[math.radians(pitch_deg), math.radians(yaw_deg)],
    )


def test_gimbal_target_is_dropped_unless_commands_are_enabled() -> None:
    m = Px4DroneConnection(writer_lock_port=0)
    io = MagicMock()
    io.stats.return_value = {}
    m._io = io
    try:
        m._on_gimbal_target(_target(-10.0, 30.0))
        io.send_gimbal_pitchyaw.assert_not_called()
        assert m.sensor_stats()["gimbal_target"] == {"sent": 0, "dropped": 1}
    finally:
        m._io = None
        m.stop()


def test_gimbal_target_is_clamped_and_sent_when_enabled() -> None:
    m = Px4DroneConnection(writer_lock_port=0, gimbal_commands_enabled=True)
    io = MagicMock()
    m._io = io
    try:
        m._on_gimbal_target(_target(-120.0, 170.0))  # beyond the A8 limits
        io.send_gimbal_pitchyaw.assert_called_once()
        pitch, yaw, device = io.send_gimbal_pitchyaw.call_args.args
        assert (pitch, yaw, device) == (-90.0, 120.0, 154)
        # Rate limit: a second command inside the 10 Hz window is not sent.
        m._on_gimbal_target(_target(0.0, 0.0))
        assert io.send_gimbal_pitchyaw.call_count == 1
    finally:
        m._io = None
        m.stop()
