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
from unittest.mock import MagicMock

import pytest

from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.px4.connection import Px4DroneConnection

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
    "supervisor_status",
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
