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

"""Px4Drone module shell: the RPC surface, start refusal beside another writer, stop order."""

from __future__ import annotations

from collections.abc import Iterator
import socket
import threading
from unittest.mock import MagicMock

import pytest

from dimos.robot.px4.px4_drone import Px4Drone

# Anything that could move the aircraft must not be reachable over RPC.
_FORBIDDEN_RPCS = {
    "arm",
    "disarm",
    "set_mode",
    "set_px4_mode",
    "send_position_setpoint",
    "send_velocity_setpoint",
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
def module() -> Iterator[Px4Drone]:
    m = Px4Drone(writer_lock_port=0)
    yield m
    m.stop()


def test_rpc_surface_has_no_actuation(module: Px4Drone) -> None:
    names = set(module.rpcs)
    assert not (names & _FORBIDDEN_RPCS), names & _FORBIDDEN_RPCS
    assert _REQUIRED_RPCS <= names, _REQUIRED_RPCS - names


def test_start_refuses_when_another_writer_holds_the_lock_port() -> None:
    holder = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    holder.bind(("127.0.0.1", 0))
    port = holder.getsockname()[1]
    m = Px4Drone(writer_lock_port=port)
    try:
        with pytest.raises(RuntimeError, match="another Offboard writer"):
            m.start()
    finally:
        holder.close()
        m.stop()
    assert m._io is None  # the MAVLink socket was never opened


def test_stop_joins_tick_before_heartbeat_before_publish_then_closes_io(
    module: Px4Drone,
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
