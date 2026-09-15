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

from __future__ import annotations

import math
from typing import Any

from dimos.robot.px4.mavlink.vehicle_state import TimedBuffer, VehicleState
from dimos.robot.px4.px4_modes import MAIN_OFFBOARD


class Msg:
    def __init__(self, typ: str, src: tuple[int, int] = (1, 1), **kw: Any) -> None:
        self._t = typ
        self._src = src
        self.__dict__.update(kw)

    def get_type(self) -> str:
        return self._t

    def get_srcSystem(self) -> int:
        return self._src[0]

    def get_srcComponent(self) -> int:
        return self._src[1]


def test_timed_buffer_interpolates_and_wraps_angles() -> None:
    buf = TimedBuffer(2.0, angular=("yaw",))
    buf.push(10.0, {"yaw": 170.0, "x": 0.0}, boot=100.0)
    buf.push(11.0, {"yaw": -170.0, "x": 2.0}, boot=101.0)
    mid = buf.at(10.5)
    assert mid is not None
    assert mid["x"] == 1.0
    assert mid["yaw"] == 180.0  # shortest way round, not the -0 average
    by_boot = buf.at_boot(100.25)
    assert by_boot is not None
    assert by_boot["x"] == 0.5
    assert buf.at(5.0) == {"yaw": 170.0, "x": 0.0}  # clamped, never extrapolated
    assert buf.at(20.0) == {"yaw": -170.0, "x": 2.0}


def test_timed_buffer_evicts_old_samples() -> None:
    buf = TimedBuffer(1.0)
    buf.push(0.0, {"v": 0.0})
    buf.push(0.5, {"v": 1.0})
    buf.push(1.6, {"v": 2.0})
    assert list(buf.t) == [1.6]


def test_messages_from_other_components_are_ignored() -> None:
    st = VehicleState()
    st.handle(
        Msg("HEARTBEAT", src=(1, 191), base_mode=128, custom_mode=MAIN_OFFBOARD << 16), now=1.0
    )
    assert st.heartbeat is None
    st.handle(Msg("HEARTBEAT", src=(1, 1), base_mode=128, custom_mode=MAIN_OFFBOARD << 16), now=2.0)
    assert st.heartbeat is not None and st.heartbeat.armed and st.heartbeat.main == MAIN_OFFBOARD


def test_snapshot_ages_use_snapshot_time() -> None:
    st = VehicleState()
    st.handle(Msg("HEARTBEAT", base_mode=0, custom_mode=0), now=100.0)
    st.handle(
        Msg("LOCAL_POSITION_NED", time_boot_ms=5000, x=1, y=2, z=-3, vx=0, vy=0, vz=0), now=100.2
    )
    st.handle(
        Msg("ATTITUDE", time_boot_ms=5000, roll=0.0, pitch=0.0, yaw=math.radians(90)), now=100.2
    )
    snap = st.snapshot(now=101.0)
    assert snap.heartbeat_age == 1.0
    assert snap.local is not None and snap.local.boot_s == 5.0
    assert math.isclose(snap.local_age, 0.8)
    assert snap.yaw_deg == 90.0
    assert snap.rc is None and snap.rc_age == math.inf


def test_gimbal_attitude_only_from_component_154() -> None:
    st = VehicleState()
    ident = [1.0, 0.0, 0.0, 0.0]
    st.handle(
        Msg("GIMBAL_DEVICE_ATTITUDE_STATUS", src=(1, 1), q=ident, flags=16, failure_flags=0),
        now=1.0,
    )
    assert st.gimbal.latest() == (None, None)
    st.handle(
        Msg("GIMBAL_DEVICE_ATTITUDE_STATUS", src=(1, 154), q=ident, flags=16, failure_flags=0),
        now=1.0,
    )
    t, g = st.gimbal.latest()
    assert t == 1.0 and g is not None and g["pitch"] == 0.0 and g["yaw"] == 0.0
    assert st.gimbal_flags == 16
