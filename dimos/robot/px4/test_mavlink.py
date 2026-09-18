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

"""The MAVLink layer without a socket: frames, timebase, vehicle state and timed buffers."""

from __future__ import annotations

import math
from typing import Any

import pytest

from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.robot.px4.mavlink import (
    ESTIMATOR_POS_HORIZ_ABS,
    ESTIMATOR_POS_VERT_ABS,
    MAIN_OFFBOARD,
    Px4Timebase,
    TimedBuffer,
    VehicleState,
    body_flu_velocity_to_ned,
    flu_to_ned,
    frd_to_flu,
    ned_to_flu,
    quaternion_from_ned_euler,
)

_UNIX = 1_800_000_000.0


def test_ned_to_flu_matches_upstream_signs() -> None:
    # Same convention as dimos/robot/drone/test_drone.py::test_ned_to_ros_coordinate_conversion:
    # north -> +x, east -> -y, down -> -z.
    assert ned_to_flu(3.0, 4.0, -1.0) == (3.0, -4.0, 1.0)
    assert flu_to_ned(*ned_to_flu(3.0, 4.0, -1.0)) == (3.0, 4.0, -1.0)
    assert frd_to_flu(1.0, 2.0, 9.8) == (1.0, -2.0, -9.8)


def test_quaternion_matches_mavlink_connection_conversion() -> None:
    # mavlink_connection.py:170: Quaternion.from_euler(Vector3(roll, -pitch, -yaw))
    q = quaternion_from_ned_euler(0.1, 0.2, 0.3)
    ref = Quaternion.from_euler(Vector3(0.1, -0.2, -0.3))
    assert (q.x, q.y, q.z, q.w) == pytest.approx((ref.x, ref.y, ref.z, ref.w))


def test_body_velocity_rotates_with_heading() -> None:
    # Heading north: forward is north, left is west (negative east).
    assert body_flu_velocity_to_ned(1.0, 0.5, 0.2, 0.0) == pytest.approx((1.0, -0.5, -0.2))
    # Heading east (yaw +90 deg clockwise): forward is east, left is north.
    vn, ve, vd = body_flu_velocity_to_ned(1.0, 0.5, 0.0, math.radians(90))
    assert (vn, ve, vd) == pytest.approx((0.5, 1.0, 0.0), abs=1e-12)


def test_median_offset_and_jump_guard() -> None:
    tb = Px4Timebase(min_samples=3, jump_guard_s=0.5)
    assert tb.quality == "none"
    with pytest.raises(RuntimeError):
        tb.to_utc(0.0)
    for boot in (10.0, 11.0, 12.0):
        tb.add_system_time(_UNIX + boot, boot, receive_wall_s=_UNIX + boot + 0.02)
    assert tb.quality == "system_time"
    assert tb.offset_s == _UNIX
    # A wild sample is rejected, not averaged in.
    tb.add_system_time(_UNIX + 13.0 + 5.0, 13.0, receive_wall_s=_UNIX + 13.02)
    assert tb.rejected == 1
    assert tb.offset_s == _UNIX
    assert tb.to_utc(20.0) == _UNIX + 20.0


def test_partial_quality_below_min_samples() -> None:
    tb = Px4Timebase(min_samples=30)
    tb.add_system_time(_UNIX + 1.0, 1.0, receive_wall_s=_UNIX + 1.01)
    assert tb.quality == "system_time_partial"
    assert tb.offset_s == _UNIX


def test_receive_time_fallback_is_min_filtered() -> None:
    tb = Px4Timebase(min_samples=3)
    # No GPS: PX4 reports unix time 0. Latency varies 10..50 ms; min wins.
    for boot, latency in ((1.0, 0.05), (2.0, 0.01), (3.0, 0.03)):
        tb.add_system_time(0.0, boot, receive_wall_s=_UNIX + boot + latency)
    assert tb.quality == "receive_time"
    assert tb.offset_s == pytest.approx(_UNIX + 0.01)


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


def test_snapshot_carries_estimator_validity_and_the_latest_warning() -> None:
    st = VehicleState()
    st.handle(Msg("ESTIMATOR_STATUS", flags=ESTIMATOR_POS_HORIZ_ABS), now=100.0)
    st.handle(Msg("STATUSTEXT", severity=6, text="Ready"), now=100.1)
    snap = st.snapshot(now=101.0)
    assert snap.estimator is not None and snap.estimator.position_valid is False
    assert snap.statustext is None  # INFO is not worth an operator's attention
    st.handle(
        Msg("ESTIMATOR_STATUS", flags=ESTIMATOR_POS_HORIZ_ABS | ESTIMATOR_POS_VERT_ABS),
        now=102.0,
    )
    st.handle(Msg("STATUSTEXT", severity=2, text=b"Arming denied: not landed\x00"), now=102.5)
    snap = st.snapshot(now=103.0)
    assert snap.estimator is not None and snap.estimator.position_valid
    assert snap.statustext is not None and snap.statustext.text == "Arming denied: not landed"
    assert snap.statustext.t == 102.5


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
