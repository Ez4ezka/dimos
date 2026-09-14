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

import pytest

from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.robot.px4.mavlink.frames import (
    body_flu_velocity_to_ned,
    flu_to_ned,
    frd_to_flu,
    ned_to_flu,
    quaternion_from_ned_euler,
)


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
