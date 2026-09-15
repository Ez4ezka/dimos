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

"""Frame conversions between PX4 (NED world, FRD body) and dimOS (FLU / ENU-like).

dimOS follows ROS: x forward (north), y left (west), z up. MAVLink LOCAL_POSITION_NED
is x north, y east, z down, and body-frame IMU data is x forward, y right, z down.

The orientation conversion matches ``dimos/robot/drone/mavlink_connection.py:170``,
which is guarded by ``test_ned_to_ros_coordinate_conversion``: roll unchanged,
pitch and yaw negated. Pure functions, no I/O.
"""

from __future__ import annotations

import math

from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3


def ned_to_flu(n: float, e: float, d: float) -> tuple[float, float, float]:
    """NED world vector -> dimOS world vector (x north, y west, z up)."""
    return n, -e, -d


def flu_to_ned(x: float, y: float, z: float) -> tuple[float, float, float]:
    """dimOS world vector -> NED world vector."""
    return x, -y, -z


def frd_to_flu(x: float, y: float, z: float) -> tuple[float, float, float]:
    """Body FRD (forward, right, down) -> body FLU (forward, left, up)."""
    return x, -y, -z


def ned_yaw_to_flu_yaw(yaw_rad: float) -> float:
    """NED heading (clockwise positive from north) -> FLU yaw (counter-clockwise)."""
    return -yaw_rad


def quaternion_from_ned_euler(roll: float, pitch: float, yaw: float) -> Quaternion:
    """MAVLink ATTITUDE euler (rad, NED/FRD) -> dimOS orientation quaternion."""
    return Quaternion.from_euler(Vector3(roll, -pitch, -yaw))


def body_flu_velocity_to_ned(
    forward: float, left: float, up: float, yaw_ned_rad: float
) -> tuple[float, float, float]:
    """Body-frame FLU velocity command -> NED world velocity using the vehicle heading.

    Used to turn a teleop ``Twist`` (body frame) into the LOCAL_NED velocity setpoint
    PX4 expects. ``yaw_ned_rad`` is the ATTITUDE yaw (clockwise from north).
    """
    right = -left
    c, s = math.cos(yaw_ned_rad), math.sin(yaw_ned_rad)
    vn = forward * c - right * s
    ve = forward * s + right * c
    return vn, ve, -up
