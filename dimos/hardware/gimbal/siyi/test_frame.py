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

from dimos.hardware.gimbal.siyi.frame import (
    BENCH_MOUNT,
    FLAG_YAW_IN_VEHICLE_FRAME,
    FLAG_YAW_LOCK,
    FLIGHT_MOUNT,
    decode_flags,
    normalize_attitude,
    quat_to_euler_deg,
)


def _quat_wxyz(roll: float, pitch: float, yaw: float) -> list[float]:
    """ZYX euler (degrees) -> MAVLink [w, x, y, z]."""
    r, p, y = (math.radians(a) / 2.0 for a in (roll, pitch, yaw))
    cr, sr, cp, sp, cy, sy = (
        math.cos(r),
        math.sin(r),
        math.cos(p),
        math.sin(p),
        math.cos(y),
        math.sin(y),
    )
    return [
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ]


def test_quat_to_euler_roundtrip() -> None:
    roll, pitch, yaw = quat_to_euler_deg(_quat_wxyz(10.0, -30.0, 45.0))
    assert (roll, pitch, yaw) == pytest.approx((10.0, -30.0, 45.0))


def test_flight_mount_uses_raw_angles() -> None:
    # Verified 2026-09-04 in the flight mount: --set 0 45 -> yaw +43.9, --set 20 0 -> pitch +20.
    pitch, yaw = normalize_attitude(_quat_wxyz(0.0, 20.0, 45.0), FLIGHT_MOUNT)
    assert (pitch, yaw) == pytest.approx((20.0, 45.0))


def test_bench_mount_negates_pitch_and_shifts_yaw() -> None:
    # Base-down on the bench the A8 reports roll 180 and yaw +180.
    pitch, yaw = normalize_attitude(_quat_wxyz(180.0, -20.0, -135.0), BENCH_MOUNT)
    assert (pitch, yaw) == pytest.approx((20.0, 45.0))


def test_decode_flags() -> None:
    assert decode_flags(0) == "none"
    assert (
        decode_flags(FLAG_YAW_LOCK | FLAG_YAW_IN_VEHICLE_FRAME) == "YAW_LOCK|YAW_IN_VEHICLE_FRAME"
    )
