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

"""SIYI A8 mini gimbal frame maths: limits, MAVLink flag bits, attitude normalisation.

Ported from drone-autonomy ``common/gimbal.py:17-86`` (verified in the flight mount
2026-09-04). The original picked the mount orientation per sample from the reported
roll; here the mount is a ``MountPreset`` chosen by config so a transient roll reading
can never flip the sign convention mid-flight. Pure functions, no I/O.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
import math

from dimos.utils.angles import wrap180

PITCH_MIN_DEG = -90.0
PITCH_MAX_DEG = 25.0
# A8 mechanical yaw limit is +/-135; keep a software margin.
YAW_MIN_DEG = -120.0
YAW_MAX_DEG = 120.0

# GIMBAL_DEVICE_FLAGS bits (MAVLink common.xml).
FLAG_RETRACT = 1
FLAG_NEUTRAL = 2
FLAG_ROLL_LOCK = 4
FLAG_PITCH_LOCK = 8
FLAG_YAW_LOCK = 16
FLAG_YAW_IN_VEHICLE_FRAME = 32
FLAG_YAW_IN_EARTH_FRAME = 64
FLAG_ACCEPTS_YAW_IN_EARTH_FRAME = 128

_FLAG_NAMES: tuple[tuple[int, str], ...] = (
    (FLAG_RETRACT, "RETRACT"),
    (FLAG_NEUTRAL, "NEUTRAL"),
    (FLAG_ROLL_LOCK, "ROLL_LOCK"),
    (FLAG_PITCH_LOCK, "PITCH_LOCK"),
    (FLAG_YAW_LOCK, "YAW_LOCK"),
    (FLAG_YAW_IN_VEHICLE_FRAME, "YAW_IN_VEHICLE_FRAME"),
    (FLAG_YAW_IN_EARTH_FRAME, "YAW_IN_EARTH_FRAME"),
    (FLAG_ACCEPTS_YAW_IN_EARTH_FRAME, "ACCEPTS_YAW_IN_EARTH_FRAME"),
)


@dataclass(frozen=True)
class MountPreset:
    """How the A8 is mounted, from ``config/gimbal_frame.json`` probe results.

    ``inverted`` selects the base-down bench orientation, where the A8 reports roll
    180 and yaw +180: pitch is negated and yaw shifted. Signs and offset apply after
    that normalisation.
    """

    name: str
    inverted: bool
    pitch_sign: float = 1.0
    yaw_sign: float = 1.0
    yaw_offset_deg: float = 0.0


# Flight mount: A8 hanging under the frame, roll reads ~0, raw angles used as-is.
# Verified 2026-09-04: yaw body-relative, pitch and roll earth-stabilised.
FLIGHT_MOUNT = MountPreset(name="flight", inverted=False)
# Bench: base down on the table (2026-09-02/04 calibration).
BENCH_MOUNT = MountPreset(name="bench", inverted=True)
MOUNT_PRESETS: dict[str, MountPreset] = {p.name: p for p in (FLIGHT_MOUNT, BENCH_MOUNT)}


def quat_to_euler_deg(q: Sequence[float]) -> tuple[float, float, float]:
    """MAVLink quaternion ``[w, x, y, z]`` -> ``(roll, pitch, yaw)`` degrees, ZYX."""
    w, x, y, z = (float(v) for v in q)
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sinp)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def normalize_attitude(q: Sequence[float], mount: MountPreset) -> tuple[float, float]:
    """``(pitch, yaw)`` degrees in the MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW convention."""
    _, raw_pitch, raw_yaw = quat_to_euler_deg(q)
    if mount.inverted:
        pitch, yaw = -raw_pitch, wrap180(raw_yaw + 180.0)
    else:
        pitch, yaw = raw_pitch, wrap180(raw_yaw)
    return mount.pitch_sign * pitch, wrap180(mount.yaw_sign * yaw + mount.yaw_offset_deg)


def decode_flags(flags: int) -> str:
    names = [name for bit, name in _FLAG_NAMES if flags & bit]
    return "|".join(names) if names else "none"
