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

"""PX4 mode numbers, landed states and Offboard setpoint type masks. Pure data.

Ported from drone-autonomy ``common/px4_offboard.py:11-32`` (flown 2026-09-09).
PX4 packs its flight mode into ``HEARTBEAT.custom_mode`` as
``main = (custom_mode >> 16) & 0xFF`` and ``sub = (custom_mode >> 24) & 0xFF``.
"""

from __future__ import annotations

MAIN_MANUAL = 1
MAIN_ALTCTL = 2
MAIN_POSCTL = 3
MAIN_AUTO = 4
MAIN_ACRO = 5
MAIN_OFFBOARD = 6
MAIN_STAB = 7

SUB_AUTO_READY = 1
SUB_AUTO_TAKEOFF = 2
SUB_AUTO_LOITER = 3
SUB_AUTO_MISSION = 4
SUB_AUTO_RTL = 5
SUB_AUTO_LAND = 6

MAIN_NAMES: dict[int, str] = {
    MAIN_MANUAL: "MANUAL",
    MAIN_ALTCTL: "ALTCTL",
    MAIN_POSCTL: "POSCTL",
    MAIN_AUTO: "AUTO",
    MAIN_ACRO: "ACRO",
    MAIN_OFFBOARD: "OFFBOARD",
    MAIN_STAB: "STABILIZED",
}
SUB_NAMES: dict[int, str] = {
    SUB_AUTO_READY: "READY",
    SUB_AUTO_TAKEOFF: "TAKEOFF",
    SUB_AUTO_LOITER: "LOITER/HOLD",
    SUB_AUTO_MISSION: "MISSION",
    SUB_AUTO_RTL: "RTL",
    SUB_AUTO_LAND: "LAND",
    7: "FOLLOW",
    8: "PRECLAND",
}

# MAV_LANDED_STATE
LANDED_UNDEFINED = 0
LANDED_ON_GROUND = 1
LANDED_IN_AIR = 2
LANDED_TAKEOFF = 3
LANDED_LANDING = 4

# HEARTBEAT.base_mode bit
MAV_MODE_FLAG_SAFETY_ARMED = 128

# SET_POSITION_TARGET_LOCAL_NED type_mask bits (1 = ignore that field).
_IGN_POS = 0x7
_IGN_VEL = 0x38
_IGN_ACC = 0x1C0
_FORCE = 0x200
_IGN_YAW = 0x400
_IGN_YAWRATE = 0x800
MASK_POS_YAW = _IGN_VEL | _IGN_ACC | _FORCE | _IGN_YAWRATE  # 0xBF8
MASK_VEL_YAW = _IGN_POS | _IGN_ACC | _FORCE | _IGN_YAWRATE  # 0xBC7
MASK_VEL_YAWRATE = _IGN_POS | _IGN_ACC | _FORCE | _IGN_YAW  # 0x7C7


def decode_custom_mode(custom_mode: int) -> tuple[int, int]:
    """``HEARTBEAT.custom_mode`` -> ``(main, sub)``."""
    return (custom_mode >> 16) & 0xFF, (custom_mode >> 24) & 0xFF


def mode_name(main: int | None, sub: int = 0) -> str:
    """Human name such as ``OFFBOARD`` or ``AUTO:LOITER/HOLD``; ``?`` when unknown."""
    if main is None:
        return "?"
    s = MAIN_NAMES.get(main, str(main))
    if main == MAIN_AUTO:
        s += ":" + SUB_NAMES.get(sub, str(sub))
    return s
