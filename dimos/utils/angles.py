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

"""Degree-domain angle helpers shared by the PX4 guidance and SIYI gimbal code.

Ported from drone-autonomy ``common/gimbal.py:36-41`` (flown 2026-09-09).
"""


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def wrap180(angle_deg: float) -> float:
    """Wrap an angle in degrees into [-180, 180)."""
    return (angle_deg + 180.0) % 360.0 - 180.0
