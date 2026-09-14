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

import pytest

from dimos.msgs.sensor_msgs.NavSatFix import (
    COVARIANCE_TYPE_DIAGONAL_KNOWN,
    STATUS_GBAS_FIX,
    NavSatFix,
)


def test_lcm_roundtrip() -> None:
    fix = NavSatFix(
        latitude=37.7749,
        longitude=-122.4194,
        altitude=12.5,
        status=STATUS_GBAS_FIX,
        position_covariance=[0.36, 0, 0, 0, 0.36, 0, 0, 0, 1.0],
        position_covariance_type=COVARIANCE_TYPE_DIAGONAL_KNOWN,
        frame_id="gps",
        ts=1700000000.25,
    )
    back = NavSatFix.lcm_decode(fix.lcm_encode())
    assert (back.latitude, back.longitude, back.altitude) == (37.7749, -122.4194, 12.5)
    assert back.status == STATUS_GBAS_FIX
    assert back.position_covariance == fix.position_covariance
    assert back.position_covariance_type == COVARIANCE_TYPE_DIAGONAL_KNOWN
    assert back.frame_id == "gps"
    assert back.ts == pytest.approx(1700000000.25, abs=1e-6)
