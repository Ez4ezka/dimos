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

"""Guidance laws. Ported from drone-autonomy ``tests/test_guidance.py:25-54``."""

from __future__ import annotations

import math

from dimos.robot.px4.config import FollowConfig, YawTrackConfig
from dimos.robot.px4.guidance import (
    TargetEstimate,
    follow_velocity,
    rate_limit_yaw,
    yaw_track_rate,
)

YT = YawTrackConfig()
FO = FollowConfig()


def close(a: float, b: float, tol: float = 0.05) -> None:
    assert abs(a - b) <= tol, (a, b)


def test_yaw_track_deadband_and_limit() -> None:
    assert yaw_track_rate(10.0, True, YT) == 0.0
    assert yaw_track_rate(40.0, False, YT) == 0.0
    assert yaw_track_rate(None, True, YT) == 0.0
    r = yaw_track_rate(40.0, True, YT)
    close(r, YT.k_yaw * 25.0)
    assert yaw_track_rate(-89.0, True, YT) == -YT.max_yaw_rate_dps


def test_follow_geometry() -> None:
    tgt = TargetEstimate(valid=True, n=30.0, e=0.0, vn=0.0, ve=0.0)
    c = follow_velocity(0.0, 0.0, -10.0, 0.0, tgt, FO)
    close(c.range_m, 30.0)
    close(c.bearing_deg, 0.0)
    assert c.vn > 0 and abs(c.ve) < 1e-9  # closes toward target
    assert math.hypot(c.vn, c.ve) <= FO.v_max_mps + 1e-9
    close(c.vd, 0.0)  # already at 10 m
    tgt = TargetEstimate(valid=True, n=12.5, e=0.0, vn=0.0, ve=0.0)
    c = follow_velocity(0.0, 0.0, -5.0, 0.0, tgt, FO)
    close(c.vn, 0.0)  # inside deadband
    assert c.vd < 0  # climb (negative D velocity) to 10 m
    tgt = TargetEstimate(valid=True, n=5.0, e=0.0, vn=0.0, ve=0.0)
    assert follow_velocity(0, 0, -10, 0, tgt, FO).vn < 0  # too close: back away
    tgt = TargetEstimate(valid=True, n=12.0, e=0.0, vn=0.0, ve=1.0)
    close(follow_velocity(0, 0, -10, 0, tgt, FO).ve, FO.ff_gain * 1.0)  # feed-forward


def test_rate_limit_yaw_wraps() -> None:
    close(rate_limit_yaw(170.0, -170.0, 30.0, 0.5), -175.0)
    close(rate_limit_yaw(0.0, 90.0, 30.0, 1.0), 30.0)
