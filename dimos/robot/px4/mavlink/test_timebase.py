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

from dimos.robot.px4.mavlink.timebase import Px4Timebase

_UNIX = 1_800_000_000.0


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
