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

"""PX4 boot-time to UTC conversion. Pure, no I/O.

PX4 stamps LOCAL_POSITION_NED, ATTITUDE and friends with ``time_boot_ms``. SYSTEM_TIME
carries both ``time_boot_ms`` and ``time_unix_usec`` (GPS-disciplined once there is a
fix), so the offset ``unix - boot`` is the vehicle's own clock. When PX4 has no valid
unix time (no GPS yet, SITL without a fix) the fallback is the min-filtered
``receive_wall - boot``: link latency only ever makes that larger, so the minimum
over many samples is the least-biased estimate.
"""

from __future__ import annotations

from collections import deque
import statistics
from typing import Literal

TimebaseQuality = Literal["none", "receive_time", "system_time_partial", "system_time"]

# PX4 reports unix time as 0 / near-epoch until it has a source; anything before this
# (2020-01-01) is not a real wall clock.
_MIN_VALID_UNIX_S = 1577836800.0


class Px4Timebase:
    def __init__(
        self,
        *,
        min_samples: int = 30,
        jump_guard_s: float = 0.5,
        window: int = 300,
    ) -> None:
        self._min_samples = min_samples
        self._jump_guard_s = jump_guard_s
        self._system: deque[float] = deque(maxlen=window)
        self._receive_min: float | None = None
        self.rejected = 0
        self.samples = 0

    def add_system_time(self, unix_s: float, boot_s: float, receive_wall_s: float) -> None:
        """One SYSTEM_TIME message: vehicle unix time, vehicle boot time, our receive time."""
        self.samples += 1
        fallback = receive_wall_s - boot_s
        self._receive_min = (
            fallback if self._receive_min is None else min(self._receive_min, fallback)
        )
        if unix_s < _MIN_VALID_UNIX_S:
            return
        offset = unix_s - boot_s
        if len(self._system) >= self._min_samples:
            # Jump guard: a single wild sample (clock step on the vehicle, corrupted
            # packet) must not drag the median; count it and drop it.
            if abs(offset - statistics.median(self._system)) > self._jump_guard_s:
                self.rejected += 1
                return
        self._system.append(offset)

    @property
    def quality(self) -> TimebaseQuality:
        if len(self._system) >= self._min_samples:
            return "system_time"
        if self._system:
            return "system_time_partial"
        if self._receive_min is not None:
            return "receive_time"
        return "none"

    @property
    def offset_s(self) -> float:
        """Seconds to add to a vehicle boot time to get UTC."""
        if self._system:
            return statistics.median(self._system)
        if self._receive_min is not None:
            return self._receive_min
        raise RuntimeError("Px4Timebase has no samples")

    def to_utc(self, boot_s: float) -> float:
        return boot_s + self.offset_s
