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

from dataclasses import dataclass, field
from io import BytesIO
import math
import struct
import time
from typing import Any

from dimos.msgs.lcm_wire import (
    check_fingerprint,
    fingerprint,
    read_header,
    read_str,
    write_header,
    write_str,
)
from dimos.types.timestamped import Timestamped

_BASE_HASH = 0x6B0D4F2A8C13E795
_FIXED = ">IddddffffIfffffH"
_FIXED_SIZE = struct.calcsize(_FIXED)

# Bump when the response scoring changes so old events stay comparable.
SCORING_VERSION = 1


@dataclass
class TrackedCommand(Timestamped):
    """One operator command with its verdict and latency breakdown, as the tracker scored it.

    The three latency segments are measured separately and never summed: ``mux_ms`` is
    request to verdict, ``onboard_ms`` verdict to the first Offboard setpoint reflecting
    it, ``response_ms`` that setpoint to the observed odometry response. ``link_ms`` is
    the operator link on the hosted path, a different clock, NaN elsewhere. Unknown
    numbers are NaN.
    """

    msg_name = "px4_msgs.TrackedCommand"

    event_id: int = 0
    command: str = ""
    argument: str = ""
    source: str = ""
    verdict: str = ""
    rejection: str = ""
    request_ts: float = 0.0
    verdict_ts: float = 0.0
    commanded_ts: float = math.nan
    moved_ts: float = math.nan
    mux_ms: float = math.nan
    onboard_ms: float = math.nan
    response_ms: float = math.nan
    link_ms: float = math.nan
    setpoint_count: int = 0
    setpoint_gap_max_ms: float = math.nan
    commanded_peak: float = math.nan
    measured_peak: float = math.nan
    clamp_ratio: float = math.nan
    duration_s: float = math.nan
    scoring_version: int = SCORING_VERSION
    state_before: str = ""
    state_after: str = ""
    frame_id: str = ""
    ts: float = field(default_factory=time.time)

    def lcm_encode(self) -> bytes:
        buf = BytesIO()
        buf.write(fingerprint(_BASE_HASH, TrackedCommand))
        write_header(buf, self.ts, self.frame_id)
        buf.write(
            struct.pack(
                _FIXED,
                self.event_id,
                self.request_ts,
                self.verdict_ts,
                self.commanded_ts,
                self.moved_ts,
                self.mux_ms,
                self.onboard_ms,
                self.response_ms,
                self.link_ms,
                self.setpoint_count,
                self.setpoint_gap_max_ms,
                self.commanded_peak,
                self.measured_peak,
                self.clamp_ratio,
                self.duration_s,
                self.scoring_version,
            )
        )
        for s in (
            self.command,
            self.argument,
            self.source,
            self.verdict,
            self.rejection,
            self.state_before,
            self.state_after,
        ):
            write_str(buf, s)
        return buf.getvalue()

    @classmethod
    def lcm_decode(cls, data: bytes, **kwargs: Any) -> TrackedCommand:
        buf = BytesIO(data)
        check_fingerprint(buf, fingerprint(_BASE_HASH, TrackedCommand), "TrackedCommand")
        ts, frame_id = read_header(buf)
        v = struct.unpack(_FIXED, buf.read(_FIXED_SIZE))
        s = [read_str(buf) for _ in range(7)]
        return cls(
            event_id=v[0],
            request_ts=v[1],
            verdict_ts=v[2],
            commanded_ts=v[3],
            moved_ts=v[4],
            mux_ms=v[5],
            onboard_ms=v[6],
            response_ms=v[7],
            link_ms=v[8],
            setpoint_count=v[9],
            setpoint_gap_max_ms=v[10],
            commanded_peak=v[11],
            measured_peak=v[12],
            clamp_ratio=v[13],
            duration_s=v[14],
            scoring_version=v[15],
            command=s[0],
            argument=s[1],
            source=s[2],
            verdict=s[3],
            rejection=s[4],
            state_before=s[5],
            state_after=s[6],
            frame_id=frame_id,
            ts=ts,
        )

    def report_line(self) -> str:
        """One readable line, the shape ``command_report`` publishes."""

        def ms(v: float) -> str:
            return "-" if math.isnan(v) else f"{v:.0f}"

        verdict = self.verdict if not self.rejection else f"{self.verdict}/{self.rejection}"
        return (
            f"#{self.event_id} {self.command} {self.argument or ''} [{self.source}] {verdict} "
            f"mux={ms(self.mux_ms)}ms onboard={ms(self.onboard_ms)}ms "
            f"response={ms(self.response_ms)}ms setpoints={self.setpoint_count} "
            f"gap_max={ms(self.setpoint_gap_max_ms)}ms {self.state_before}->{self.state_after}"
        )

    def to_rerun(self) -> Any:
        import rerun as rr

        return rr.TextLog(self.report_line(), level="INFO" if self.verdict == "ok" else "WARN")
