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
import struct
import time
from typing import Any

from dimos.msgs.px4_msgs.lcm_codec import (
    check_fingerprint,
    fingerprint,
    read_header,
    read_str,
    write_header,
    write_str,
)
from dimos.types.timestamped import Timestamped

_BASE_HASH = 0x7D42A6C3F09E1B58
_FIXED = ">?ff"
_FIXED_SIZE = struct.calcsize(_FIXED)

TELEMETRY_FULL = "full"
TELEMETRY_THROTTLED = "throttled"


@dataclass
class LinkPolicy(Timestamped):
    """What the aircraft may put on the operator link, derived from LinkStatus.

    RtspCamera and the relay obey it. ``source`` is ``auto`` when derived, ``override``
    when an operator forced it by RPC; ``reason`` says which measurement decided.
    """

    msg_name = "link_msgs.LinkPolicy"

    video_allowed: bool = False
    max_video_bitrate_bps: float = 0.0
    jpeg_hz: float = 0.0
    telemetry_profile: str = TELEMETRY_THROTTLED
    source: str = "auto"
    reason: str = ""
    frame_id: str = ""
    ts: float = field(default_factory=time.time)

    def lcm_encode(self) -> bytes:
        buf = BytesIO()
        buf.write(fingerprint(_BASE_HASH, LinkPolicy))
        write_header(buf, self.ts, self.frame_id)
        buf.write(struct.pack(_FIXED, self.video_allowed, self.max_video_bitrate_bps, self.jpeg_hz))
        for s in (self.telemetry_profile, self.source, self.reason):
            write_str(buf, s)
        return buf.getvalue()

    @classmethod
    def lcm_decode(cls, data: bytes, **kwargs: Any) -> LinkPolicy:
        buf = BytesIO(data)
        check_fingerprint(buf, fingerprint(_BASE_HASH, LinkPolicy), "LinkPolicy")
        ts, frame_id = read_header(buf)
        video_allowed, bitrate, jpeg_hz = struct.unpack(_FIXED, buf.read(_FIXED_SIZE))
        profile, source, reason = (read_str(buf) for _ in range(3))
        return cls(
            video_allowed=video_allowed,
            max_video_bitrate_bps=bitrate,
            jpeg_hz=jpeg_hz,
            telemetry_profile=profile,
            source=source,
            reason=reason,
            frame_id=frame_id,
            ts=ts,
        )

    def to_rerun(self) -> Any:
        import rerun as rr

        return rr.TextLog(
            f"video={'on' if self.video_allowed else 'off'} "
            f"max={self.max_video_bitrate_bps / 1e3:.0f}kbps jpeg={self.jpeg_hz:.1f}Hz "
            f"telemetry={self.telemetry_profile} [{self.source}] {self.reason}",
            level="INFO" if self.video_allowed else "WARN",
        )
