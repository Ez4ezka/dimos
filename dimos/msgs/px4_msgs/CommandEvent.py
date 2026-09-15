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

_BASE_HASH = 0x3C71E9A5D06B2F44
_FIXED = ">Id?"
_FIXED_SIZE = struct.calcsize(_FIXED)


@dataclass
class CommandEvent(Timestamped):
    """One operator command as the connection judged it.

    ``ts`` is the request time; ``verdict_ts`` when the verdict was known. ``rejection`` is
    the SupervisorCore rejection value, empty when accepted. ``source`` says which path the
    request took (``rpc``, ``estop_in``, ``cmd_vel``).
    """

    msg_name = "px4_msgs.CommandEvent"

    request_id: int = 0
    command: str = ""
    argument: str = ""
    source: str = "rpc"
    verdict_ts: float = 0.0
    accepted: bool = False
    rejection: str = ""
    state_before: str = ""
    state_after: str = ""
    frame_id: str = ""
    ts: float = field(default_factory=time.time)

    def lcm_encode(self) -> bytes:
        buf = BytesIO()
        buf.write(fingerprint(_BASE_HASH, CommandEvent))
        write_header(buf, self.ts, self.frame_id)
        buf.write(struct.pack(_FIXED, self.request_id, self.verdict_ts, self.accepted))
        for s in (
            self.command,
            self.argument,
            self.source,
            self.rejection,
            self.state_before,
            self.state_after,
        ):
            write_str(buf, s)
        return buf.getvalue()

    @classmethod
    def lcm_decode(cls, data: bytes, **kwargs: Any) -> CommandEvent:
        buf = BytesIO(data)
        check_fingerprint(buf, fingerprint(_BASE_HASH, CommandEvent), "CommandEvent")
        ts, frame_id = read_header(buf)
        request_id, verdict_ts, accepted = struct.unpack(_FIXED, buf.read(_FIXED_SIZE))
        strings = [read_str(buf) for _ in range(6)]
        return cls(
            request_id=request_id,
            command=strings[0],
            argument=strings[1],
            source=strings[2],
            verdict_ts=verdict_ts,
            accepted=accepted,
            rejection=strings[3],
            state_before=strings[4],
            state_after=strings[5],
            frame_id=frame_id,
            ts=ts,
        )

    def to_rerun(self) -> Any:
        import rerun as rr

        verdict = "accepted" if self.accepted else f"rejected/{self.rejection}"
        return rr.TextLog(
            f"#{self.request_id} {self.command} {self.argument} via {self.source}: {verdict} "
            f"({self.state_before} -> {self.state_after})",
            level="INFO" if self.accepted else "WARN",
        )
