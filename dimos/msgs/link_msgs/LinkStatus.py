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

_BASE_HASH = 0x2F9C4E11A7D3B860
_FIXED = ">??ffffffff"
_FIXED_SIZE = struct.calcsize(_FIXED)


@dataclass
class LinkStatus(Timestamped):
    """What the operator link can carry right now. Unknown numbers are NaN, unknown text empty.

    ``healthy`` is False when a source (modem, overlay network) could not be read; the
    fields it would have filled are then NaN or empty rather than stale.
    """

    msg_name = "link_msgs.LinkStatus"

    healthy: bool = False
    modem_present: bool = False
    rsrp_dbm: float = math.nan
    sinr_db: float = math.nan
    rsrq_db: float = math.nan
    rtt_ms: float = math.nan
    throughput_up_bps: float = math.nan
    throughput_down_bps: float = math.nan
    usable_uplink_bps: float = math.nan
    loss_pct: float = math.nan
    band: str = ""
    cell_id: str = ""
    interface: str = ""
    overlay_path: str = ""  # "direct", "relay", "none"
    ground_station: str = ""
    notes: str = ""
    frame_id: str = ""
    ts: float = field(default_factory=time.time)

    def lcm_encode(self) -> bytes:
        buf = BytesIO()
        buf.write(fingerprint(_BASE_HASH, LinkStatus))
        write_header(buf, self.ts, self.frame_id)
        buf.write(
            struct.pack(
                _FIXED,
                self.healthy,
                self.modem_present,
                self.rsrp_dbm,
                self.sinr_db,
                self.rsrq_db,
                self.rtt_ms,
                self.throughput_up_bps,
                self.throughput_down_bps,
                self.usable_uplink_bps,
                self.loss_pct,
            )
        )
        for s in (
            self.band,
            self.cell_id,
            self.interface,
            self.overlay_path,
            self.ground_station,
            self.notes,
        ):
            write_str(buf, s)
        return buf.getvalue()

    @classmethod
    def lcm_decode(cls, data: bytes, **kwargs: Any) -> LinkStatus:
        buf = BytesIO(data)
        check_fingerprint(buf, fingerprint(_BASE_HASH, LinkStatus), "LinkStatus")
        ts, frame_id = read_header(buf)
        v = struct.unpack(_FIXED, buf.read(_FIXED_SIZE))
        s = [read_str(buf) for _ in range(6)]
        return cls(
            healthy=v[0],
            modem_present=v[1],
            rsrp_dbm=v[2],
            sinr_db=v[3],
            rsrq_db=v[4],
            rtt_ms=v[5],
            throughput_up_bps=v[6],
            throughput_down_bps=v[7],
            usable_uplink_bps=v[8],
            loss_pct=v[9],
            band=s[0],
            cell_id=s[1],
            interface=s[2],
            overlay_path=s[3],
            ground_station=s[4],
            notes=s[5],
            frame_id=frame_id,
            ts=ts,
        )

    def to_rerun(self) -> Any:
        import rerun as rr

        return rr.TextLog(
            f"{self.interface or '-'} {self.overlay_path or '-'} rsrp={self.rsrp_dbm:.0f}dBm "
            f"sinr={self.sinr_db:.0f}dB rtt={self.rtt_ms:.0f}ms "
            f"uplink={self.usable_uplink_bps / 1e3:.0f}kbps",
            level="INFO" if self.healthy else "WARN",
        )
