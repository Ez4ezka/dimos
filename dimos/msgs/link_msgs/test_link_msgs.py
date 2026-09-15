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

import math

import pytest

from dimos.msgs.link_msgs.LinkPolicy import LinkPolicy
from dimos.msgs.link_msgs.LinkStatus import LinkStatus


def test_link_status_roundtrip() -> None:
    st = LinkStatus(
        healthy=True,
        modem_present=True,
        rsrp_dbm=-83.0,
        sinr_db=14.0,
        rtt_ms=86.0,
        usable_uplink_bps=260_000.0,
        band="n41",
        cell_id="0x1A2B3C",
        interface="wwan5g",
        overlay_path="relay",
        ground_station="gigabyte",
        ts=1700000000.5,
    )
    back = LinkStatus.lcm_decode(st.lcm_encode())
    assert (back.healthy, back.modem_present) == (True, True)
    assert (back.rsrp_dbm, back.sinr_db, back.rtt_ms) == pytest.approx((-83.0, 14.0, 86.0))
    assert back.usable_uplink_bps == pytest.approx(260_000.0)
    assert math.isnan(back.rsrq_db) and math.isnan(back.throughput_up_bps)
    assert (back.band, back.cell_id, back.interface, back.overlay_path) == (
        "n41",
        "0x1A2B3C",
        "wwan5g",
        "relay",
    )
    assert back.ts == pytest.approx(1700000000.5, abs=1e-6)


def test_link_policy_roundtrip() -> None:
    pol = LinkPolicy(
        video_allowed=True,
        max_video_bitrate_bps=600_000.0,
        jpeg_hz=2.0,
        telemetry_profile="throttled",
        source="auto",
        reason="rtt 86 ms on relay",
        ts=1700000001.0,
    )
    back = LinkPolicy.lcm_decode(pol.lcm_encode())
    assert back.video_allowed is True
    assert back.max_video_bitrate_bps == pytest.approx(600_000.0)
    assert back.jpeg_hz == pytest.approx(2.0)
    assert (back.telemetry_profile, back.source, back.reason) == (
        "throttled",
        "auto",
        "rtt 86 ms on relay",
    )
