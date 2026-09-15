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

"""Link gate: every recorded scenario through the LinkMonitor, no modem, no network.

For each scenario measured on 2026-09-08/09 (home Wi-Fi, home 5G, downtown idle,
downtown with video) plus a laptop with no hardware, prints the derived usable uplink and
the resulting policy against the expected bands. PASS or FAIL::

    python dimos/robot/px4/tool_link_gate.py
"""

from __future__ import annotations

import math
import time

from dimos.robot.px4.link_monitor import LinkMonitor
from dimos.robot.px4.link_sources import SCENARIOS

# Expected usable uplink band (bps) per scenario, from the measurements.
_UPLINK_BANDS = {
    "home_wifi": (1_000_000.0, math.inf),
    "home_5g": (1_000_000.0, math.inf),
    "downtown_idle": (100_000.0, 600_000.0),
    "downtown_video": (0.0, 200_000.0),
    "no_hardware": (math.nan, math.nan),
}


def main() -> int:
    ok = True
    for name, scenario in SCENARIOS.items():
        m = LinkMonitor(source="replay", replay_scenario=name)
        for port in ("link_status", "link_policy"):
            getattr(m, port).publish = lambda _msg: None
        m.setup_sources()
        try:
            m.poll()
            time.sleep(0.5)  # the second round measures the throughput over a real interval
            status, policy = m.poll()
        finally:
            m.stop()
        lo, hi = _UPLINK_BANDS[name]
        up = status.usable_uplink_bps
        in_band = math.isnan(up) if math.isnan(lo) else lo <= up <= hi
        good = (
            in_band
            and policy.video_allowed == scenario.expect_video
            and policy.telemetry_profile == scenario.expect_telemetry
        )
        ok &= good
        print(
            f"{name:15s} healthy={status.healthy!s:5} path={status.overlay_path or '-':6} "
            f"rsrp={status.rsrp_dbm:6.0f} sinr={status.sinr_db:4.0f} rtt={status.rtt_ms:5.0f}ms "
            f"loss={status.loss_pct:3.0f}% tx={status.throughput_up_bps / 1e3:7.0f}kbps "
            f"usable={up / 1e3:7.0f}kbps -> video={'on ' if policy.video_allowed else 'off'} "
            f"max={policy.max_video_bitrate_bps / 1e3:5.0f}kbps jpeg={policy.jpeg_hz:.1f}Hz "
            f"telemetry={policy.telemetry_profile:9s} {'OK' if good else 'FAIL'}  ({scenario.note})"
        )
    print("GATE", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
