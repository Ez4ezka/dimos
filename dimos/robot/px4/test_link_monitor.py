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

"""LinkMonitor on replayed modem and overlay readings. No modem, no tailscale, no network."""

from __future__ import annotations

from collections.abc import Iterator
import math
from typing import Any

import pytest

from dimos.robot.px4.link_monitor import LinkMonitor, estimate_uplink_bps
from dimos.robot.px4.link_sources import (
    SCENARIOS,
    ModemReading,
    PathReading,
    PingReading,
    parse_ping,
    parse_qcsq,
    parse_qeng_servingcell,
    parse_tailscale_status,
)

_FLIGHT_FACING = {"cmd_vel", "gimbal_target", "estop_in", "target_state", "offboard_setpoint"}


def _monitor(scenario: str) -> tuple[LinkMonitor, dict[str, list[Any]]]:
    m = LinkMonitor(source="replay", replay_scenario=scenario)
    published: dict[str, list[Any]] = {"link_status": [], "link_policy": []}
    for name, sink in published.items():
        getattr(m, name).publish = sink.append
    m.setup_sources()  # no poll thread: the tests drive every poll themselves
    return m, published


@pytest.fixture
def downtown() -> Iterator[tuple[LinkMonitor, dict[str, list[Any]]]]:
    m, published = _monitor("downtown_idle")
    yield m, published
    m.stop()


def test_parsers_read_the_quectel_and_tailscale_formats() -> None:
    r = parse_qcsq('+QCSQ: "NR5G-SA",-83,18,-11\r\n\r\nOK\r\n')
    assert (r.present, r.sysmode, r.rsrp_dbm, r.sinr_db, r.rsrq_db) == (
        True,
        "NR5G-SA",
        -83.0,
        18.0,
        -11.0,
    )
    lte = parse_qcsq('+QCSQ: "LTE",-51,-79,20,-8')
    assert (lte.rsrp_dbm, lte.sinr_db) == (-79.0, 20.0)
    assert parse_qcsq("ERROR").present is False
    assert parse_qeng_servingcell(SCENARIOS["downtown_idle"].qeng) == ("n41", "2B3C4D")
    path = parse_tailscale_status(SCENARIOS["downtown_idle"].tailscale, "gigabyte")
    assert (path.available, path.online, path.direct, path.relay) == (True, True, False, "sfo")
    assert parse_tailscale_status(SCENARIOS["home_5g"].tailscale, "100.100.114.31").direct
    ping = parse_ping(SCENARIOS["downtown_video"].ping)
    assert (ping.reachable, ping.rtt_ms, ping.loss_pct) == (True, 413.0, 5.0)
    assert parse_ping("").reachable is False


def test_uplink_estimate_orders_the_measured_scenarios() -> None:
    def est(name: str) -> float:
        s = SCENARIOS[name]
        return estimate_uplink_bps(
            parse_qcsq(s.qcsq) if s.qcsq else ModemReading(present=False),
            parse_tailscale_status(s.tailscale, "gigabyte") if s.tailscale else PathReading(False),
            parse_ping(s.ping) if s.ping else PingReading(False),
        )

    assert est("home_5g") > est("downtown_idle") > est("downtown_video")
    assert est("downtown_idle") < 600_000.0  # the relay cell had 0.26 Mbit/s spare
    assert math.isnan(est("no_hardware"))


def test_advisory_only_surface(downtown: tuple[LinkMonitor, dict[str, list[Any]]]) -> None:
    m, _ = downtown
    assert set(m.inputs) == set()
    assert set(m.outputs) == {"link_status", "link_policy"}
    assert not (set(m.outputs) & _FLIGHT_FACING)
    assert {"status", "path", "set_policy", "clear_policy"} <= set(m.rpcs)


def test_downtown_relay_keeps_video_off_and_telemetry_throttled(
    downtown: tuple[LinkMonitor, dict[str, list[Any]]],
) -> None:
    m, published = downtown
    warm, _ = m.poll(now=999.0)
    assert math.isnan(warm.usable_uplink_bps)  # first counter sample: no rate yet
    status, policy = m.poll(now=1000.0)
    assert status.healthy and status.overlay_path == "relay" and status.band == "n41"
    assert 100_000.0 < status.usable_uplink_bps < 300_000.0  # the measured 0.26 Mbit/s spare
    assert status.rtt_ms == 86.0
    assert policy.video_allowed is False and policy.telemetry_profile == "throttled"
    assert policy.jpeg_hz == 0.5 and policy.source == "auto"
    assert m.path()["overlay_path"] == "relay" and m.path()["peers"][1]["role"] == "ground"
    assert len(published["link_status"]) == 2 and len(published["link_policy"]) == 2


def test_home_wifi_allows_video_with_full_telemetry() -> None:
    m, _ = _monitor("home_wifi")
    try:
        m.poll(now=999.0)
        status, policy = m.poll(now=1000.0)
        assert status.overlay_path == "direct" and status.rtt_ms == 7.0
        assert policy.video_allowed and policy.telemetry_profile == "full"
        assert policy.max_video_bitrate_bps > 1_000_000.0
    finally:
        m.stop()


def test_video_hysteresis_off_after_5s_on_after_20s() -> None:
    m, _ = _monitor("home_5g")
    try:
        m.poll(now=-1.0)
        assert m.poll(now=0.0)[1].video_allowed  # first real reading: on at once
        # The link degrades: same cell, but everything the policy checks turns bad.
        m._ping = type("P", (), {"read": staticmethod(lambda: PingReading(True, 600.0, 10.0))})()
        assert m.poll(now=1.0)[1].video_allowed  # not yet
        assert m.poll(now=6.0)[1].video_allowed is False
        m._ping = type("P", (), {"read": staticmethod(lambda: PingReading(True, 100.0, 0.0))})()
        assert m.poll(now=10.0)[1].video_allowed is False  # recovering, not yet trusted
        assert m.poll(now=31.0)[1].video_allowed
    finally:
        m.stop()


def test_operator_override_wins_and_clears(
    downtown: tuple[LinkMonitor, dict[str, list[Any]]],
) -> None:
    m, _ = downtown
    m.poll(now=999.0)
    m.set_policy(video_allowed=True, max_video_bitrate_bps=250_000.0)
    _, policy = m.poll(now=1000.0)
    assert policy.video_allowed and policy.max_video_bitrate_bps == 250_000.0
    assert policy.source == "override"
    m.clear_policy()
    _, policy = m.poll(now=1001.0)
    assert policy.video_allowed is False and policy.source == "auto"


def test_no_hardware_yields_nulls_and_a_health_flag() -> None:
    m, _ = _monitor("no_hardware")
    try:
        status, policy = m.poll(now=1000.0)
        assert status.healthy is False and status.modem_present is False
        assert math.isnan(status.rsrp_dbm) and math.isnan(status.rtt_ms)
        assert math.isnan(status.usable_uplink_bps)
        assert "no modem" in status.notes and "no overlay status" in status.notes
        assert policy.video_allowed is False and policy.reason == "no measurement"
        assert m.status()["rsrp_dbm"] is None
    finally:
        m.stop()


def test_hardware_sources_degrade_on_a_laptop_without_the_modem() -> None:
    m = LinkMonitor(source="hardware", modem_port="/dev/does-not-exist", interface="no-such-iface")
    for name in ("link_status", "link_policy"):
        getattr(m, name).publish = lambda _msg: None
    m.setup_sources()
    try:
        status, _ = m.poll(now=1000.0)
        assert status.modem_present is False
        assert math.isnan(status.throughput_up_bps)
    finally:
        m.stop()
