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

"""LinkMonitor: what the operator link can carry right now, so other modules decide
instead of guessing.

Advisory only: it never touches the flight path. It polls the modem, the overlay network's
own status, an ICMP round trip to the ground station and the interface counters on a
timer thread, and publishes ``link_status`` (the measurements plus one derived
``usable_uplink_bps``) and ``link_policy`` (video allowed, max video bitrate, telemetry
profile). RtspCamera and the relay obey the policy; an operator can override it by RPC.

Degrade, never fail: no modem, no overlay, no serial port yields NaNs plus
``healthy=False``, not an exception, so it starts cleanly on a laptop with none of that.

This is the seed of a shared connection layer for several aircraft. Once a second
embodiment uses it, move it to ``dimos/network/link_monitor.py`` with the message types
under ``dimos/msgs/link_msgs`` (already there) and keep only the modem source here.
Peer discovery by multicast finds nothing on the overlay (it routes packets but carries
no multicast), so peers are configured explicitly; peer transport is a later round.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import threading
import time
from typing import Any, Literal

from pydantic import BaseModel, Field

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.msgs.link_msgs.LinkPolicy import TELEMETRY_FULL, TELEMETRY_THROTTLED, LinkPolicy
from dimos.msgs.link_msgs.LinkStatus import LinkStatus
from dimos.robot.px4.link_sources import (
    SCENARIOS,
    CounterSource,
    IcmpPing,
    IfaceCounters,
    ModemReading,
    ModemSource,
    PathReading,
    PathSource,
    PingReading,
    PingSource,
    ReplayCounters,
    ReplayModem,
    ReplayPath,
    ReplayPing,
    SerialModem,
    SysfsCounters,
    TailscalePath,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class Peer(BaseModel):
    """One explicitly configured tailnet peer. Multicast discovery finds nothing here."""

    name: str
    address: str
    role: Literal["aircraft", "ground"]


class LinkMonitorConfig(ModuleConfig):
    # "hardware" polls the modem, tailscale and ping; "replay" reads a recorded scenario.
    source: Literal["hardware", "replay"] = Field(default="hardware")
    replay_scenario: str = Field(default="downtown_idle")
    peers: tuple[Peer, ...] = Field(
        default=(
            Peer(name="dimosdrone-2", address="100.110.224.46", role="aircraft"),
            Peer(name="gigabyte", address="100.100.114.31", role="ground"),
        )
    )
    # Which peer the round trip and throughput are measured against.
    ground_station: str = Field(default="gigabyte")
    modem_port: str = Field(default="/dev/ttyUSB2")
    # The cellular interface (udev names the RNDIS device wwan5g on the Jetson).
    interface: str = Field(default="wwan5g")
    poll_s: float = Field(default=1.0)
    # Policy thresholds, from the 2026-09-09 measurements: 4 Mbit/s video needs a direct
    # path or a cell with headroom; the downtown relay path had 0.26 Mbit/s spare.
    video_min_uplink_bps: float = Field(default=300_000.0)
    video_max_rtt_ms: float = Field(default=250.0)
    video_max_loss_pct: float = Field(default=2.0)
    # Hysteresis: video drops after this long below the bar and returns only after this
    # long above it, so a single bad ping does not flap the camera.
    video_off_after_s: float = Field(default=5.0)
    video_on_after_s: float = Field(default=20.0)
    video_bitrate_cap_bps: float = Field(default=4_000_000.0)
    video_bitrate_fraction: float = Field(default=0.7)
    jpeg_hz_video_on: float = Field(default=2.0)
    jpeg_hz_video_off: float = Field(default=0.5)
    # Telemetry is "full" only on a direct path with LAN-class latency.
    full_telemetry_max_rtt_ms: float = Field(default=30.0)


@dataclass
class _Override:
    video_allowed: bool | None = None
    max_video_bitrate_bps: float | None = None
    telemetry_profile: str | None = None


def estimate_uplink_bps(modem: ModemReading, path: PathReading, ping: PingReading) -> float:
    """Usable uplink from signal quality, path and latency. An estimate, calibrated against
    the 2026-09-09 measurements (SINR 33 direct: 4 Mbit/s carried; SINR 10..21 on the
    relay: 0.26 Mbit/s spare). NaN when nothing is known."""
    if not modem.present and not ping.reachable:
        return math.nan
    if modem.present and not math.isnan(modem.sinr_db):
        sinr = modem.sinr_db
        # Calibrated so the downtown relay cell (SINR 18, 80 kbit/s of telemetry already on
        # it) lands at the measured 0.26 Mbit/s of spare uplink.
        if sinr >= 25:
            base = 4_000_000.0
        elif sinr >= 20:
            base = 1_200_000.0
        elif sinr >= 15:
            base = 680_000.0
        elif sinr >= 10:
            base = 300_000.0
        else:
            base = 100_000.0
    else:
        base = 2_000_000.0  # not cellular: a Wi-Fi or wired path
    if path.available and not path.direct:
        base *= 0.5
    if not math.isnan(ping.rtt_ms) and ping.rtt_ms > 250.0:
        base = min(base, 150_000.0)
    if not math.isnan(ping.loss_pct) and ping.loss_pct >= 2.0:
        base = min(base, 100_000.0)
    return base


class LinkMonitor(Module):
    """Advisory link measurements and the policy other modules obey. Never in the flight path."""

    config: LinkMonitorConfig

    link_status: Out[LinkStatus]
    link_policy: Out[LinkPolicy]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = threading.Lock()
        self._override = _Override()
        self._last_status: LinkStatus | None = None
        self._last_policy: LinkPolicy | None = None
        self._video_ok_since: float | None = None
        self._video_bad_since: float | None = None
        self._video_allowed = False
        self._prev_counters: IfaceCounters | None = None
        self._warming_up = False
        self._ever_bad = False
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._modem: ModemSource | None = None
        self._path: PathSource | None = None
        self._ping: PingSource | None = None
        self._counters: CounterSource | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self.setup_sources()
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._poll_loop, name="link-poll", daemon=True)
        self._thread.start()

    def setup_sources(self) -> None:
        """Bind the modem, path, ping and counter sources per config (no thread)."""
        cfg = self.config
        gs = next((p for p in cfg.peers if p.name == cfg.ground_station), None)
        gs_address = gs.address if gs else cfg.ground_station
        if cfg.source == "replay":
            scenario = SCENARIOS[cfg.replay_scenario]
            self._modem = ReplayModem(scenario)
            self._path = ReplayPath(scenario, cfg.ground_station)
            self._ping = ReplayPing(scenario)
            self._counters = ReplayCounters(scenario)
        else:
            self._modem = SerialModem(cfg.modem_port)
            self._path = TailscalePath(cfg.ground_station)
            self._ping = IcmpPing(gs_address)
            self._counters = SysfsCounters(cfg.interface)

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
            self._thread = None
        super().stop()

    def _poll_loop(self) -> None:
        # A modem query can take a second; the poll thread absorbs it so no caller waits.
        while not self._stop_event.is_set():
            t0 = time.monotonic()
            self.poll()
            self._stop_event.wait(max(0.0, self.config.poll_s - (time.monotonic() - t0)))

    def poll(self, now: float | None = None) -> tuple[LinkStatus, LinkPolicy]:
        """One measurement round and the policy it implies. Publishes both."""
        assert self._modem and self._path and self._ping and self._counters
        wall = time.time() if now is None else now
        modem = self._modem.read()
        path = self._path.read()
        ping = self._ping.read()
        counters = self._counters.read()
        up_bps, down_bps = self._throughput(counters)
        status = self._status(modem, path, ping, up_bps, down_bps, wall)
        policy = self._policy(status, wall)
        with self._lock:
            self._last_status, self._last_policy = status, policy
        self.link_status.publish(status)
        self.link_policy.publish(policy)
        return status, policy

    def _throughput(self, counters: IfaceCounters | None) -> tuple[float, float]:
        prev, self._prev_counters = self._prev_counters, counters
        if counters is None:
            return math.nan, math.nan
        if prev is None or counters.name != prev.name:
            # First sample on this interface: the rate is unknown, not zero. Callers see
            # "no measurement" for one poll instead of an estimate that ignores the load.
            self._warming_up = True
            return math.nan, math.nan
        self._warming_up = False
        dt = counters.t - prev.t
        if dt <= 0:
            return math.nan, math.nan
        return (counters.tx_bytes - prev.tx_bytes) * 8 / dt, (
            counters.rx_bytes - prev.rx_bytes
        ) * 8 / dt

    def _status(
        self,
        modem: ModemReading,
        path: PathReading,
        ping: PingReading,
        up_bps: float,
        down_bps: float,
        wall: float,
    ) -> LinkStatus:
        cfg = self.config
        usable = estimate_uplink_bps(modem, path, ping)
        if self._warming_up:
            usable = math.nan
        elif not math.isnan(usable) and not math.isnan(up_bps):
            usable = max(0.0, usable - up_bps)
        notes = []
        if not modem.present:
            notes.append("no modem")
        if not path.available:
            notes.append("no overlay status")
        if not ping.reachable:
            notes.append("ground station unreachable")
        return LinkStatus(
            healthy=modem.present and path.available and ping.reachable,
            modem_present=modem.present,
            rsrp_dbm=modem.rsrp_dbm,
            sinr_db=modem.sinr_db,
            rsrq_db=modem.rsrq_db,
            rtt_ms=ping.rtt_ms,
            throughput_up_bps=up_bps,
            throughput_down_bps=down_bps,
            usable_uplink_bps=usable,
            loss_pct=ping.loss_pct,
            band=modem.band,
            cell_id=modem.cell_id,
            interface=cfg.interface if modem.present else "",
            overlay_path=("direct" if path.direct else "relay")
            if path.available and path.online
            else ("none" if path.available else ""),
            ground_station=cfg.ground_station,
            notes="; ".join(notes),
            ts=wall,
        )

    def _policy(self, st: LinkStatus, wall: float) -> LinkPolicy:
        cfg = self.config
        good = (
            not math.isnan(st.usable_uplink_bps)
            and st.usable_uplink_bps >= cfg.video_min_uplink_bps
            and not math.isnan(st.rtt_ms)
            and st.rtt_ms <= cfg.video_max_rtt_ms
            and (math.isnan(st.loss_pct) or st.loss_pct <= cfg.video_max_loss_pct)
        )
        if good:
            self._video_bad_since = None
            self._video_ok_since = self._video_ok_since or wall
            on_delay = cfg.video_on_after_s if self._ever_bad else 0.0
            if not self._video_allowed and wall - self._video_ok_since >= on_delay:
                self._video_allowed = True
        else:
            self._video_ok_since = None
            self._video_bad_since = self._video_bad_since or wall
            if not math.isnan(st.usable_uplink_bps):
                self._ever_bad = True  # a real bad reading, not the warm-up poll
            if self._video_allowed and wall - self._video_bad_since >= cfg.video_off_after_s:
                self._video_allowed = False
        video = self._video_allowed
        bitrate = 0.0
        if video:
            bitrate = min(
                cfg.video_bitrate_cap_bps, cfg.video_bitrate_fraction * st.usable_uplink_bps
            )
        full = (
            st.overlay_path == "direct"
            and not math.isnan(st.rtt_ms)
            and st.rtt_ms <= cfg.full_telemetry_max_rtt_ms
        )
        telemetry = TELEMETRY_FULL if full else TELEMETRY_THROTTLED
        reason = (
            f"uplink {st.usable_uplink_bps / 1e3:.0f} kbps, rtt {st.rtt_ms:.0f} ms, {st.overlay_path or 'unknown path'}"
            if not math.isnan(st.usable_uplink_bps)
            else "no measurement"
        )
        source = "auto"
        with self._lock:
            ov = self._override
        if (
            ov.video_allowed is not None
            or ov.max_video_bitrate_bps is not None
            or ov.telemetry_profile is not None
        ):
            source = "override"
            video = ov.video_allowed if ov.video_allowed is not None else video
            if ov.max_video_bitrate_bps is not None:
                bitrate = ov.max_video_bitrate_bps
            telemetry = ov.telemetry_profile or telemetry
        return LinkPolicy(
            video_allowed=video,
            max_video_bitrate_bps=bitrate if video else 0.0,
            jpeg_hz=cfg.jpeg_hz_video_on if video else cfg.jpeg_hz_video_off,
            telemetry_profile=telemetry,
            source=source,
            reason=reason,
            ts=wall,
        )

    # RPCs

    @rpc
    def status(self) -> dict[str, Any]:
        """The latest LinkStatus as a dict."""
        with self._lock:
            st = self._last_status
        return {} if st is None else _nan_to_none(vars(st))

    @rpc
    def path(self) -> dict[str, Any]:
        """Which way packets reach the ground station right now, and the configured peers."""
        with self._lock:
            st = self._last_status
        return {
            "ground_station": self.config.ground_station,
            "overlay_path": None if st is None else st.overlay_path,
            "interface": None if st is None else st.interface,
            "rtt_ms": None if st is None or math.isnan(st.rtt_ms) else st.rtt_ms,
            "peers": [p.model_dump() for p in self.config.peers],
        }

    @rpc
    def set_policy(
        self,
        video_allowed: bool | None = None,
        max_video_bitrate_bps: float | None = None,
        telemetry_profile: str | None = None,
    ) -> dict[str, Any]:
        """Operator override. Any field left None keeps the derived value; clear with
        ``clear_policy``."""
        with self._lock:
            self._override = _Override(video_allowed, max_video_bitrate_bps, telemetry_profile)
        return {"override": vars(self._override)}

    @rpc
    def clear_policy(self) -> dict[str, Any]:
        """Back to the derived policy."""
        with self._lock:
            self._override = _Override()
        return {"override": None}


def _nan_to_none(d: dict[str, Any]) -> dict[str, Any]:
    return {k: (None if isinstance(v, float) and math.isnan(v) else v) for k, v in d.items()}
