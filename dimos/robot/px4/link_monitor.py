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

What it reads and how, top of the file: Quectel RM520N-GL AT replies on ``/dev/ttyUSB2``
(``AT+QCSQ`` for the serving signal, ``AT+QENG="servingcell"`` for band and cell;
ModemManager is disabled on the Jetson, it would lock the port), ``tailscale status
--json`` for direct-or-relayed, ``ping`` and the interface counters. Parsers are pure; the
hardware sources degrade to "not present", never raise. The replay sources carry the four
scenarios measured on 2026-09-08/09 so the policy is tested and gated without a modem.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import json
import math
from pathlib import Path
import re
import subprocess
import threading
import time
from typing import Any, Literal, Protocol

from pydantic import BaseModel, Field

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.msgs.link_msgs.LinkPolicy import TELEMETRY_FULL, TELEMETRY_THROTTLED, LinkPolicy
from dimos.msgs.link_msgs.LinkStatus import LinkStatus
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


# Readings, parsers, sources and the replay scenarios


@dataclass(frozen=True)
class ModemReading:
    present: bool
    sysmode: str = ""
    rsrp_dbm: float = math.nan
    sinr_db: float = math.nan
    rsrq_db: float = math.nan
    band: str = ""
    cell_id: str = ""


@dataclass(frozen=True)
class PathReading:
    """Overlay path to the ground station, from the overlay's own status."""

    available: bool
    online: bool = False
    direct: bool = False
    relay: str = ""
    address: str = ""


@dataclass(frozen=True)
class PingReading:
    reachable: bool
    rtt_ms: float = math.nan
    loss_pct: float = math.nan


@dataclass(frozen=True)
class IfaceCounters:
    name: str
    tx_bytes: int
    rx_bytes: int
    t: float


# Parsers (pure)


def _num(s: str) -> float:
    try:
        return float(s.strip().strip('"'))
    except ValueError:
        return math.nan


def parse_qcsq(reply: str) -> ModemReading:
    """``+QCSQ: "NR5G-SA",-83,14,-11`` and the LTE / NSA layouts. Unknown -> not present."""
    m = re.search(r"\+QCSQ:\s*(.+)", reply)
    if not m:
        return ModemReading(present=False)
    parts = [p.strip() for p in m.group(1).split(",")]
    mode = parts[0].strip('"')
    if mode in ("NOSERVICE", "") or len(parts) < 2:
        return ModemReading(present=True, sysmode=mode)
    if mode == "NR5G-SA":
        # <NR5G_RSRP>,<NR5G_SINR>,<NR5G_RSRQ>
        return ModemReading(
            present=True,
            sysmode=mode,
            rsrp_dbm=_num(parts[1]),
            sinr_db=_num(parts[2]) if len(parts) > 2 else math.nan,
            rsrq_db=_num(parts[3]) if len(parts) > 3 else math.nan,
        )
    if mode == "NR5G-NSA" and len(parts) >= 8:
        # <lte_rssi>,<lte_rsrp>,<lte_sinr>,<lte_rsrq>,<nr_rsrp>,<nr_sinr>,<nr_rsrq>
        return ModemReading(
            present=True,
            sysmode=mode,
            rsrp_dbm=_num(parts[5]),
            sinr_db=_num(parts[6]),
            rsrq_db=_num(parts[7]),
        )
    # LTE: <rssi>,<rsrp>,<sinr>,<rsrq>
    return ModemReading(
        present=True,
        sysmode=mode,
        rsrp_dbm=_num(parts[2]) if len(parts) > 2 else math.nan,
        sinr_db=_num(parts[3]) if len(parts) > 3 else math.nan,
        rsrq_db=_num(parts[4]) if len(parts) > 4 else math.nan,
    )


def parse_qeng_servingcell(reply: str) -> tuple[str, str]:
    """``(band, cell_id)`` from ``+QENG: "servingcell",...``; empty strings when absent."""
    m = re.search(r'\+QENG:\s*"servingcell",(.+)', reply)
    if not m:
        return "", ""
    parts = [p.strip().strip('"') for p in m.group(1).split(",")]
    # NR5G-SA: <state>,<mode>,<duplex>,<MCC>,<MNC>,<cellID>,<PCID>,<TAC>,<ARFCN>,<band>,...
    # LTE:     <state>,<mode>,<duplex>,<MCC>,<MNC>,<cellID>,<PCID>,<earfcn>,<freq_band>,...
    if len(parts) < 10:
        return "", ""
    mode = parts[1]
    cell_id = parts[5]
    band = parts[9] if mode.startswith("NR5G") else parts[8]
    if mode.startswith("NR5G") and band and not band.startswith("n"):
        band = f"n{band}"
    return band, cell_id


def parse_tailscale_status(status: dict[str, Any], ground_station: str) -> PathReading:
    """Path to the peer named ``ground_station`` (hostname or tailnet address)."""
    for peer in status.get("Peer", {}).values():
        names = {str(peer.get("HostName", "")).lower(), *peer.get("TailscaleIPs", [])}
        if ground_station.lower() in names:
            cur = str(peer.get("CurAddr", "") or "")
            relay = str(peer.get("Relay", "") or "")
            return PathReading(
                available=True,
                online=bool(peer.get("Online", False)),
                direct=bool(cur),
                relay="" if cur else relay,
                address=cur or relay,
            )
    return PathReading(available=True, online=False)


def parse_ping(output: str) -> PingReading:
    loss = re.search(r"(\d+(?:\.\d+)?)% packet loss", output)
    rtt = re.search(r"rtt [^=]*= [\d.]+/([\d.]+)/", output)
    if loss is None:
        return PingReading(reachable=False)
    loss_pct = float(loss.group(1))
    if rtt is None:
        return PingReading(reachable=False, loss_pct=loss_pct)
    return PingReading(reachable=loss_pct < 100.0, rtt_ms=float(rtt.group(1)), loss_pct=loss_pct)


# Sources


class ModemSource(Protocol):
    def read(self) -> ModemReading: ...


class PathSource(Protocol):
    def read(self) -> PathReading: ...


class PingSource(Protocol):
    def read(self) -> PingReading: ...


class CounterSource(Protocol):
    def read(self) -> IfaceCounters | None: ...


class SerialModem:
    """AT over the modem's serial port, opened per poll so nothing else is locked out."""

    def __init__(
        self, port: str = "/dev/ttyUSB2", baud: int = 115200, timeout_s: float = 1.0
    ) -> None:
        self._port = port
        self._baud = baud
        self._timeout_s = timeout_s

    def _at(self, ser: Any, cmd: str) -> str:
        ser.reset_input_buffer()
        ser.write((cmd + "\r\n").encode())
        deadline = time.monotonic() + self._timeout_s
        out = b""
        while time.monotonic() < deadline:
            out += ser.read(ser.in_waiting or 1)
            if b"OK" in out or b"ERROR" in out:
                break
        return out.decode("utf-8", "replace")

    def read(self) -> ModemReading:
        if not Path(self._port).exists():
            return ModemReading(present=False)
        try:
            import serial  # pyserial, optional extra

            with serial.Serial(self._port, self._baud, timeout=0.2) as ser:
                reading = parse_qcsq(self._at(ser, "AT+QCSQ"))
                band, cell = parse_qeng_servingcell(self._at(ser, 'AT+QENG="servingcell"'))
        except (OSError, ImportError) as exc:
            logger.warning("modem unreadable", port=self._port, error=str(exc))
            return ModemReading(present=False)
        return ModemReading(
            present=reading.present,
            sysmode=reading.sysmode,
            rsrp_dbm=reading.rsrp_dbm,
            sinr_db=reading.sinr_db,
            rsrq_db=reading.rsrq_db,
            band=band,
            cell_id=cell,
        )


class TailscalePath:
    def __init__(self, ground_station: str, timeout_s: float = 3.0) -> None:
        self._ground_station = ground_station
        self._timeout_s = timeout_s

    def read(self) -> PathReading:
        try:
            out = subprocess.run(
                ["tailscale", "status", "--json"],
                capture_output=True,
                text=True,
                timeout=self._timeout_s,
                check=False,
            )
        except (OSError, subprocess.TimeoutExpired):
            return PathReading(available=False)
        if out.returncode != 0 or not out.stdout.strip():
            return PathReading(available=False)
        try:
            return parse_tailscale_status(json.loads(out.stdout), self._ground_station)
        except ValueError:
            return PathReading(available=False)


class IcmpPing:
    def __init__(self, address: str, count: int = 3, timeout_s: float = 1.0) -> None:
        self._address = address
        self._count = count
        self._timeout_s = timeout_s

    def read(self) -> PingReading:
        if not self._address:
            return PingReading(reachable=False)
        try:
            out = subprocess.run(
                [
                    "ping",
                    "-n",
                    "-c",
                    str(self._count),
                    "-W",
                    str(int(self._timeout_s)),
                    "-i",
                    "0.2",
                    self._address,
                ],
                capture_output=True,
                text=True,
                timeout=self._count * (self._timeout_s + 0.5) + 2.0,
                check=False,
            )
        except (OSError, subprocess.TimeoutExpired):
            return PingReading(reachable=False)
        return parse_ping(out.stdout)


class SysfsCounters:
    def __init__(self, interface: str) -> None:
        self._iface = interface

    def read(self) -> IfaceCounters | None:
        base = Path("/sys/class/net") / self._iface / "statistics"
        try:
            tx = int((base / "tx_bytes").read_text())
            rx = int((base / "rx_bytes").read_text())
        except (OSError, ValueError):
            return None
        return IfaceCounters(self._iface, tx, rx, time.time())


# Replay: the four scenarios measured on 2026-09-08/09 (numbers real, AT text reconstructed)


@dataclass(frozen=True)
class Scenario:
    name: str
    qcsq: str
    qeng: str
    tailscale: dict[str, Any]
    ping: str
    interface: str
    tx_bps: float
    expect_video: bool
    expect_telemetry: str
    note: str = ""
    extra: dict[str, Any] = field(default_factory=dict)


def _ts(direct: bool, online: bool = True) -> dict[str, Any]:
    return {
        "Self": {"HostName": "dimosdrone-2", "TailscaleIPs": ["100.110.224.46"]},
        "Peer": {
            "gs": {
                "HostName": "gigabyte",
                "TailscaleIPs": ["100.100.114.31"],
                "Online": online,
                "Relay": "" if direct else "sfo",
                "CurAddr": "10.0.0.188:41641" if direct else "",
            }
        },
    }


def _ping(avg_ms: float, loss_pct: float) -> str:
    return (
        f"3 packets transmitted, {3 - round(3 * loss_pct / 100)} received, {loss_pct:.0f}% packet loss, time 400ms\n"
        f"rtt min/avg/max/mdev = {avg_ms * 0.8:.3f}/{avg_ms:.3f}/{avg_ms * 1.4:.3f}/1.000 ms\n"
    )


SCENARIOS: dict[str, Scenario] = {
    "home_wifi": Scenario(
        name="home_wifi",
        qcsq='+QCSQ: "NR5G-SA",-73,33,-10\r\n\r\nOK\r\n',
        qeng='+QENG: "servingcell","NOCONN","NR5G-SA","TDD",310,260,1A2B3C,42,7F01,520110,41,12,-73,-10,33,1,-\r\n\r\nOK\r\n',
        tailscale=_ts(direct=True),
        ping=_ping(7.0, 0.0),
        interface="wlP1p1s0",
        tx_bps=50_000.0,
        expect_video=True,
        expect_telemetry="full",
        note="Jetson on home Wi-Fi, laptop on the LAN; the modem is up but not the default route",
    ),
    "home_5g": Scenario(
        name="home_5g",
        qcsq='+QCSQ: "NR5G-SA",-73,33,-10\r\n\r\nOK\r\n',
        qeng='+QENG: "servingcell","NOCONN","NR5G-SA","TDD",310,260,1A2B3C,42,7F01,520110,41,12,-73,-10,33,1,-\r\n\r\nOK\r\n',
        tailscale=_ts(direct=True),
        ping=_ping(100.0, 0.0),
        interface="wwan5g",
        tx_bps=200_000.0,
        expect_video=True,
        expect_telemetry="throttled",
        note="Jetson on 5G only, laptop on the LAN, direct path: 4 Mbit/s video worked",
    ),
    "downtown_idle": Scenario(
        name="downtown_idle",
        qcsq='+QCSQ: "NR5G-SA",-83,18,-11\r\n\r\nOK\r\n',
        qeng='+QENG: "servingcell","NOCONN","NR5G-SA","TDD",310,260,2B3C4D,17,7F02,520110,41,12,-83,-11,18,1,-\r\n\r\nOK\r\n',
        tailscale=_ts(direct=False),
        ping=_ping(86.0, 0.0),
        interface="wwan5g",
        tx_bps=115_000.0,
        expect_video=False,
        expect_telemetry="throttled",
        note="both ends cellular through the DERP relay, throttled telemetry only: 0.26 Mbit/s spare",
    ),
    "downtown_video": Scenario(
        name="downtown_video",
        qcsq='+QCSQ: "NR5G-SA",-85,12,-12\r\n\r\nOK\r\n',
        qeng='+QENG: "servingcell","NOCONN","NR5G-SA","TDD",310,260,2B3C4D,17,7F02,520110,41,12,-85,-12,12,1,-\r\n\r\nOK\r\n',
        tailscale=_ts(direct=False),
        ping=_ping(413.0, 5.0),
        interface="wwan5g",
        tx_bps=4_000_000.0,
        expect_video=False,
        expect_telemetry="throttled",
        note="same cell with 4 Mbit/s video: 413 ms, 5 % loss, QGC stalls",
    ),
    "no_hardware": Scenario(
        name="no_hardware",
        qcsq="",
        qeng="",
        tailscale={},
        ping="",
        interface="",
        tx_bps=0.0,
        expect_video=False,
        expect_telemetry="throttled",
        note="a laptop with no modem and no overlay: nulls plus a health flag, no exception",
    ),
}


class ReplayModem:
    def __init__(self, scenario: Scenario) -> None:
        self._s = scenario

    def read(self) -> ModemReading:
        if not self._s.qcsq:
            return ModemReading(present=False)
        r = parse_qcsq(self._s.qcsq)
        band, cell = parse_qeng_servingcell(self._s.qeng)
        return ModemReading(True, r.sysmode, r.rsrp_dbm, r.sinr_db, r.rsrq_db, band, cell)


class ReplayPath:
    def __init__(self, scenario: Scenario, ground_station: str) -> None:
        self._s = scenario
        self._gs = ground_station

    def read(self) -> PathReading:
        if not self._s.tailscale:
            return PathReading(available=False)
        return parse_tailscale_status(self._s.tailscale, self._gs)


class ReplayPing:
    def __init__(self, scenario: Scenario) -> None:
        self._s = scenario

    def read(self) -> PingReading:
        return parse_ping(self._s.ping) if self._s.ping else PingReading(reachable=False)


class ReplayCounters:
    """Counters that grow at the scenario's measured uplink rate."""

    def __init__(self, scenario: Scenario) -> None:
        self._s = scenario
        self._t0 = time.time()

    def read(self) -> IfaceCounters | None:
        if not self._s.interface:
            return None
        now = time.time()
        tx = int((now - self._t0) * self._s.tx_bps / 8)
        return IfaceCounters(self._s.interface, tx, tx // 4, now)


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
