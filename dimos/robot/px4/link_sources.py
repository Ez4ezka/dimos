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

"""What the link monitor reads and how it reads it: modem AT replies, the overlay's status,
ping, interface counters. Parsers are pure; the hardware sources degrade to None, never raise.

Modem: Quectel RM520N-GL on ``/dev/ttyUSB2`` (ModemManager is disabled on the Jetson, it
would lock the port). ``AT+QCSQ`` gives the serving signal, ``AT+QENG="servingcell"`` the
band and cell. Overlay: ``tailscale status --json`` says whether the ground station is
reached directly or through a relay. The replay sources carry the four scenarios measured
on 2026-09-08/09 so the policy can be tested and gated without a modem.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import json
import math
from pathlib import Path
import re
import subprocess
import time
from typing import Any, Protocol

from dimos.utils.logging_config import setup_logger

logger = setup_logger()


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
