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

"""MavlinkIO: the one pymavlink socket of the PX4 stack, its reader thread, and the senders.

``px4-reader`` is the only ``recv_match`` caller in the process. It dispatches every
message into :class:`VehicleState`, feeds SYSTEM_TIME into :class:`Px4Timebase`, and
resolves COMMAND_ACK futures keyed by command id, so no RPC ever blocks on the socket.

The setpoint senders live here and only here. They are not RPCs: the supervisor core
calls them through the :class:`~dimos.robot.px4.supervisor_core.Px4Actuator` protocol
from the tick thread. Exactly one thing produces Offboard setpoints.

Ported from drone-autonomy ``common/mavconn.py:14-93`` and ``common/px4_offboard.py:35-78``
(flown 2026-09-09). Unlike ``dimos/robot/drone/mavlink_connection.py`` this never calls
``wait_heartbeat`` (it mis-latches on a multi-component bus) and filters on source
system 1 and component 1 explicitly.
"""

from __future__ import annotations

from collections import deque
from concurrent.futures import Future
from dataclasses import dataclass, replace
import statistics
import threading
import time
from typing import TYPE_CHECKING, Any

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.robot.px4.mavlink.px4_modes import MASK_POS_YAW, MASK_VEL_YAW, MASK_VEL_YAWRATE
from dimos.robot.px4.mavlink.timebase import Px4Timebase
from dimos.robot.px4.mavlink.vehicle_state import PX4_COMPID, PX4_SYSID, VehicleState, is_from_px4
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from pymavlink.mavutil import MavlinkConnection

logger = setup_logger()

# MAVLink common message ids and command ids used here.
MSG_ID_SYSTEM_TIME = 2
MAV_CMD_SET_MESSAGE_INTERVAL = 511
MAV_CMD_COMPONENT_ARM_DISARM = 400
MAV_CMD_DO_SET_MODE = 176
MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1
MAV_FRAME_LOCAL_NED = 1
MAV_RESULT_ACCEPTED = 0
_MAV_TYPE_ONBOARD_CONTROLLER = 18
_MAV_AUTOPILOT_INVALID = 8
_MAV_STATE_ACTIVE = 4

_RECV_TIMEOUT_S = 0.5
_STAMP_LAG_WINDOW = 600


@dataclass(frozen=True)
class SetpointRecord:
    """The last datagram written, stamped at the moment it left this process."""

    kind: str
    n: float
    e: float
    d: float
    vn: float
    ve: float
    vd: float
    yaw_rad: float | None
    yaw_rate_rad: float | None
    wall_t: float
    seq: int


@dataclass
class _MsgStat:
    received: int = 0
    last_mono: float = 0.0


def _boot_ms() -> int:
    return int(time.monotonic() * 1000) & 0xFFFFFFFF


class MavlinkIO:
    """Owns the socket. Construct cheap; ``start()`` opens, ``stop()`` closes."""

    def __init__(
        self,
        state: VehicleState,
        timebase: Px4Timebase,
        *,
        url: str,
        source_system: int,
        source_component: int,
        target_system: int = PX4_SYSID,
        target_component: int = PX4_COMPID,
        baud: int = 921600,
    ) -> None:
        self._state = state
        self._timebase = timebase
        self._url = url
        self._source_system = source_system
        self._source_component = source_component
        self._target_system = target_system
        self._target_component = target_component
        self._baud = baud
        self._conn: MavlinkConnection | None = None
        self._reader: threading.Thread | None = None
        self._stop = threading.Event()
        self._send_lock = threading.Lock()
        self._stats_lock = threading.Lock()
        self._stats: dict[str, _MsgStat] = {}
        self._bad_data = 0
        self._acks: dict[int, Future[int]] = {}
        self._ack_results: dict[int, int] = {}
        self._last_setpoint: SetpointRecord | None = None
        self._setpoint_seq = 0
        self._stamp_lag_ms: deque[float] = deque(maxlen=_STAMP_LAG_WINDOW)

    @property
    def writer_id(self) -> str:
        return f"{self._source_system}/{self._source_component}"

    @property
    def last_setpoint(self) -> SetpointRecord | None:
        with self._send_lock:
            return self._last_setpoint

    # Lifecycle

    def start(self) -> None:
        # Lazy import: pymavlink is an optional extra and ~60 MB.
        from pymavlink import mavutil

        kwargs: dict[str, Any] = dict(
            source_system=self._source_system,
            source_component=self._source_component,
            autoreconnect=True,
        )
        if self._url.startswith("/dev/"):
            self._conn = mavutil.mavlink_connection(self._url, baud=self._baud, **kwargs)
        else:
            self._conn = mavutil.mavlink_connection(self._url, **kwargs)
        self._conn.target_system = self._target_system
        self._conn.target_component = self._target_component
        self._stop.clear()
        self._reader = threading.Thread(target=self._reader_loop, name="px4-reader", daemon=True)
        self._reader.start()
        logger.info("MAVLink opened", url=self._url, writer=self.writer_id)

    def stop(self) -> None:
        self._stop.set()
        if self._reader is not None:
            self._reader.join(timeout=_RECV_TIMEOUT_S + DEFAULT_THREAD_JOIN_TIMEOUT)
            self._reader = None
        if self._conn is not None:
            self._conn.close()
            self._conn = None
        for fut in self._acks.values():
            if not fut.done():
                fut.cancel()
        self._acks.clear()

    def wait_for_px4(self, timeout_s: float) -> None:
        """Block until a HEARTBEAT from system 1 / component 1 has been seen.

        With a ``udpin`` URL pymavlink cannot transmit until it has received one packet
        (it needs the peer address), so every sender waits on this first.
        """
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self._state.heartbeat is not None:
                return
            if self._stop.wait(0.05):
                raise RuntimeError("MavlinkIO stopped while waiting for PX4")
        raise RuntimeError(
            f"no PX4 heartbeat on {self._url} within {timeout_s:.0f}s "
            "(is mavlink-routerd running and the endpoint configured?)"
        )

    # Reader

    def _reader_loop(self) -> None:
        conn = self._conn
        assert conn is not None
        while not self._stop.is_set():
            msg = conn.recv_match(blocking=True, timeout=_RECV_TIMEOUT_S)
            if msg is None:
                continue
            name = msg.get_type()
            if name == "BAD_DATA":
                self._bad_data += 1
                continue
            now = time.time()
            with self._stats_lock:
                st = self._stats.setdefault(name, _MsgStat())
                st.received += 1
                st.last_mono = time.monotonic()
            self._state.handle(msg, now)
            if not is_from_px4(msg):
                continue
            if name == "SYSTEM_TIME":
                self._timebase.add_system_time(
                    msg.time_unix_usec / 1e6, msg.time_boot_ms / 1e3, now
                )
            elif name == "COMMAND_ACK":
                self._resolve_ack(int(msg.command), int(msg.result))
            elif name == "LOCAL_POSITION_NED" and self._timebase.quality != "none":
                # Receive time minus the converted vehicle stamp: link latency plus
                # timebase error, without any publish or transport delay.
                self._stamp_lag_ms.append(
                    (now - self._timebase.to_utc(msg.time_boot_ms / 1e3)) * 1e3
                )

    def _resolve_ack(self, command: int, result: int) -> None:
        with self._send_lock:
            self._ack_results[command] = result
            fut = self._acks.pop(command, None)
        if fut is not None and not fut.done():
            fut.set_result(result)

    # Senders

    def send_heartbeat(self) -> None:
        conn = self._conn
        if conn is None:
            return
        with self._send_lock:
            conn.mav.heartbeat_send(
                _MAV_TYPE_ONBOARD_CONTROLLER, _MAV_AUTOPILOT_INVALID, 0, 0, _MAV_STATE_ACTIVE
            )

    def send_command(self, command: int, *params: float) -> Future[int]:
        """COMMAND_LONG to PX4; the future resolves with the COMMAND_ACK result code."""
        conn = self._conn
        if conn is None:
            raise RuntimeError("MavlinkIO not started")
        p = list(params) + [0.0] * (7 - len(params))
        fut: Future[int] = Future()
        with self._send_lock:
            self._acks[command] = fut
            conn.mav.command_long_send(
                self._target_system, self._target_component, command, 0, *p[:7]
            )
        return fut

    def set_message_interval(self, message_id: int, hz: float, timeout_s: float) -> bool:
        interval_us = 0.0 if hz <= 0 else 1e6 / hz
        fut = self.send_command(MAV_CMD_SET_MESSAGE_INTERVAL, float(message_id), interval_us)
        try:
            return fut.result(timeout=timeout_s) == MAV_RESULT_ACCEPTED
        except TimeoutError:
            logger.warning("no ack for SET_MESSAGE_INTERVAL", message_id=message_id)
            return False

    # Px4Actuator protocol. Fire-and-forget like the flown supervisor: the core watches
    # HEARTBEAT for the effect, and the ack result is kept for sensor_stats.

    def set_mode(self, main: int, sub: int = 0) -> None:
        self.send_command(MAV_CMD_DO_SET_MODE, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, main, sub)

    def arm(self, value: bool) -> None:
        self.send_command(MAV_CMD_COMPONENT_ARM_DISARM, 1.0 if value else 0.0)

    def send_position_setpoint(self, n: float, e: float, d: float, yaw_rad: float) -> None:
        conn = self._conn
        if conn is None:
            raise RuntimeError("MavlinkIO not started")
        with self._send_lock:
            conn.mav.set_position_target_local_ned_send(
                _boot_ms(),
                self._target_system,
                self._target_component,
                MAV_FRAME_LOCAL_NED,
                MASK_POS_YAW,
                n, e, d, 0, 0, 0, 0, 0, 0, yaw_rad, 0,
            )  # fmt: skip
            self._record_setpoint("pos", n, e, d, 0.0, 0.0, 0.0, yaw_rad, None)

    def send_velocity_setpoint(
        self,
        vn: float,
        ve: float,
        vd: float,
        yaw_rad: float | None = None,
        yaw_rate_rad: float | None = None,
    ) -> None:
        conn = self._conn
        if conn is None:
            raise RuntimeError("MavlinkIO not started")
        if yaw_rad is not None:
            mask, yaw, rate = MASK_VEL_YAW, yaw_rad, 0.0
        else:
            mask, yaw, rate = MASK_VEL_YAWRATE, 0.0, (yaw_rate_rad or 0.0)
        with self._send_lock:
            conn.mav.set_position_target_local_ned_send(
                _boot_ms(),
                self._target_system,
                self._target_component,
                MAV_FRAME_LOCAL_NED,
                mask,
                0, 0, 0, vn, ve, vd, 0, 0, 0, yaw, rate,
            )  # fmt: skip
            self._record_setpoint("vel", 0.0, 0.0, 0.0, vn, ve, vd, yaw_rad, yaw_rate_rad)

    def _record_setpoint(
        self,
        kind: str,
        n: float,
        e: float,
        d: float,
        vn: float,
        ve: float,
        vd: float,
        yaw_rad: float | None,
        yaw_rate_rad: float | None,
    ) -> None:
        # Caller holds _send_lock. Stamp at the write, not at publish time.
        self._setpoint_seq += 1
        self._last_setpoint = SetpointRecord(
            kind, n, e, d, vn, ve, vd, yaw_rad, yaw_rate_rad, time.time(), self._setpoint_seq
        )

    # Diagnostics

    def stats(self) -> dict[str, Any]:
        now = time.monotonic()
        with self._stats_lock:
            per_msg = {
                name: {"received": s.received, "age_s": now - s.last_mono}
                for name, s in sorted(self._stats.items())
            }
        with self._send_lock:
            acks = dict(self._ack_results)
            seq = self._setpoint_seq
        return {
            "messages": per_msg,
            "bad_data": self._bad_data,
            "setpoints_sent": seq,
            "last_ack_result": acks,
            "timebase": {
                "quality": self._timebase.quality,
                "samples": self._timebase.samples,
                "rejected": self._timebase.rejected,
                "stamp_lag_ms_p50": (
                    statistics.median(self._stamp_lag_ms) if self._stamp_lag_ms else None
                ),
            },
        }

    def stats_snapshot(self) -> dict[str, _MsgStat]:
        with self._stats_lock:
            return {k: replace(v) for k, v in self._stats.items()}
