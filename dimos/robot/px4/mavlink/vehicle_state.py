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

"""Latest-value store and short time-indexed buffers for PX4 and A8 telemetry. Pure.

Feed every received MAVLink message to ``VehicleState.handle(msg)``. Each entry keeps
two clocks: the wall-clock receive time (the clock the perception stack stamps
``capture_time`` with) and the vehicle boot time from the message, so odometry can be
assembled from LOCAL_POSITION_NED and ATTITUDE at the same vehicle instant.

Ported from drone-autonomy ``common/telemetry.py:18-137`` (flown 2026-09-09). Messages
are duck-typed pymavlink objects (``get_type``, ``get_srcSystem``,
``get_srcComponent``, fields); nothing here imports pymavlink.
"""

from __future__ import annotations

import bisect
from collections import deque
from collections.abc import Iterable
from dataclasses import dataclass
import math
import threading
import time
from typing import Any

from dimos.hardware.gimbal.siyi.frame import (
    FLIGHT_MOUNT,
    MountPreset,
    normalize_attitude,
    quat_to_euler_deg,
)
from dimos.robot.px4.config import A8_COMPID, PX4_COMPID, PX4_SYSID
from dimos.robot.px4.px4_modes import (
    MAIN_OFFBOARD,
    MAV_MODE_FLAG_SAFETY_ARMED,
    decode_custom_mode,
)
from dimos.utils.angles import wrap180

_UINT16_INVALID = 65535
_STATUSTEXT_KEEP = 64
# MAV_SEVERITY names, index = severity value.
STATUSTEXT_SEVERITY = (
    "EMERGENCY",
    "ALERT",
    "CRITICAL",
    "ERROR",
    "WARNING",
    "NOTICE",
    "INFO",
    "DEBUG",
)


def is_from_px4(msg: Any) -> bool:
    return bool(msg.get_srcSystem() == PX4_SYSID and msg.get_srcComponent() == PX4_COMPID)


def is_from_a8(msg: Any) -> bool:
    return bool(msg.get_srcSystem() == PX4_SYSID and msg.get_srcComponent() == A8_COMPID)


def _boot_s(msg: Any) -> float | None:
    """Vehicle boot time in seconds from ``time_boot_ms`` / ``time_usec``, if present."""
    ms = getattr(msg, "time_boot_ms", None)
    if ms is not None:
        return float(ms) / 1e3
    us = getattr(msg, "time_usec", None)
    if us is not None:
        return float(us) / 1e6
    return None


class TimedBuffer:
    """Ring buffer of ``(t, boot, value)`` with linear interpolation (angles wrap-aware).

    Interpolation clamps to the nearest sample outside the buffered range; it never
    extrapolates.
    """

    def __init__(self, seconds: float = 2.0, angular: Iterable[str] = ()) -> None:
        self.seconds = seconds
        self.angular = set(angular)
        self.t: deque[float] = deque()
        self.boot: deque[float | None] = deque()
        self.v: deque[dict[str, float]] = deque()

    def push(self, t: float, value: dict[str, float], boot: float | None = None) -> None:
        self.t.append(t)
        self.boot.append(boot)
        self.v.append(value)
        while self.t and t - self.t[0] > self.seconds:
            self.t.popleft()
            self.boot.popleft()
            self.v.popleft()

    def latest(self) -> tuple[float | None, dict[str, float] | None]:
        return (self.t[-1], self.v[-1]) if self.t else (None, None)

    def latest_boot(self) -> float | None:
        return self.boot[-1] if self.boot else None

    def at(self, t: float) -> dict[str, float] | None:
        """Interpolated value at receive time ``t``, or None if empty."""
        return self._interp(list(self.t), t)

    def at_boot(self, boot: float) -> dict[str, float] | None:
        """Interpolated value at vehicle boot time ``boot``; None if no boot stamps."""
        if any(b is None for b in self.boot):
            return None
        return self._interp([b for b in self.boot if b is not None], boot)

    def _interp(self, ts: list[float], t: float) -> dict[str, float] | None:
        if not ts:
            return None
        i = bisect.bisect_left(ts, t)
        if i <= 0:
            return dict(self.v[0])
        if i >= len(ts):
            return dict(self.v[-1])
        t0, t1 = ts[i - 1], ts[i]
        a, b = self.v[i - 1], self.v[i]
        f = 0.0 if t1 == t0 else (t - t0) / (t1 - t0)
        out: dict[str, float] = {}
        for k in a:
            if k in self.angular:
                out[k] = a[k] + f * wrap180(b[k] - a[k])
            else:
                out[k] = a[k] + f * (b[k] - a[k])
        return out


@dataclass(frozen=True)
class LocalPosition:
    n: float
    e: float
    d: float
    vn: float
    ve: float
    vd: float
    boot_s: float | None
    t: float


@dataclass(frozen=True)
class GlobalPosition:
    lat: float
    lon: float
    alt_msl: float
    rel_alt: float
    hdg_deg: float
    boot_s: float | None
    t: float


@dataclass(frozen=True)
class GpsFix:
    fix: int
    sats: int
    eph: float
    epv: float
    t: float


@dataclass(frozen=True)
class Heartbeat:
    armed: bool
    custom_mode: int
    main: int
    sub: int
    t: float


@dataclass(frozen=True)
class SysStatus:
    present: int
    enabled: int
    health: int
    batt_pct: int
    volt: float
    current_a: float
    t: float


@dataclass(frozen=True)
class RcChannels:
    chan: tuple[int, ...]
    count: int
    rssi: int
    t: float


@dataclass(frozen=True)
class ImuSample:
    """HIGHRES_IMU in the body FRD frame (m/s^2, rad/s)."""

    xacc: float
    yacc: float
    zacc: float
    xgyro: float
    ygyro: float
    zgyro: float
    boot_s: float | None
    t: float


@dataclass(frozen=True)
class HomePosition:
    lat: float
    lon: float
    alt: float
    n: float
    e: float
    d: float
    t: float


@dataclass(frozen=True)
class SystemTime:
    unix_s: float
    boot_s: float
    t: float


@dataclass(frozen=True)
class ServoOutputs:
    """SERVO_OUTPUT_RAW: what PX4 drove each output to, in PWM microseconds."""

    pwm: tuple[int, ...]
    boot_s: float | None
    t: float


@dataclass(frozen=True)
class StatusText:
    seq: int
    severity: int
    text: str
    t: float

    @property
    def severity_name(self) -> str:
        if 0 <= self.severity < len(STATUSTEXT_SEVERITY):
            return STATUSTEXT_SEVERITY[self.severity]
        return str(self.severity)


@dataclass(frozen=True)
class VehicleSnapshot:
    """What the supervisor core sees each tick. Ages are wall-clock seconds at snapshot time."""

    wall: float
    heartbeat: Heartbeat | None
    heartbeat_age: float
    local: LocalPosition | None
    local_age: float
    gps: GpsFix | None
    sys_status: SysStatus | None
    rc: RcChannels | None
    rc_age: float
    landed_state: int | None
    yaw_deg: float | None
    px4_msg_age: float

    @property
    def armed(self) -> bool:
        return bool(self.heartbeat and self.heartbeat.armed)

    @property
    def in_offboard(self) -> bool:
        return bool(self.heartbeat and self.heartbeat.main == MAIN_OFFBOARD)

    @property
    def batt_pct(self) -> int:
        return self.sys_status.batt_pct if self.sys_status else -1


def _age(item: Any, now: float) -> float:
    return math.inf if item is None else now - item.t


class VehicleState:
    """Thread-safe latest-value store fed by the MAVLink reader thread."""

    def __init__(self, gimbal_mount: MountPreset = FLIGHT_MOUNT) -> None:
        self._lock = threading.Lock()
        self._gimbal_mount = gimbal_mount
        self.attitude = TimedBuffer(2.0, angular=("roll", "pitch", "yaw"))
        self.gimbal = TimedBuffer(2.0, angular=("roll", "pitch", "yaw"))
        self.gimbal_flags = 0
        self.gimbal_failure = 0
        self.local: LocalPosition | None = None
        self.global_pos: GlobalPosition | None = None
        self.gps: GpsFix | None = None
        self.home: HomePosition | None = None
        self.heartbeat: Heartbeat | None = None
        self.landed_state: tuple[int, float] | None = None
        self.sys_status: SysStatus | None = None
        self.battery_pct: int | None = None
        self.rc: RcChannels | None = None
        self.imu: ImuSample | None = None
        self.system_time: SystemTime | None = None
        self.servo_outputs: ServoOutputs | None = None
        # PX4's own warnings, kept until the publish loop forwards them.
        self.statustext: deque[StatusText] = deque(maxlen=_STATUSTEXT_KEEP)
        self._statustext_seq = 0
        self.last_px4_msg = 0.0

    @property
    def lock(self) -> threading.Lock:
        return self._lock

    def handle(self, msg: Any, now: float | None = None) -> None:
        """Ingest one MAVLink message. Messages not from PX4 (1/1) or the A8 are ignored."""
        t = time.time() if now is None else now
        with self._lock:
            self._handle_locked(msg, t)

    def _handle_locked(self, msg: Any, t: float) -> None:
        name = msg.get_type()
        if is_from_a8(msg) and name == "GIMBAL_DEVICE_ATTITUDE_STATUS":
            roll, raw_pitch, raw_yaw = quat_to_euler_deg(msg.q)
            pitch, yaw = normalize_attitude(msg.q, self._gimbal_mount)
            self.gimbal.push(
                t,
                dict(roll=roll, pitch=pitch, yaw=yaw, raw_pitch=raw_pitch, raw_yaw=raw_yaw),
                boot=_boot_s(msg),
            )
            self.gimbal_flags = int(msg.flags)
            self.gimbal_failure = int(msg.failure_flags)
            return
        if not is_from_px4(msg):
            return
        self.last_px4_msg = t
        if name == "ATTITUDE":
            self.attitude.push(
                t,
                dict(
                    roll=math.degrees(msg.roll),
                    pitch=math.degrees(msg.pitch),
                    yaw=math.degrees(msg.yaw),
                ),
                boot=_boot_s(msg),
            )
        elif name == "LOCAL_POSITION_NED":
            self.local = LocalPosition(
                n=msg.x, e=msg.y, d=msg.z, vn=msg.vx, ve=msg.vy, vd=msg.vz, boot_s=_boot_s(msg), t=t
            )
        elif name == "GLOBAL_POSITION_INT":
            self.global_pos = GlobalPosition(
                lat=msg.lat / 1e7,
                lon=msg.lon / 1e7,
                alt_msl=msg.alt / 1000.0,
                rel_alt=msg.relative_alt / 1000.0,
                hdg_deg=(msg.hdg / 100.0 if msg.hdg != _UINT16_INVALID else math.nan),
                boot_s=_boot_s(msg),
                t=t,
            )
        elif name == "GPS_RAW_INT":
            self.gps = GpsFix(
                fix=msg.fix_type,
                sats=msg.satellites_visible,
                eph=(msg.eph / 100.0 if msg.eph != _UINT16_INVALID else math.nan),
                epv=(msg.epv / 100.0 if msg.epv != _UINT16_INVALID else math.nan),
                t=t,
            )
        elif name == "HOME_POSITION":
            self.home = HomePosition(
                lat=msg.latitude / 1e7,
                lon=msg.longitude / 1e7,
                alt=msg.altitude / 1000.0,
                n=msg.x,
                e=msg.y,
                d=msg.z,
                t=t,
            )
        elif name == "HEARTBEAT":
            cm = int(msg.custom_mode)
            main, sub = decode_custom_mode(cm)
            self.heartbeat = Heartbeat(
                armed=bool(msg.base_mode & MAV_MODE_FLAG_SAFETY_ARMED),
                custom_mode=cm,
                main=main,
                sub=sub,
                t=t,
            )
        elif name == "EXTENDED_SYS_STATE":
            self.landed_state = (int(msg.landed_state), t)
        elif name == "SYS_STATUS":
            self.sys_status = SysStatus(
                present=msg.onboard_control_sensors_present,
                enabled=msg.onboard_control_sensors_enabled,
                health=msg.onboard_control_sensors_health,
                batt_pct=int(msg.battery_remaining),
                volt=msg.voltage_battery / 1000.0,
                current_a=(getattr(msg, "current_battery", -1) / 100.0),
                t=t,
            )
        elif name == "BATTERY_STATUS":
            self.battery_pct = int(msg.battery_remaining)
        elif name == "RC_CHANNELS":
            chans = tuple(int(getattr(msg, f"chan{i}_raw")) for i in range(1, 19))
            self.rc = RcChannels(chan=chans, count=int(msg.chancount), rssi=int(msg.rssi), t=t)
        elif name == "HIGHRES_IMU":
            self.imu = ImuSample(
                xacc=msg.xacc,
                yacc=msg.yacc,
                zacc=msg.zacc,
                xgyro=msg.xgyro,
                ygyro=msg.ygyro,
                zgyro=msg.zgyro,
                boot_s=_boot_s(msg),
                t=t,
            )
        elif name == "SYSTEM_TIME":
            self.system_time = SystemTime(
                unix_s=msg.time_unix_usec / 1e6, boot_s=msg.time_boot_ms / 1e3, t=t
            )
        elif name == "SERVO_OUTPUT_RAW":
            pwm = tuple(
                int(v)
                for v in (getattr(msg, f"servo{i}_raw", None) for i in range(1, 17))
                if v is not None
            )
            self.servo_outputs = ServoOutputs(pwm=pwm, boot_s=_boot_s(msg), t=t)
        elif name == "STATUSTEXT":
            self._statustext_seq += 1
            raw = msg.text
            text = raw.decode("utf-8", "replace") if isinstance(raw, bytes) else str(raw)
            self.statustext.append(
                StatusText(
                    seq=self._statustext_seq,
                    severity=int(msg.severity),
                    text=text.rstrip("\0"),
                    t=t,
                )
            )

    def age(self, item: Any) -> float:
        """Seconds since ``item`` was received; ``inf`` when there is no item."""
        return _age(item, time.time())

    def attitude_at(self, t: float) -> dict[str, float] | None:
        with self._lock:
            return self.attitude.at(t)

    def gimbal_at(self, t: float) -> dict[str, float] | None:
        with self._lock:
            return self.gimbal.at(t)

    def snapshot(self, now: float | None = None) -> VehicleSnapshot:
        """Immutable copy of what the supervisor needs, with ages computed at ``now``."""
        wall = time.time() if now is None else now
        with self._lock:
            _, att = self.attitude.latest()
            return VehicleSnapshot(
                wall=wall,
                heartbeat=self.heartbeat,
                heartbeat_age=_age(self.heartbeat, wall),
                local=self.local,
                local_age=_age(self.local, wall),
                gps=self.gps,
                sys_status=self.sys_status,
                rc=self.rc,
                rc_age=_age(self.rc, wall),
                landed_state=self.landed_state[0] if self.landed_state else None,
                yaw_deg=att["yaw"] if att else None,
                px4_msg_age=(math.inf if not self.last_px4_msg else wall - self.last_px4_msg),
            )
