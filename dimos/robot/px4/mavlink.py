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

"""The MAVLink layer of the PX4 package: constants, frames, timebase, vehicle state, socket.

Everything that knows MAVLink lives here, in the order a message travels:

* PX4 mode numbers, landed states and the Offboard setpoint type masks (pure data).
* Frame conversions between PX4 (NED world, FRD body) and dimOS (FLU, ROS-like), with the
  same signs as ``dimos/robot/drone/mavlink_connection.py``.
* :class:`Px4Timebase`: vehicle boot time to UTC from SYSTEM_TIME, or min-filtered receive
  time when the vehicle has no GPS clock.
* :class:`VehicleState`: the latest value of every message we use plus short time-indexed
  buffers, so odometry can be assembled at one vehicle instant. Duck-typed pymavlink
  messages in, frozen dataclasses out; nothing here imports pymavlink at module level.
* :class:`MavlinkIO`: the one socket of the stack, its reader thread, the command futures
  and the setpoint senders. It is the only place a byte leaves for the aircraft.

Ported from the drone-autonomy stack flown on 2026-09-09 (``common/px4_offboard.py``,
``common/telemetry.py``, ``common/mavconn.py``). Unlike the DJI module this never calls
``wait_heartbeat`` (it mis-latches on a multi-component bus) and filters on source system 1
component 1 explicitly.
"""

from __future__ import annotations

import bisect
from collections import deque
from collections.abc import Iterable
from concurrent.futures import Future
from dataclasses import dataclass, replace
import math
import statistics
import threading
import time
from typing import TYPE_CHECKING, Any, Literal

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.hardware.gimbal.siyi.frame import (
    FLIGHT_MOUNT,
    MountPreset,
    normalize_attitude,
    quat_to_euler_deg,
)
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.robot.px4.config import A8_COMPID, PX4_COMPID, PX4_SYSID
from dimos.utils.angles import wrap180
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from pymavlink.mavutil import MavlinkConnection

logger = setup_logger()


# PX4 modes, landed states and setpoint masks. custom_mode packs the flight mode as
# main = (custom_mode >> 16) & 0xFF, sub = (custom_mode >> 24) & 0xFF.

MAIN_MANUAL = 1
MAIN_ALTCTL = 2
MAIN_POSCTL = 3
MAIN_AUTO = 4
MAIN_ACRO = 5
MAIN_OFFBOARD = 6
MAIN_STAB = 7

SUB_AUTO_READY = 1
SUB_AUTO_TAKEOFF = 2
SUB_AUTO_LOITER = 3
SUB_AUTO_MISSION = 4
SUB_AUTO_RTL = 5
SUB_AUTO_LAND = 6

MAIN_NAMES: dict[int, str] = {
    MAIN_MANUAL: "MANUAL",
    MAIN_ALTCTL: "ALTCTL",
    MAIN_POSCTL: "POSCTL",
    MAIN_AUTO: "AUTO",
    MAIN_ACRO: "ACRO",
    MAIN_OFFBOARD: "OFFBOARD",
    MAIN_STAB: "STABILIZED",
}
SUB_NAMES: dict[int, str] = {
    SUB_AUTO_READY: "READY",
    SUB_AUTO_TAKEOFF: "TAKEOFF",
    SUB_AUTO_LOITER: "LOITER/HOLD",
    SUB_AUTO_MISSION: "MISSION",
    SUB_AUTO_RTL: "RTL",
    SUB_AUTO_LAND: "LAND",
    7: "FOLLOW",
    8: "PRECLAND",
}

# MAV_LANDED_STATE
LANDED_UNDEFINED = 0
LANDED_ON_GROUND = 1
LANDED_IN_AIR = 2
LANDED_TAKEOFF = 3
LANDED_LANDING = 4

# HEARTBEAT.base_mode bit
MAV_MODE_FLAG_SAFETY_ARMED = 128

# SET_POSITION_TARGET_LOCAL_NED type_mask bits (1 = ignore that field).
_IGN_POS = 0x7
_IGN_VEL = 0x38
_IGN_ACC = 0x1C0
_FORCE = 0x200
_IGN_YAW = 0x400
_IGN_YAWRATE = 0x800
MASK_POS_YAW = _IGN_VEL | _IGN_ACC | _FORCE | _IGN_YAWRATE  # 0xBF8
MASK_VEL_YAW = _IGN_POS | _IGN_ACC | _FORCE | _IGN_YAWRATE  # 0xBC7
MASK_VEL_YAWRATE = _IGN_POS | _IGN_ACC | _FORCE | _IGN_YAW  # 0x7C7


def decode_custom_mode(custom_mode: int) -> tuple[int, int]:
    """``HEARTBEAT.custom_mode`` -> ``(main, sub)``."""
    return (custom_mode >> 16) & 0xFF, (custom_mode >> 24) & 0xFF


def mode_name(main: int | None, sub: int = 0) -> str:
    """Human name such as ``OFFBOARD`` or ``AUTO:LOITER/HOLD``; ``?`` when unknown."""
    if main is None:
        return "?"
    s = MAIN_NAMES.get(main, str(main))
    if main == MAIN_AUTO:
        s += ":" + SUB_NAMES.get(sub, str(sub))
    return s


# Frames. dimOS follows ROS: x forward (north), y left (west), z up. MAVLink LOCAL_NED is
# x north, y east, z down; body IMU data is FRD. Roll unchanged, pitch and yaw negated
# (guarded upstream by test_ned_to_ros_coordinate_conversion).


def ned_to_flu(n: float, e: float, d: float) -> tuple[float, float, float]:
    """NED world vector -> dimOS world vector (x north, y west, z up)."""
    return n, -e, -d


def flu_to_ned(x: float, y: float, z: float) -> tuple[float, float, float]:
    """dimOS world vector -> NED world vector."""
    return x, -y, -z


def frd_to_flu(x: float, y: float, z: float) -> tuple[float, float, float]:
    """Body FRD (forward, right, down) -> body FLU (forward, left, up)."""
    return x, -y, -z


def ned_yaw_to_flu_yaw(yaw_rad: float) -> float:
    """NED heading (clockwise positive from north) -> FLU yaw (counter-clockwise)."""
    return -yaw_rad


def quaternion_from_ned_euler(roll: float, pitch: float, yaw: float) -> Quaternion:
    """MAVLink ATTITUDE euler (rad, NED/FRD) -> dimOS orientation quaternion."""
    return Quaternion.from_euler(Vector3(roll, -pitch, -yaw))


def body_flu_velocity_to_ned(
    forward: float, left: float, up: float, yaw_ned_rad: float
) -> tuple[float, float, float]:
    """Body-frame FLU velocity command -> NED world velocity using the vehicle heading.

    Used to turn a teleop ``Twist`` (body frame) into the LOCAL_NED velocity setpoint
    PX4 expects. ``yaw_ned_rad`` is the ATTITUDE yaw (clockwise from north).
    """
    right = -left
    c, s = math.cos(yaw_ned_rad), math.sin(yaw_ned_rad)
    vn = forward * c - right * s
    ve = forward * s + right * c
    return vn, ve, -up


# Timebase. PX4 stamps LOCAL_POSITION_NED and friends with time_boot_ms; SYSTEM_TIME
# carries boot and unix time together, so unix - boot is the vehicle's own clock. Without
# a GPS clock the fallback is the min-filtered receive_wall - boot: latency only ever
# makes that larger, so the minimum over many samples is the least-biased estimate.

TimebaseQuality = Literal["none", "receive_time", "system_time_partial", "system_time"]

# PX4 reports unix time as 0 / near-epoch until it has a source; anything before this
# (2020-01-01) is not a real wall clock.
_MIN_VALID_UNIX_S = 1577836800.0


class Px4Timebase:
    def __init__(
        self,
        *,
        min_samples: int = 30,
        jump_guard_s: float = 0.5,
        window: int = 300,
    ) -> None:
        self._min_samples = min_samples
        self._jump_guard_s = jump_guard_s
        self._system: deque[float] = deque(maxlen=window)
        self._receive_min: float | None = None
        self.rejected = 0
        self.samples = 0

    def add_system_time(self, unix_s: float, boot_s: float, receive_wall_s: float) -> None:
        """One SYSTEM_TIME message: vehicle unix time, vehicle boot time, our receive time."""
        self.samples += 1
        fallback = receive_wall_s - boot_s
        self._receive_min = (
            fallback if self._receive_min is None else min(self._receive_min, fallback)
        )
        if unix_s < _MIN_VALID_UNIX_S:
            return
        offset = unix_s - boot_s
        if len(self._system) >= self._min_samples:
            # Jump guard: a single wild sample (clock step on the vehicle, corrupted
            # packet) must not drag the median; count it and drop it.
            if abs(offset - statistics.median(self._system)) > self._jump_guard_s:
                self.rejected += 1
                return
        self._system.append(offset)

    @property
    def quality(self) -> TimebaseQuality:
        if len(self._system) >= self._min_samples:
            return "system_time"
        if self._system:
            return "system_time_partial"
        if self._receive_min is not None:
            return "receive_time"
        return "none"

    @property
    def offset_s(self) -> float:
        """Seconds to add to a vehicle boot time to get UTC."""
        if self._system:
            return statistics.median(self._system)
        if self._receive_min is not None:
            return self._receive_min
        raise RuntimeError("Px4Timebase has no samples")

    def to_utc(self, boot_s: float) -> float:
        return boot_s + self.offset_s


# Vehicle state. Each entry keeps two clocks: the wall-clock receive time (what the camera
# stamps frames with) and the vehicle boot time from the message.

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
        self.rc: RcChannels | None = None
        self.imu: ImuSample | None = None
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


# The socket. px4-reader is the only recv_match caller in the process. It feeds every
# message to VehicleState, SYSTEM_TIME to the timebase, and resolves COMMAND_ACK futures
# keyed by command id, so no RPC ever blocks on the socket. The setpoint senders live
# here and only here; the supervisor calls them through the Px4Actuator protocol from
# the tick thread. Exactly one thing produces Offboard setpoints.

# MAVLink common message ids and command ids used here.
MSG_ID_SYSTEM_TIME = 2
MAV_CMD_SET_MESSAGE_INTERVAL = 511
MAV_CMD_COMPONENT_ARM_DISARM = 400
MAV_CMD_DO_SET_MODE = 176
MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW = 1000
MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE = 1001
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

    # Gimbal manager (PX4 is the manager, the A8 is device 154). Ported from
    # drone-autonomy common/gimbal.py:98-113, the calls the known-good keyboard controller
    # made. Only used when the connection is configured to own gimbal control.

    def claim_gimbal_control(self, gimbal_device: int = A8_COMPID) -> Future[int]:
        """Take primary control for our own (system, component)."""
        return self.send_command(
            MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE,
            float(self._source_system),
            float(self._source_component),
            -1.0,
            -1.0,
            0.0,
            0.0,
            float(gimbal_device),
        )

    def send_gimbal_pitchyaw(
        self, pitch_deg: float, yaw_deg: float, gimbal_device: int = A8_COMPID
    ) -> None:
        """Absolute pitch/yaw in degrees, flags 0 = yaw in the body (follow) frame."""
        nan = float("nan")
        self.send_command(
            MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
            pitch_deg,
            yaw_deg,
            nan,
            nan,
            0.0,
            0.0,
            float(gimbal_device),
        )

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
