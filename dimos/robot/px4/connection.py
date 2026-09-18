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

"""PX4 drone connection Module: MAVLink bridge + Offboard flight supervisor, one process.

Owns the one MAVLink socket of the dimOS stack (mavlink-router endpoint 14556, component
195) and exposes the vehicle as dimos streams: odometry, IMU, motor outputs, GPS,
battery, RC, gimbal attitude and status, plus the flight supervisor's own streams.
Consumes teleop ``cmd_vel``, the perception target and E-STOP.

Mirrors ``dimos/robot/galaxea/r1pro/connection.py``: Field-defaulted config, lazy driver
import in ``start()``, socket opened in ``start()`` never ``__init__``, per-message stat
counters behind a ``sensor_stats`` RPC, a drift-free publish loop, and the static mount
edge republished on every tf tick. The flight logic (``supervisor_core.py``) and the
MAVLink layer (``mavlink.py``: socket, vehicle state, timebase, frames) are plain classes
this module owns, the way R1ProConnection owns its RawROS nodes and sensor workers.

Safety invariants (README): the RC pilot always wins; exactly one writer of Offboard
setpoints (this module's tick thread through MavlinkIO); if it stops, the stream stops and
PX4's Offboard-loss failsafe takes over; E-STOP is Hold plus a latch; no arm, mode or
setpoint RPC exists on this surface.
"""

from __future__ import annotations

from collections import deque
from collections.abc import Callable
import dataclasses
import json
import math
import socket
import statistics
import threading
import time
from typing import Any, Literal

from dimos_lcm.std_msgs import Bool  # type: ignore[import-untyped]
from pydantic import Field
from reactivex.disposable import Disposable

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.hardware.gimbal.siyi.frame import (
    MOUNT_PRESETS,
    PITCH_MAX_DEG,
    PITCH_MIN_DEG,
    YAW_MAX_DEG,
    YAW_MIN_DEG,
)
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.px4_msgs.CommandEvent import CommandEvent
from dimos.msgs.px4_msgs.VehicleStatus import VehicleStatus
from dimos.msgs.sensor_msgs.BatteryState import (
    POWER_SUPPLY_STATUS_DISCHARGING,
    POWER_SUPPLY_TECHNOLOGY_LIPO,
    BatteryState,
)
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.Joy import Joy
from dimos.msgs.sensor_msgs.NavSatFix import (
    COVARIANCE_TYPE_DIAGONAL_KNOWN,
    COVARIANCE_TYPE_UNKNOWN,
    STATUS_FIX,
    STATUS_GBAS_FIX,
    STATUS_NO_FIX,
    STATUS_SBAS_FIX,
    NavSatFix,
)
from dimos.msgs.std_msgs.String import String
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.robot.px4.config import (
    GIMBAL_MOUNT_XYZ_UNMEASURED,
    LEGACY_SUPERVISOR_LOCK_PORT,
    PX4_HARDWARE,
    GuidanceConfig,
    SupervisorLimits,
)
from dimos.robot.px4.mavlink import (
    MAIN_AUTO,
    MSG_ID_SYSTEM_TIME,
    SUB_AUTO_LOITER,
    MavlinkIO,
    Px4Timebase,
    VehicleSnapshot,
    VehicleState,
    frd_to_flu,
    mode_name,
    ned_to_flu,
    ned_yaw_to_flu_yaw,
    quaternion_from_ned_euler,
)
from dimos.robot.px4.supervisor_core import (
    ARMED_STATES,
    GUIDANCE_MODES,
    GuidanceMode,
    Rejection,
    SupervisorCore,
    TargetEstimate,
    TeleopCommand,
)
from dimos.utils.angles import clamp
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# One 100 Hz loop serves every output stream on its own divisor, like R1ProConnection's
# publish loop; a thread per stream would buy nothing for messages this small.
_PUBLISH_BASE_HZ = 100.0
_JITTER_WINDOW = 2000
_GIMBAL_MAX_AGE_S = 1.0
_GIMBAL_JOINTS = ["gimbal_roll", "gimbal_pitch", "gimbal_yaw"]
_GIMBAL_TARGET_JOINTS = ("gimbal_pitch", "gimbal_yaw")
_TELEOP_DEADBAND = 1e-3


class Px4DroneConnectionConfig(ModuleConfig):
    # Our own mavlink-router endpoint. Never 14550 (gimbal controller) or 14552 (flown
    # supervisor); the whole port table is ROUTER_ENDPOINTS in config.py.
    mav_url: str = Field(default=PX4_HARDWARE.mav_url)
    # A component of the vehicle (system 1), never a ground station (255).
    source_system: int = Field(default=PX4_HARDWARE.source_system)
    source_component: int = Field(default=PX4_HARDWARE.source_component)
    target_system: int = Field(default=PX4_HARDWARE.target_system)
    target_component: int = Field(default=PX4_HARDWARE.target_component)
    gimbal_component: int = Field(default=PX4_HARDWARE.gimbal_component)
    connect_timeout_s: float = Field(default=PX4_HARDWARE.connect_timeout_s)
    heartbeat_hz: float = Field(default=PX4_HARDWARE.heartbeat_hz)
    ack_timeout_s: float = Field(default=PX4_HARDWARE.ack_timeout_s)
    odom_frame_id: str = Field(default="odom")
    base_frame_id: str = Field(default="base_link")
    gimbal_base_frame_id: str = Field(default="gimbal_base")
    # base_link -> gimbal_base, metres, FLU. The default is the UNMEASURED placeholder and
    # the module warns at start until it is replaced.
    gimbal_mount_xyz: tuple[float, float, float] = Field(default=PX4_HARDWARE.gimbal_mount_xyz)
    # "flight" = A8 hanging under the frame (verified 2026-09-04); "bench" = base down.
    gimbal_mount_preset: Literal["flight", "bench"] = Field(default="flight")
    # The static base_link -> gimbal_base edge rides on every tf tick here, unless the
    # SiyiA8Gimbal module runs and publishes the whole gimbal chain itself.
    publish_gimbal_mount_tf: bool = Field(default=True)
    odom_hz: float = Field(default=PX4_HARDWARE.odom_hz)
    imu_hz: float = Field(default=PX4_HARDWARE.imu_hz)
    motor_outputs_hz: float = Field(default=PX4_HARDWARE.motor_outputs_hz)
    rc_hz: float = Field(default=PX4_HARDWARE.rc_hz)
    status_hz: float = Field(default=PX4_HARDWARE.status_hz)
    gps_hz: float = Field(default=PX4_HARDWARE.gps_hz)
    battery_hz: float = Field(default=PX4_HARDWARE.battery_hz)
    gimbal_hz: float = Field(default=PX4_HARDWARE.gimbal_hz)
    statustext_hz: float = Field(default=PX4_HARDWARE.statustext_hz)
    robot_state_hz: float = Field(default=PX4_HARDWARE.robot_state_hz)
    # "system_time" converts vehicle boot time to UTC from SYSTEM_TIME (GPS-disciplined);
    # "receive_time" stamps with the Jetson wall clock at receipt.
    timebase_source: Literal["system_time", "receive_time"] = Field(default="system_time")
    # SYSTEM_TIME samples wanted before the tick loop starts, and how long to wait for them.
    timebase_samples: int = Field(default=30)
    timebase_timeout_s: float = Field(default=10.0)
    # PX4 streams SYSTEM_TIME at 1 Hz by default; ask for this rate so the samples arrive
    # in seconds, not half a minute (0 = leave the vehicle setting alone).
    system_time_hz: float = Field(default=10.0)
    # Off: gimbal_target is counted and dropped and the flown controller on component 191
    # keeps control of the A8. On: this module claims primary gimbal control at start.
    gimbal_commands_enabled: bool = Field(default=False)
    gimbal_command_hz: float = Field(default=PX4_HARDWARE.gimbal_command_hz)
    # SITL: the RC enable switch is faked by the sitl_enable RPC and RC checks are skipped.
    sitl: bool = Field(default=False)
    sensor_stats_interval_s: float = Field(default=10.0)
    # The flown supervisor binds this UDP port; holding it refuses to start beside it and
    # keeps it from starting beside us (0 disables the lock).
    writer_lock_port: int = Field(default=LEGACY_SUPERVISOR_LOCK_PORT)
    tick_hz: float = Field(default=PX4_HARDWARE.tick_hz)
    limits: SupervisorLimits = Field(default_factory=SupervisorLimits)
    guidance: GuidanceConfig = Field(default_factory=GuidanceConfig)


def _divisor(hz: float) -> int:
    return max(1, round(_PUBLISH_BASE_HZ / hz)) if hz > 0 else 0


def _navsat_status(fix_type: int) -> int:
    # MAVLink GPS_FIX_TYPE: 0/1 none, 2 2D, 3 3D, 4 DGPS, 5 RTK float, 6 RTK fixed.
    if fix_type >= 5:
        return STATUS_GBAS_FIX
    if fix_type == 4:
        return STATUS_SBAS_FIX
    if fix_type >= 2:
        return STATUS_FIX
    return STATUS_NO_FIX


def _nan_if_inf(v: float) -> float:
    return math.nan if math.isinf(v) else v


class Px4DroneConnection(Module):
    """PX4 drone Module: the MAVLink bridge and the Offboard supervisor, one socket, one writer."""

    # Unlike R1ProConnection this runs a 20 Hz flight loop; its own process keeps the tick
    # jitter away from every other module's Python interpreter.
    dedicated_worker = True

    config: Px4DroneConnectionConfig

    # Control inputs.
    cmd_vel: In[Twist]
    gimbal_target: In[JointState]
    estop_in: In[Bool]

    # Perception inputs (PerceptionBridge).
    target_state: In[Odometry]
    target_valid: In[Bool]
    target_los: In[PoseStamped]

    # Vehicle feedback.
    odometry: Out[Odometry]
    odom: Out[PoseStamped]
    tf: Out[TFMessage]
    imu: Out[Imu]
    motor_outputs: Out[JointState]
    gps: Out[NavSatFix]
    battery: Out[BatteryState]
    rc: Out[Joy]
    gimbal_attitude: Out[JointState]
    global_pose: Out[PoseStamped]
    vehicle_status: Out[VehicleStatus]
    statustext: Out[String]

    # Supervisor streams.
    supervisor_state: Out[String]
    command_event: Out[CommandEvent]
    offboard_setpoint: Out[Odometry]
    robot_state: Out[bytes]
    stop_movement: Out[Bool]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._state: VehicleState | None = None
        self._timebase = Px4Timebase()
        self._io: MavlinkIO | None = None
        self._core: SupervisorCore | None = None
        # Guards the core between the tick thread and the RPC threads.
        self._core_lock = threading.Lock()
        self._writer_lock_sock: socket.socket | None = None
        self._stop_event = threading.Event()
        # Joined in this order on stop: tick first so no setpoint can leave after teardown
        # starts, then heartbeat, then publish; the reader goes with MavlinkIO.
        self._threads: list[tuple[str, threading.Thread]] = []
        self.stop_sequence: list[str] = []
        self._jitter_ms: deque[float] = deque(maxlen=_JITTER_WINDOW)
        self._last_setpoint_seq = 0
        self._last_motor_boot: float | None = None
        self._last_statustext_seq = 0
        self._target_valid = False
        self._target_los_yaw_body_deg: float | None = None
        self._target_los_t = 0.0
        self._cmd_vel_rejections: dict[str, int] = {}
        self._cmd_vel_accepted = 0
        self._cmd_vel_moving = False
        self._cmd_vel_last_verdict: str | None = None
        self._gimbal_target_dropped = 0
        self._gimbal_target_sent = 0
        self._gimbal_last_send = 0.0
        self._event_seq = 0
        self._event_lock = threading.Lock()

    # Lifecycle

    @rpc
    def start(self) -> None:
        super().start()
        cfg = self.config
        self._acquire_writer_lock()
        self._state = VehicleState(gimbal_mount=MOUNT_PRESETS[cfg.gimbal_mount_preset])
        self._core = SupervisorCore(cfg.limits, cfg.guidance, sitl=cfg.sitl)
        self._io = MavlinkIO(
            self._state,
            self._timebase,
            url=cfg.mav_url,
            source_system=cfg.source_system,
            source_component=cfg.source_component,
            target_system=cfg.target_system,
            target_component=cfg.target_component,
        )
        self._io.start()
        self._io.wait_for_px4(cfg.connect_timeout_s)
        self._io.send_heartbeat()
        if cfg.publish_gimbal_mount_tf and cfg.gimbal_mount_xyz == GIMBAL_MOUNT_XYZ_UNMEASURED:
            logger.warning(
                "gimbal_mount_xyz is the UNMEASURED placeholder; the tf gimbal edge is a guess"
            )
        if cfg.timebase_source == "system_time":
            self._collect_timebase()
        if cfg.gimbal_commands_enabled:
            self._io.claim_gimbal_control(cfg.gimbal_component)
            logger.warning("claimed primary gimbal control", component=cfg.source_component)

        self.register_disposable(Disposable(self.cmd_vel.subscribe(self._on_cmd_vel)))
        self.register_disposable(Disposable(self.gimbal_target.subscribe(self._on_gimbal_target)))
        self.register_disposable(Disposable(self.target_state.subscribe(self._on_target_state)))
        self.register_disposable(Disposable(self.target_valid.subscribe(self._on_target_valid)))
        self.register_disposable(Disposable(self.target_los.subscribe(self._on_target_los)))
        self.register_disposable(Disposable(self.estop_in.subscribe(self._on_estop_in)))

        self._stop_event.clear()
        self._threads = [
            (name, threading.Thread(target=fn, name=name, daemon=True))
            for name, fn in (
                ("px4-tick", self._tick_loop),
                ("px4-heartbeat", self._heartbeat_loop),
                ("px4-publish", self._publish_loop),
                ("px4-stats", self._stats_loop),
            )
        ]
        for _, t in self._threads:
            t.start()
        logger.info(
            "Px4DroneConnection started",
            writer=self._io.writer_id,
            timebase=self._timebase.quality,
            sitl=cfg.sitl,
        )

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        self.stop_sequence = []
        for name, t in self._threads:
            if t.is_alive():
                t.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
            self.stop_sequence.append(name)
        self._threads = []
        if self._io is not None:
            self._io.stop()
            self._io = None
            self.stop_sequence.append("px4-io")
        if self._writer_lock_sock is not None:
            self._writer_lock_sock.close()
            self._writer_lock_sock = None
            self.stop_sequence.append("writer-lock")
        super().stop()

    def _acquire_writer_lock(self) -> None:
        port = self.config.writer_lock_port
        if port <= 0:
            return
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.bind(("127.0.0.1", port))
        except OSError as e:
            sock.close()
            raise RuntimeError(
                f"another Offboard writer holds UDP 127.0.0.1:{port} "
                "(flight_supervisor.py still running?); refusing to start"
            ) from e
        self._writer_lock_sock = sock

    def _collect_timebase(self) -> None:
        cfg = self.config
        assert self._io is not None
        if cfg.system_time_hz > 0:
            self._io.set_message_interval(MSG_ID_SYSTEM_TIME, cfg.system_time_hz, cfg.ack_timeout_s)
        deadline = time.monotonic() + cfg.timebase_timeout_s
        while time.monotonic() < deadline and self._timebase.samples < cfg.timebase_samples:
            time.sleep(0.05)
        if self._timebase.quality != "system_time":
            logger.warning(
                "timebase below target quality",
                quality=self._timebase.quality,
                samples=self._timebase.samples,
            )

    # Command events (what the command tracker reads)

    def _emit_event(
        self,
        command: str,
        argument: str,
        source: str,
        request_ts: float,
        state_before: str,
        why: Rejection | None,
    ) -> None:
        core = self._core
        with self._event_lock:
            self._event_seq += 1
            seq = self._event_seq
        self.command_event.publish(
            CommandEvent(
                request_id=seq,
                command=command,
                argument=argument,
                source=source,
                verdict_ts=time.time(),
                accepted=why is None,
                rejection="" if why is None else why.value,
                state_before=state_before,
                state_after=core.state if core else "",
                ts=request_ts,
            )
        )

    # Input handlers

    def _on_cmd_vel(self, msg: Twist) -> None:
        core = self._core
        if core is None:
            return
        now = time.time()
        cmd = TeleopCommand(
            forward=float(msg.linear.x),
            left=float(msg.linear.y),
            up=float(msg.linear.z),
            yaw_rate_ccw=float(msg.angular.z),
            t=now,
        )
        with self._core_lock:
            before = core.state
            why = core.on_cmd_vel(cmd, now)
        if why is None:
            self._cmd_vel_accepted += 1
        else:
            self._cmd_vel_rejections[why.value] = self._cmd_vel_rejections.get(why.value, 0) + 1
        # One event per keypress, not per 20 Hz frame: on the motion edges and whenever the
        # verdict changes while a key is held.
        moving = (
            max(abs(cmd.forward), abs(cmd.left), abs(cmd.up), abs(cmd.yaw_rate_ccw))
            > _TELEOP_DEADBAND
        )
        verdict = "ok" if why is None else why.value
        argument = f"{cmd.forward:.2f},{cmd.left:.2f},{cmd.up:.2f},{cmd.yaw_rate_ccw:.2f}"
        if moving and (not self._cmd_vel_moving or verdict != self._cmd_vel_last_verdict):
            self._emit_event("cmd_vel", argument, "cmd_vel", now, before, why)
        elif not moving and self._cmd_vel_moving:
            self._emit_event("cmd_vel_release", argument, "cmd_vel", now, before, None)
        self._cmd_vel_moving = moving
        self._cmd_vel_last_verdict = verdict if moving else None

    def _on_gimbal_target(self, msg: JointState) -> None:
        io = self._io
        cfg = self.config
        if io is None or not cfg.gimbal_commands_enabled:
            self._gimbal_target_dropped += 1
            return
        now = time.monotonic()
        if now - self._gimbal_last_send < 1.0 / cfg.gimbal_command_hz:
            return
        angles = dict(zip(msg.name, msg.position, strict=False))
        try:
            pitch = math.degrees(angles["gimbal_pitch"])
            yaw = math.degrees(angles["gimbal_yaw"])
        except KeyError:
            logger.warning(
                "gimbal_target needs joints", expected=_GIMBAL_TARGET_JOINTS, got=msg.name
            )
            self._gimbal_target_dropped += 1
            return
        io.send_gimbal_pitchyaw(
            clamp(pitch, PITCH_MIN_DEG, PITCH_MAX_DEG),
            clamp(yaw, YAW_MIN_DEG, YAW_MAX_DEG),
            cfg.gimbal_component,
        )
        self._gimbal_last_send = now
        self._gimbal_target_sent += 1

    def _on_target_valid(self, msg: Bool) -> None:
        self._target_valid = bool(msg.data)

    def _on_target_los(self, msg: PoseStamped) -> None:
        # Line of sight in base_link (FLU, counter-clockwise positive); the gimbal yaw
        # convention is body-relative clockwise positive.
        self._target_los_yaw_body_deg = -math.degrees(msg.yaw)
        self._target_los_t = time.time()

    def _on_target_state(self, msg: Odometry) -> None:
        core = self._core
        if core is None:
            return
        now = time.time()
        n, e, _ = ned_to_flu(msg.x, msg.y, msg.z)  # FLU -> NED is the same sign flip
        vn, ve, _ = ned_to_flu(msg.vx, msg.vy, 0.0)
        los_fresh = now - self._target_los_t <= self.config.limits.target_stale_s
        target = TargetEstimate(
            valid=self._target_valid,
            n=n,
            e=e,
            vn=vn,
            ve=ve,
            los_valid=los_fresh,
            gimbal_yaw_body_deg=self._target_los_yaw_body_deg if los_fresh else None,
        )
        with self._core_lock:
            core.on_target(target, now)

    def _on_estop_in(self, msg: Bool) -> None:
        if msg.data:
            self._estop("estop_in")

    # Threads

    def _tick_loop(self) -> None:
        core, io, state = self._core, self._io, self._state
        assert core is not None and io is not None and state is not None
        period = 1.0 / self.config.tick_hz
        next_tick = time.perf_counter()
        last_state = core.state
        while not self._stop_event.is_set():
            actual = time.perf_counter()
            now = time.time()
            snap = state.snapshot(now)
            with self._core_lock:
                try:
                    core.step(snap, io, now)
                    core.stream(io, now)
                except Exception:
                    logger.exception("supervisor step raised; holding")
                    if core.state in ARMED_STATES:
                        core.sp = None
                        if snap.in_offboard:
                            io.set_mode(MAIN_AUTO, SUB_AUTO_LOITER)
                        core.goto("ABORT", "exception in supervisor", now)
                transitions, core.transitions = core.transitions, []
            for new_state, reason, _ in transitions:
                logger.info("supervisor", state=new_state, reason=reason)
                self.supervisor_state.publish(String(new_state))
                if new_state in ("IDLE", "ABORT") and last_state in ARMED_STATES:
                    self.stop_movement.publish(Bool(data=True))
                last_state = new_state
            self._publish_setpoint_mirror(io)

            next_tick += period
            sleep_for = next_tick - time.perf_counter()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                next_tick = time.perf_counter()
            self._jitter_ms.append((time.perf_counter() - actual - period) * 1e3)

    def _publish_setpoint_mirror(self, io: MavlinkIO) -> None:
        rec = io.last_setpoint
        if rec is None or rec.seq == self._last_setpoint_seq:
            return
        self._last_setpoint_seq = rec.seq
        x, y, z = ned_to_flu(rec.n, rec.e, rec.d)
        vx, vy, vz = ned_to_flu(rec.vn, rec.ve, rec.vd)
        yaw = ned_yaw_to_flu_yaw(rec.yaw_rad) if rec.yaw_rad is not None else 0.0
        wz = -(rec.yaw_rate_rad or 0.0)
        self.offboard_setpoint.publish(
            Odometry(
                ts=rec.wall_t,
                frame_id=self.config.odom_frame_id,
                child_frame_id=f"setpoint_{rec.kind}",
                pose=Pose(Vector3(x, y, z), Quaternion.from_euler(Vector3(0.0, 0.0, yaw))),
                twist=Twist(Vector3(vx, vy, vz), Vector3(0.0, 0.0, wz)),
            )
        )

    def _heartbeat_loop(self) -> None:
        io = self._io
        assert io is not None
        period = 1.0 / self.config.heartbeat_hz
        while not self._stop_event.is_set():
            io.send_heartbeat()
            self._stop_event.wait(period)

    def _stats_loop(self) -> None:
        interval = self.config.sensor_stats_interval_s
        if interval <= 0:
            return
        io = self._io
        assert io is not None
        prev = io.stats_snapshot()
        prev_t = time.monotonic()
        while not self._stop_event.wait(interval):
            cur = io.stats_snapshot()
            now = time.monotonic()
            dt = now - prev_t
            rates = {
                name: round((s.received - prev[name].received) / dt, 1) if name in prev else None
                for name, s in sorted(cur.items())
            }
            logger.info("PX4 message rates (Hz)", window_s=round(dt), **rates)
            prev, prev_t = cur, now

    def _publish_loop(self) -> None:
        cfg = self.config
        state = self._state
        assert state is not None
        period = 1.0 / _PUBLISH_BASE_HZ
        divisors = {
            "odom": _divisor(cfg.odom_hz),
            "imu": _divisor(cfg.imu_hz),
            "motor_outputs": _divisor(cfg.motor_outputs_hz),
            "rc": _divisor(cfg.rc_hz),
            "status": _divisor(cfg.status_hz),
            "gps": _divisor(cfg.gps_hz),
            "battery": _divisor(cfg.battery_hz),
            "gimbal": _divisor(cfg.gimbal_hz),
            "statustext": _divisor(cfg.statustext_hz),
            "robot_state": _divisor(cfg.robot_state_hz),
        }
        publishers = {
            "odom": self._publish_odometry,
            "imu": self._publish_imu,
            "motor_outputs": self._publish_motor_outputs,
            "rc": self._publish_rc,
            "status": self._publish_status,
            "gps": self._publish_gps,
            "battery": self._publish_battery,
            "gimbal": self._publish_gimbal,
            "statustext": self._publish_statustext,
            "robot_state": self._publish_robot_state,
        }
        tick = 0
        next_tick = time.perf_counter()
        while not self._stop_event.is_set():
            for name, div in divisors.items():
                if div and tick % div == 0:
                    try:
                        publishers[name](state)
                    except Exception:
                        logger.exception("publish failed", stream=name)
            tick += 1
            next_tick += period
            sleep_for = next_tick - time.perf_counter()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                next_tick = time.perf_counter()

    # Publishers (publish thread)

    def _stamp(self, boot_s: float | None, rx_t: float) -> float:
        if boot_s is None or self.config.timebase_source != "system_time":
            return rx_t
        if self._timebase.quality == "none":
            return rx_t
        return self._timebase.to_utc(boot_s)

    def _publish_odometry(self, state: VehicleState) -> None:
        with state.lock:
            local = state.local
            if local is None:
                return
            att = None
            if local.boot_s is not None:
                att = state.attitude.at_boot(local.boot_s)
            if att is None:
                att = state.attitude.at(local.t)
            imu = state.imu
        if att is None:
            return
        cfg = self.config
        ts = self._stamp(local.boot_s, local.t)
        x, y, z = ned_to_flu(local.n, local.e, local.d)
        vx, vy, vz = ned_to_flu(local.vn, local.ve, local.vd)
        q = quaternion_from_ned_euler(
            math.radians(att["roll"]), math.radians(att["pitch"]), math.radians(att["yaw"])
        )
        angular = Vector3(*frd_to_flu(imu.xgyro, imu.ygyro, imu.zgyro)) if imu else Vector3()
        pose = PoseStamped(
            ts=ts, frame_id=cfg.odom_frame_id, position=Vector3(x, y, z), orientation=q
        )
        self.odom.publish(pose)
        self.odometry.publish(
            Odometry(
                ts=ts,
                frame_id=cfg.odom_frame_id,
                child_frame_id=cfg.base_frame_id,
                pose=Pose(Vector3(x, y, z), q),
                twist=Twist(Vector3(vx, vy, vz), angular),
            )
        )
        # Both edges at the odom stamp so consumers never see one without the other.
        edges = [Transform.from_pose(cfg.base_frame_id, pose)]
        if cfg.publish_gimbal_mount_tf:
            edges.append(
                Transform(
                    translation=Vector3(*cfg.gimbal_mount_xyz),
                    frame_id=cfg.base_frame_id,
                    child_frame_id=cfg.gimbal_base_frame_id,
                    ts=ts,
                )
            )
        self.tf.publish(TFMessage(*edges))

    def _publish_imu(self, state: VehicleState) -> None:
        with state.lock:
            imu = state.imu
            _, att = state.attitude.latest()
        if imu is None:
            return
        orientation = (
            quaternion_from_ned_euler(
                math.radians(att["roll"]), math.radians(att["pitch"]), math.radians(att["yaw"])
            )
            if att
            else Quaternion()
        )
        self.imu.publish(
            Imu(
                angular_velocity=Vector3(*frd_to_flu(imu.xgyro, imu.ygyro, imu.zgyro)),
                linear_acceleration=Vector3(*frd_to_flu(imu.xacc, imu.yacc, imu.zacc)),
                orientation=orientation,
                frame_id=self.config.base_frame_id,
                ts=self._stamp(imu.boot_s, imu.t),
            )
        )

    def _publish_motor_outputs(self, state: VehicleState) -> None:
        with state.lock:
            servo = state.servo_outputs
        if servo is None or servo.boot_s == self._last_motor_boot:
            return
        self._last_motor_boot = servo.boot_s
        # PWM microseconds as PX4 drove them; the X500 uses outputs 1..4.
        self.motor_outputs.publish(
            JointState(
                ts=self._stamp(servo.boot_s, servo.t),
                frame_id=self.config.base_frame_id,
                name=[f"motor{i + 1}" for i in range(len(servo.pwm))],
                position=[float(p) for p in servo.pwm],
                velocity=[],
                effort=[],
            )
        )

    def _publish_rc(self, state: VehicleState) -> None:
        with state.lock:
            rc = state.rc
        if rc is None:
            return
        # Raw microseconds so the enable-switch threshold (1500 us) keeps its meaning.
        self.rc.publish(
            Joy(ts=rc.t, frame_id="rc", axes=[float(c) for c in rc.chan[: rc.count]], buttons=[])
        )

    def _publish_gps(self, state: VehicleState) -> None:
        with state.lock:
            gps, gp, home, local = state.gps, state.global_pos, state.home, state.local
        if gps is None:
            return
        cov_known = not (math.isnan(gps.eph) or math.isnan(gps.epv))
        eph2, epv2 = (gps.eph**2, gps.epv**2) if cov_known else (0.0, 0.0)
        self.gps.publish(
            NavSatFix(
                latitude=gp.lat if gp else 0.0,
                longitude=gp.lon if gp else 0.0,
                altitude=gp.alt_msl if gp else 0.0,
                status=_navsat_status(gps.fix),
                position_covariance=[eph2, 0.0, 0.0, 0.0, eph2, 0.0, 0.0, 0.0, epv2],
                position_covariance_type=(
                    COVARIANCE_TYPE_DIAGONAL_KNOWN if cov_known else COVARIANCE_TYPE_UNKNOWN
                ),
                frame_id="gps",
                ts=gps.t,
            )
        )
        if gp is None:
            return
        gp_n = gp_e = 0.0
        if local is not None and home is not None:
            gp_n, gp_e = local.n - home.n, local.e - home.e
        x, y, _ = ned_to_flu(gp_n, gp_e, 0.0)
        yaw = ned_yaw_to_flu_yaw(math.radians(gp.hdg_deg)) if not math.isnan(gp.hdg_deg) else 0.0
        self.global_pose.publish(
            PoseStamped(
                ts=self._stamp(gp.boot_s, gp.t),
                frame_id="home",
                position=Vector3(x, y, gp.rel_alt),
                orientation=Quaternion.from_euler(Vector3(0.0, 0.0, yaw)),
            )
        )

    def _publish_battery(self, state: VehicleState) -> None:
        with state.lock:
            ss = state.sys_status
        if ss is None:
            return
        self.battery.publish(
            BatteryState(
                voltage=ss.volt,
                current=ss.current_a,
                percentage=ss.batt_pct / 100.0 if ss.batt_pct >= 0 else math.nan,
                power_supply_status=POWER_SUPPLY_STATUS_DISCHARGING,
                power_supply_technology=POWER_SUPPLY_TECHNOLOGY_LIPO,
                present=True,
                frame_id="battery",
                ts=ss.t,
            )
        )

    def _publish_gimbal(self, state: VehicleState) -> None:
        with state.lock:
            t, g = state.gimbal.latest()
            boot = state.gimbal.latest_boot()
            flags, failure = state.gimbal_flags, state.gimbal_failure
        if t is None or g is None or time.time() - t > _GIMBAL_MAX_AGE_S:
            return
        self.gimbal_attitude.publish(
            JointState(
                ts=self._stamp(boot, t),
                frame_id=self.config.gimbal_base_frame_id,
                name=list(_GIMBAL_JOINTS),
                position=[
                    math.radians(g["roll"]),
                    math.radians(g["pitch"]),
                    math.radians(g["yaw"]),
                ],
                velocity=[],
                effort=[float(flags), float(failure), 0.0],
            )
        )

    def _publish_statustext(self, state: VehicleState) -> None:
        with state.lock:
            pending = [s for s in state.statustext if s.seq > self._last_statustext_seq]
        for st in pending:
            self.statustext.publish(String(f"[{st.severity_name}] {st.text}"))
            self._last_statustext_seq = st.seq

    def _publish_status(self, state: VehicleState) -> None:
        core, io = self._core, self._io
        if core is None or io is None:
            return
        now = time.time()
        snap = state.snapshot(now)
        with self._core_lock:
            streaming = core.sp is not None
            estop = core.estop_latched
            core_state = core.state
        writer = io.writer_id if streaming else ""
        with state.lock:
            home, ss = state.home, state.sys_status
        hb = snap.heartbeat
        jitter = _percentiles(list(self._jitter_ms))
        self.vehicle_status.publish(
            VehicleStatus(
                armed=snap.armed,
                main_mode=hb.main if hb else 0,
                sub_mode=hb.sub if hb else 0,
                mode=mode_name(hb.main if hb else None, hb.sub if hb else 0),
                landed_state=snap.landed_state if snap.landed_state is not None else -1,
                battery_pct=ss.batt_pct if ss else -1,
                voltage=ss.volt if ss else math.nan,
                gps_fix=snap.gps.fix if snap.gps else -1,
                gps_sats=snap.gps.sats if snap.gps else -1,
                gps_eph=snap.gps.eph if snap.gps else math.nan,
                rc_age_s=_nan_if_inf(snap.rc_age),
                heartbeat_age_s=_nan_if_inf(snap.heartbeat_age),
                home_valid=home is not None,
                home_lat=home.lat if home else 0.0,
                home_lon=home.lon if home else 0.0,
                home_alt=home.alt if home else 0.0,
                timebase_quality=self._timebase.quality,
                timebase_offset_s=(
                    self._timebase.offset_s if self._timebase.quality != "none" else 0.0
                ),
                writer=writer,
                state=core_state,
                estop_latched=estop,
                tick_jitter_p99_ms=jitter["p99"] if jitter["p99"] is not None else math.nan,
                frame_id=self.config.base_frame_id,
                ts=now,
            )
        )

    def _publish_robot_state(self, state: VehicleState) -> None:
        core = self._core
        if core is None:
            return
        snap = state.snapshot()
        hb = snap.heartbeat
        payload = {
            "state": core.state,
            "guidance_mode": core.guidance_mode,
            "armed": snap.armed,
            "mode": mode_name(hb.main if hb else None, hb.sub if hb else 0),
            "estop_latched": core.estop_latched,
            "battery_pct": snap.batt_pct,
            "landed_state": snap.landed_state,
        }
        self.robot_state.publish(json.dumps(payload).encode())

    # RPC surface. Note what is absent: arm, set_mode, send_*_setpoint, offboard_gate_*.

    def _result(self, why: Rejection | None) -> dict[str, Any]:
        core = self._core
        return {
            "accepted": why is None,
            "rejection": None if why is None else why.value,
            "state": core.state if core else None,
        }

    def _snap(self) -> VehicleSnapshot:
        assert self._state is not None
        return self._state.snapshot()

    def _command(
        self,
        name: str,
        argument: str,
        run: Callable[[SupervisorCore, MavlinkIO], Rejection | None],
    ) -> dict[str, Any]:
        """One operator command: run it on the core under the lock, publish its event, answer."""
        core, io = self._core, self._io
        assert core is not None and io is not None
        t0 = time.time()
        with self._core_lock:
            before = core.state
            why = run(core, io)
        self._emit_event(name, argument, "rpc", t0, before, why)
        return self._result(why)

    def _estop(self, source: str, land: bool = False) -> dict[str, Any]:
        core, io = self._core, self._io
        assert core is not None and io is not None
        t0 = time.time()
        with self._core_lock:
            before = core.state
            (core.estop_land if land else core.estop)(self._snap(), io)
        self.stop_movement.publish(Bool(data=True))
        self._emit_event("estop_land" if land else "estop", "", source, t0, before, None)
        logger.warning("E-STOP latched", source=source, land=land)
        return self._result(None)

    @rpc
    def takeoff(self) -> dict[str, Any]:
        """Run preflight and, if it passes, stream, enter OFFBOARD, arm and climb to hover."""
        return self._command("takeoff", "", lambda core, io: core.takeoff_cmd(self._snap()))

    @rpc
    def land(self) -> dict[str, Any]:
        """AUTO.LAND. Refused unless PX4 is in OFFBOARD (the pilot owns it otherwise)."""
        return self._command("land", "", lambda core, io: core.land_cmd(self._snap(), io))

    @rpc
    def hold(self) -> dict[str, Any]:
        """Stop streaming and put PX4 in Hold if we are in OFFBOARD; back to IDLE."""
        return self._command("hold", "", lambda core, io: core.hold_cmd(self._snap(), io))

    @rpc
    def set_guidance_mode(self, mode: str) -> dict[str, Any]:
        """Select HOVER, YAW_TRACK, FOLLOW or TELEOP; applied after the hover settles."""
        m = mode.upper()
        if m not in GUIDANCE_MODES:
            raise ValueError(f"unknown guidance mode {mode!r}; one of {GUIDANCE_MODES}")
        guidance: GuidanceMode = m  # type: ignore[assignment]
        return self._command(
            "set_guidance_mode", m, lambda core, io: core.set_guidance_mode(guidance)
        )

    @rpc
    def estop(self) -> dict[str, Any]:
        """Hold plus latch. Synchronous. Nothing moves the aircraft again until estop_clear."""
        return self._estop("rpc")

    @rpc
    def estop_land(self) -> dict[str, Any]:
        """AUTO.LAND plus latch."""
        return self._estop("rpc", land=True)

    @rpc
    def estop_clear(self) -> dict[str, Any]:
        """Release the latch. Only works in IDLE."""
        return self._command("estop_clear", "", lambda core, io: core.estop_clear())

    @rpc
    def sitl_enable(self, value: bool) -> dict[str, Any]:
        """SITL only: fake the RC enable switch."""
        return self._command("sitl_enable", str(value), lambda core, io: core.sitl_enable(value))

    @rpc
    def status(self) -> dict[str, Any]:
        """The supervisor's status dict plus the setpoint writer and the tick jitter."""
        core, io = self._core, self._io
        assert core is not None and io is not None
        snap = self._snap()
        with self._core_lock:
            out = core.status(snap)
        out["writer"] = io.writer_id if out["setpoint"] is not None else None
        out["tick_jitter_ms"] = _percentiles(list(self._jitter_ms))
        return out

    @rpc
    def snapshot(self) -> dict[str, Any]:
        """The current :class:`VehicleSnapshot` as a dict."""
        return dataclasses.asdict(self._snap())

    @rpc
    def sensor_stats(self) -> dict[str, Any]:
        """Per-message counters and ages, ack results, timebase quality, cmd_vel verdicts."""
        io = self._io
        out: dict[str, Any] = {} if io is None else io.stats()
        out["cmd_vel"] = {
            "accepted": self._cmd_vel_accepted,
            "rejected": dict(self._cmd_vel_rejections),
        }
        out["gimbal_target"] = {
            "sent": self._gimbal_target_sent,
            "dropped": self._gimbal_target_dropped,
        }
        out["tick_jitter_ms"] = _percentiles(list(self._jitter_ms))
        return out


def _percentiles(samples: list[float]) -> dict[str, float | None]:
    if not samples:
        return {"p50": None, "p99": None, "max": None}
    ordered = sorted(samples)
    return {
        "p50": statistics.median(ordered),
        "p99": ordered[min(len(ordered) - 1, int(0.99 * len(ordered)))],
        "max": ordered[-1],
    }
