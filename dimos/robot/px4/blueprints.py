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

"""The three PX4 blueprints: the connection alone, the aircraft, and its simulator twin.

``px4-basic``
    Px4DroneConnection plus the viewer. On the Jetson against mavlink-router endpoint 14556.
``px4-drone``
    Everything on the aircraft: the connection, the command tracker, the A8 camera and
    gimbal, the link monitor and the perception bridge.
``px4-sitl``
    The same modules against PX4 SITL (``make px4_sitl gz_x500``). What the simulator lacks
    is stood in for: the camera replays a generated clip, FakeA8 answers as the gimbal, the
    link monitor replays a measured scenario, the perception bridge runs the blob detector.
    ``tool_sitl_gate.py`` runs this blueprint end to end.

Every module except the connection is optional. Each binds to the others by stream name and
type, and each is written to degrade when a peer is absent (no gimbal attitude means no
gimbal tf and no line of sight, no link monitor means the camera keeps its own defaults,
no perception means the connection never leaves HOVER for FOLLOW), so removing a module
from a blueprint removes a capability, never a startup.

Layout mirrors ``r1pro_coordinator.py``: one function per layer returning a Blueprint,
composed with ``autoconnect``. All streams of the package are declared once in
:func:`px4_transports`; a key for a port no module in the blueprint has is simply unused.
"""

from __future__ import annotations

from typing import Any

from dimos_lcm.std_msgs import Bool  # type: ignore[import-untyped]

from dimos.core.coordination.blueprints import Blueprint, TransportSpec, autoconnect
from dimos.core.global_config import global_config
from dimos.core.stream import Transport
from dimos.core.transport import ZenohTransport
from dimos.hardware.gimbal.siyi.gimbal import SiyiA8Gimbal
from dimos.hardware.sensors.camera.rtsp.camera import SYNTHETIC_URL, RtspCamera
from dimos.msgs.foxglove_msgs.CompressedVideo import CompressedVideo
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.link_msgs.LinkPolicy import LinkPolicy
from dimos.msgs.link_msgs.LinkStatus import LinkStatus
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.px4_msgs.CommandEvent import CommandEvent
from dimos.msgs.px4_msgs.TrackedCommand import TrackedCommand
from dimos.msgs.px4_msgs.VehicleStatus import VehicleStatus
from dimos.msgs.sensor_msgs.BatteryState import BatteryState
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.CompressedImage import CompressedImage
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.Joy import Joy
from dimos.msgs.sensor_msgs.NavSatFix import NavSatFix
from dimos.msgs.std_msgs.Float32 import Float32
from dimos.msgs.std_msgs.String import String
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.msgs.vision_msgs.Detection2DArray import Detection2DArray
from dimos.protocol.pubsub.impl.zenohpubsub import QOS_LATEST_WINS, Topic as ZenohTopic, Zenoh
from dimos.robot.px4.command_tracker import CommandTracker
from dimos.robot.px4.config import SITL_MAV_URL
from dimos.robot.px4.connection import Px4DroneConnection
from dimos.robot.px4.link_monitor import LinkMonitor
from dimos.robot.px4.perception.bridge import PerceptionBridge
from dimos.robot.px4.sitl import FakeA8
from dimos.visualization.rerun.bridge import RerunBridgeModule
from dimos.visualization.rerun.websocket_server import RerunWebSocketServer

# Streams


def zenoh_transport(topic: str, msg_type: type, *, latest_wins: bool = False) -> TransportSpec:
    """One zenoh topic ``dimos/<topic>``; latest-wins for streams where stale beats late."""
    return ZenohTransport.spec(
        ZenohTopic(f"dimos/{topic}", msg_type, qos=QOS_LATEST_WINS if latest_wins else None)
    )


def px4_transports() -> dict[tuple[str, type], TransportSpec | Transport[Any]]:
    """Every stream of the package, keyed by (port name, type), the way modules bind.

    The heavy vehicle streams are latest-wins: a stale odometry sample is worse than a
    dropped one. ``color_image`` is deliberately absent: 2.8 MB a frame, it stays on the
    default local transport, on-Jetson only. ``robot_state`` (bytes) stays on the default
    transport too; the hosted blueprint binds it to the operator link.
    """
    return {
        # Into the connection. cmd_vel is the public Twist bus any teleop module drives.
        ("cmd_vel", Twist): zenoh_transport("cmd_vel", Twist),
        ("gimbal_target", JointState): zenoh_transport("gimbal_target", JointState),
        ("estop_in", Bool): zenoh_transport("estop_in", Bool),
        ("target_state", Odometry): zenoh_transport("target_state", Odometry),
        ("target_valid", Bool): zenoh_transport("target_valid", Bool),
        ("target_los", PoseStamped): zenoh_transport("target_los", PoseStamped),
        # Out of the connection: the vehicle.
        ("odometry", Odometry): zenoh_transport("odometry", Odometry, latest_wins=True),
        ("odom", PoseStamped): zenoh_transport("odom", PoseStamped, latest_wins=True),
        ("tf", TFMessage): zenoh_transport("tf", TFMessage, latest_wins=True),
        ("imu", Imu): zenoh_transport("imu", Imu, latest_wins=True),
        ("motor_outputs", JointState): zenoh_transport("motor_outputs", JointState),
        ("gps", NavSatFix): zenoh_transport("gps", NavSatFix),
        ("battery", BatteryState): zenoh_transport("battery", BatteryState),
        ("rc", Joy): zenoh_transport("rc", Joy),
        ("gimbal_attitude", JointState): zenoh_transport(
            "gimbal_attitude", JointState, latest_wins=True
        ),
        ("global_pose", PoseStamped): zenoh_transport("global_pose", PoseStamped),
        ("vehicle_status", VehicleStatus): zenoh_transport("vehicle_status", VehicleStatus),
        ("statustext", String): zenoh_transport("statustext", String),
        # Out of the connection: the supervisor.
        ("supervisor_status", String): zenoh_transport("supervisor_status", String),
        ("supervisor_state", String): zenoh_transport("supervisor_state", String),
        ("command_event", CommandEvent): zenoh_transport("command_event", CommandEvent),
        ("offboard_setpoint", Odometry): zenoh_transport("offboard_setpoint", Odometry),
        ("stop_movement", Bool): zenoh_transport("stop_movement", Bool),
        # Camera and gimbal.
        ("video", CompressedVideo): zenoh_transport("video", CompressedVideo, latest_wins=True),
        ("color_jpeg", CompressedImage): zenoh_transport(
            "color_jpeg", CompressedImage, latest_wins=True
        ),
        ("camera_info", CameraInfo): zenoh_transport("camera_info", CameraInfo),
        # Link monitor.
        ("link_status", LinkStatus): zenoh_transport("link_status", LinkStatus),
        ("link_policy", LinkPolicy): zenoh_transport("link_policy", LinkPolicy),
        # Command tracker.
        ("tracked_command", TrackedCommand): zenoh_transport("tracked_command", TrackedCommand),
        ("command_report", String): zenoh_transport("command_report", String),
        ("cmd_forward", Float32): zenoh_transport("cmd_forward", Float32),
        ("meas_forward", Float32): zenoh_transport("meas_forward", Float32),
        # Perception.
        ("tracks", Detection2DArray): zenoh_transport("tracks", Detection2DArray, latest_wins=True),
        ("track_select", PointStamped): zenoh_transport("track_select", PointStamped),
    }


# Viewer


def _layout(with_video: bool) -> Any:
    """3D world beside the video, the setpoint and battery traces and the status log.

    Entity paths assume the bridge's default ``entity_prefix="world"``.
    """
    import rerun as rr
    import rerun.blueprint as rrb

    world = rrb.Spatial3DView(
        origin="world",
        name="3D",
        background=rrb.Background(kind="SolidColor", color=[0, 0, 0]),
        line_grid=rrb.LineGrid3D(plane=rr.components.Plane3D.XY.with_distance(1.0)),
    )
    left: Any = world
    if with_video:
        left = rrb.Vertical(
            world, rrb.Spatial2DView(origin="world/video", name="A8 H.265"), row_shares=[2, 1]
        )
    return rrb.Blueprint(
        rrb.Horizontal(
            left,
            rrb.Vertical(
                rrb.TimeSeriesView(origin="world/offboard_setpoint", name="Setpoint"),
                rrb.TimeSeriesView(origin="world/battery", name="Battery"),
                rrb.TextLogView(origin="world", name="Status and commands"),
            ),
            column_shares=[2, 1],
        ),
        rrb.TimePanel(state="hidden"),
        rrb.SelectionPanel(state="hidden"),
    )


def _layout_basic() -> Any:
    return _layout(with_video=False)


def _layout_full() -> Any:
    return _layout(with_video=True)


# Per-entity rate caps for the rerun bridge (visualization only; the flight loop sees full
# rate). Sized for the Jetson viewed over Wi-Fi. `world/video` must never be capped:
# dropping H.265 access units breaks in-viewer decode; the camera sets that rate.
_RERUN_MAX_HZ = {
    "world/color_jpeg": 2.0,
    "world/odometry": 10.0,
    "world/odom": 10.0,
    "world/tf": 10.0,
    "world/imu": 5.0,
    "world/motor_outputs": 5.0,
    "world/tracks": 10.0,
    "world/gimbal_attitude": 10.0,
}
# Suppressed entirely: raw decoded frames are the heaviest payload on the viewer link, and
# the flown JSON packet duplicates the typed vehicle_status.
_RERUN_VISUAL_OVERRIDE = {"world/color_image": None, "world/supervisor_status": None}


def px4_visualization(*, with_video: bool = False) -> Blueprint:
    if global_config.viewer == "none":
        return Blueprint(blueprints=())
    if global_config.viewer != "rerun":
        raise ValueError(f"Unsupported viewer: {global_config.viewer}")
    rerun_config = {
        "blueprint": _layout_full if with_video else _layout_basic,
        "pubsubs": [Zenoh()],
        "rerun_open": global_config.rerun_open,
        "rerun_web": global_config.rerun_web,
        # A live viewer needs only a small rolling buffer, and every viewer (re)connect
        # replays the whole buffer before going live.
        "memory_limit": "256MB",
        "max_hz": _RERUN_MAX_HZ,
        "visual_override": _RERUN_VISUAL_OVERRIDE,
    }
    return autoconnect(
        RerunBridgeModule.blueprint(**rerun_config),
        RerunWebSocketServer.blueprint(),
    )


# Layers


def px4_control(**connection: Any) -> Blueprint:
    """Px4DroneConnection; kwargs are Px4DroneConnectionConfig fields (``mav_url``, ``sitl``...)."""
    return autoconnect(Px4DroneConnection.blueprint(**connection))


# n_workers keeps the viewer encode out of the interpreter that runs the tracker, gimbal and
# link monitor; the connection, camera and perception bridge are dedicated workers, so the
# 20 Hz flight loop never shares a GIL with anything. (The registry scanner wants each
# blueprint as one top-level autoconnect chain, hence the repetition.)
px4_basic = (
    autoconnect(px4_visualization(), px4_control())
    .transports(px4_transports())
    .global_config(transport="zenoh", n_workers=2)
)

# The gimbal module owns the whole gimbal tf chain, so the connection's own copy of the
# mount edge is switched off. Aim requests stay off until the connection is configured to
# own the A8 (``gimbal_commands_enabled``); the flown controller on component 191 keeps it.
px4_drone = (
    autoconnect(
        px4_visualization(with_video=True),
        px4_control(publish_gimbal_mount_tf=False),
        CommandTracker.blueprint(),
        RtspCamera.blueprint(),
        SiyiA8Gimbal.blueprint(),
        LinkMonitor.blueprint(),
        PerceptionBridge.blueprint(),
    )
    .transports(px4_transports())
    .global_config(transport="zenoh", n_workers=2)
)

# The aircraft on the ground gives no altitude, so the estimator uses a fixed 10 m AGL the
# way the flown bench runs did; the fake A8 starts 20 deg down and 30 deg right so the
# synthetic target (parked at the image centre) has a non-trivial line of sight.
px4_sitl = (
    autoconnect(
        px4_visualization(with_video=True),
        px4_control(mav_url=SITL_MAV_URL, sitl=True, publish_gimbal_mount_tf=False),
        CommandTracker.blueprint(),
        RtspCamera.blueprint(url=SYNTHETIC_URL, color_hz=25.0),
        SiyiA8Gimbal.blueprint(aim_enabled=True),
        FakeA8.blueprint(initial_pitch_deg=-20.0, initial_yaw_deg=30.0),
        LinkMonitor.blueprint(source="replay", replay_scenario="home_5g"),
        PerceptionBridge.blueprint(
            detector="blob", estimator={"agl_source": "fixed", "fixed_agl_m": 10.0}
        ),
    )
    .transports(px4_transports())
    .global_config(transport="zenoh", n_workers=2)
)
