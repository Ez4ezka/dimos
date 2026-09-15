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

"""PX4 drone connection plus viewer: the base every other px4 blueprint derives from.

Mirrors ``r1pro_coordinator.py``: one function per layer returning a Blueprint,
``px4_control()`` and ``px4_visualization()``, composed into ``px4_basic``. Derived
blueprints call ``px4_control()`` afresh when they need a different connection config,
because ``.transports()`` is applied by value and does not propagate into blueprints
built from an earlier value.

Usage:
    dimos run px4-basic        # on the Jetson, against mavlink-router endpoint 14556
"""

from __future__ import annotations

from typing import Any

from dimos_lcm.std_msgs import Bool  # type: ignore[import-untyped]

from dimos.core.coordination.blueprints import Blueprint, TransportSpec, autoconnect
from dimos.core.global_config import global_config
from dimos.core.transport import ZenohTransport
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.px4_msgs.CommandEvent import CommandEvent
from dimos.msgs.px4_msgs.VehicleStatus import VehicleStatus
from dimos.msgs.sensor_msgs.BatteryState import BatteryState
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.Joy import Joy
from dimos.msgs.sensor_msgs.NavSatFix import NavSatFix
from dimos.msgs.std_msgs.String import String
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.protocol.pubsub.impl.zenohpubsub import QOS_LATEST_WINS, Topic as ZenohTopic, Zenoh
from dimos.robot.px4.connection import Px4DroneConnection
from dimos.visualization.rerun.bridge import RerunBridgeModule
from dimos.visualization.rerun.websocket_server import RerunWebSocketServer


def _px4_rerun_blueprint() -> Any:
    """3D world beside the setpoint and battery traces and the status log.

    Entity paths assume the bridge's default ``entity_prefix="world"``.
    """
    import rerun as rr
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial3DView(
                origin="world",
                name="3D",
                background=rrb.Background(kind="SolidColor", color=[0, 0, 0]),
                line_grid=rrb.LineGrid3D(
                    plane=rr.components.Plane3D.XY.with_distance(1.0),
                ),
            ),
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


# Per-entity rate caps for the rerun bridge (visualization only; the flight loop sees
# full rate). Sized for the Jetson viewed over Wi-Fi. Keys are rerun entity paths.
# `world/video` must never be capped: dropping H.265 access units breaks in-viewer decode;
# the camera module controls that rate at the source.
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

# Per-topic overrides for the rerun bridge (None = suppress entirely).
_RERUN_VISUAL_OVERRIDE = {
    # Raw decoded frames are the heaviest payload on the viewer link.
    "world/color_image": None,
    # The flown JSON packet; vehicle_status carries the same as a typed message.
    "world/supervisor_status": None,
    "world/target_status": None,
}

rerun_config = {
    "blueprint": _px4_rerun_blueprint,
    "pubsubs": [Zenoh()],
    "rerun_open": global_config.rerun_open,
    "rerun_web": global_config.rerun_web,
    # A live viewer needs only a small rolling buffer, and every viewer (re)connect
    # replays the whole buffer before going live.
    "memory_limit": "256MB",
    "max_hz": _RERUN_MAX_HZ,
    "visual_override": _RERUN_VISUAL_OVERRIDE,
}


def px4_visualization() -> Blueprint:
    if global_config.viewer == "rerun":
        return autoconnect(
            RerunBridgeModule.blueprint(**rerun_config),
            RerunWebSocketServer.blueprint(),
        )
    if global_config.viewer == "none":
        return Blueprint(blueprints=())
    raise ValueError(f"Unsupported viewer: {global_config.viewer}")


def _zenoh_transport(
    topic: str,
    msg_type: type,
    *,
    latest_wins: bool = False,
) -> TransportSpec:
    return ZenohTransport.spec(
        ZenohTopic(
            f"dimos/{topic.lstrip('/')}",
            msg_type,
            qos=QOS_LATEST_WINS if latest_wins else None,
        )
    )


def px4_control(**connection: Any) -> Blueprint:
    """Px4DroneConnection with the port-contract transport map.

    ``connection`` kwargs are Px4DroneConnectionConfig fields (``mav_url``, ``sitl``, ...).
    Topics are ``dimos/<port>``; the heavy vehicle streams are latest-wins.
    """
    return (
        autoconnect(Px4DroneConnection.blueprint(**connection))
        .transports(
            {
                # Public Twist bus: any module's cmd_vel Out (MovementManager, the viewer's
                # teleop) drives the connection's cmd_vel In.
                ("cmd_vel", Twist): _zenoh_transport("/cmd_vel", Twist),
                ("tele_cmd_vel", Twist): _zenoh_transport("/tele_cmd_vel", Twist),
                ("gimbal_target", JointState): _zenoh_transport("/gimbal_target", JointState),
                ("estop_in", Bool): _zenoh_transport("/estop_in", Bool),
                # Perception inputs, the same three FakeTarget and PerceptionBridge publish.
                ("target_state", Odometry): _zenoh_transport("/target_state", Odometry),
                ("target_valid", Bool): _zenoh_transport("/target_valid", Bool),
                ("target_los", PoseStamped): _zenoh_transport("/target_los", PoseStamped),
                # Vehicle feedback. Heavy streams are latest-wins: a stale odometry sample
                # is worse than a dropped one.
                ("odometry", Odometry): _zenoh_transport("/odometry", Odometry, latest_wins=True),
                ("odom", PoseStamped): _zenoh_transport("/odom", PoseStamped, latest_wins=True),
                ("tf", TFMessage): _zenoh_transport("/tf", TFMessage, latest_wins=True),
                ("imu", Imu): _zenoh_transport("/imu", Imu, latest_wins=True),
                ("motor_outputs", JointState): _zenoh_transport("/motor_outputs", JointState),
                ("gps", NavSatFix): _zenoh_transport("/gps", NavSatFix),
                ("battery", BatteryState): _zenoh_transport("/battery", BatteryState),
                ("rc", Joy): _zenoh_transport("/rc", Joy),
                ("gimbal_attitude", JointState): _zenoh_transport(
                    "/gimbal_attitude", JointState, latest_wins=True
                ),
                ("global_pose", PoseStamped): _zenoh_transport("/global_pose", PoseStamped),
                ("vehicle_status", VehicleStatus): _zenoh_transport(
                    "/vehicle_status", VehicleStatus
                ),
                ("statustext", String): _zenoh_transport("/statustext", String),
                # Supervisor streams.
                ("supervisor_status", String): _zenoh_transport("/supervisor_status", String),
                ("supervisor_state", String): _zenoh_transport("/supervisor_state", String),
                ("command_event", CommandEvent): _zenoh_transport("/command_event", CommandEvent),
                ("offboard_setpoint", Odometry): _zenoh_transport("/offboard_setpoint", Odometry),
                ("stop_movement", Bool): _zenoh_transport("/stop_movement", Bool),
                # robot_state (bytes) stays on the default transport; the hosted blueprint
                # binds it to the operator link.
            }
        )
        .global_config(transport="zenoh")
    )


# n_workers keeps the rerun bridge's encode work out of the interpreter that runs the
# connection; the connection itself is a dedicated worker, so its 20 Hz flight loop never
# shares a GIL with anything.
px4_basic = autoconnect(
    px4_visualization(),
    px4_control(),
).global_config(n_workers=2)
