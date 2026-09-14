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

"""PX4 drone blueprints: the shared transport map, viewer layout, and the SITL stacks.

Mirrors ``r1pro_coordinator.py``. Every derived blueprint calls ``px4_control()``
afresh: ``.transports()`` is applied by value and does not propagate into blueprints
built from an earlier value.

Usage:
    dimos run px4-sitl              # against `make px4_sitl gz_x500`
    dimos run px4-sitl-follow       # + a scripted target for FOLLOW / YAW_TRACK
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
from dimos.msgs.sensor_msgs.BatteryState import BatteryState
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.Joy import Joy
from dimos.msgs.sensor_msgs.NavSatFix import NavSatFix
from dimos.msgs.std_msgs.String import String
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.protocol.pubsub.impl.zenohpubsub import QOS_LATEST_WINS, Topic as ZenohTopic, Zenoh
from dimos.robot.px4.px4_drone import Px4Drone
from dimos.robot.px4.sitl.fake_target import FakeTarget
from dimos.visualization.rerun.bridge import RerunBridgeModule
from dimos.visualization.rerun.websocket_server import RerunWebSocketServer

# PX4 SITL (`make px4_sitl gz_x500`) sends its onboard-computer MAVLink stream here.
_SITL_MAV_URL = "udpin:0.0.0.0:14540"


def _px4_rerun_blueprint() -> Any:
    """3D world plus the supervisor/vehicle status text."""
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
            ),
            column_shares=[2, 1],
        ),
        rrb.TimePanel(state="hidden"),
        rrb.SelectionPanel(state="hidden"),
    )


# Per-entity rate caps for the rerun bridge (visualization only). Keys are rerun entity
# paths. `world/video` must never be capped: dropping H.265 access units breaks decode.
_RERUN_MAX_HZ = {
    "world/color_jpeg": 2.0,
    "world/odometry": 10.0,
    "world/odom": 10.0,
    "world/tf": 10.0,
    "world/imu": 5.0,
    "world/tracks": 10.0,
    "world/gimbal_state": 10.0,
}

# Per-topic overrides for the rerun bridge (None = suppress entirely).
_RERUN_VISUAL_OVERRIDE = {
    "world/color_image": None,
    "world/vehicle_status": None,
    "world/target_status": None,
}

rerun_config = {
    "blueprint": _px4_rerun_blueprint,
    "pubsubs": [Zenoh()],
    "rerun_open": global_config.rerun_open,
    "rerun_web": global_config.rerun_web,
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


def px4_control(**px4: Any) -> Blueprint:
    """Px4Drone with the shared transport map. ``px4`` kwargs are Px4DroneConfig fields."""
    return (
        autoconnect(Px4Drone.blueprint(**px4))
        .transports(
            {
                # Public Twist bus: MovementManager / viewer teleop drives cmd_vel.
                ("cmd_vel", Twist): _zenoh_transport("/cmd_vel", Twist),
                ("tele_cmd_vel", Twist): _zenoh_transport("/tele_cmd_vel", Twist),
                ("target_state", Odometry): _zenoh_transport("/target_state", Odometry),
                ("target_valid", Bool): _zenoh_transport("/target_valid", Bool),
                ("target_los", PoseStamped): _zenoh_transport("/target_los", PoseStamped),
                ("estop_in", Bool): _zenoh_transport("/estop_in", Bool),
                ("odometry", Odometry): _zenoh_transport("/odometry", Odometry, latest_wins=True),
                ("odom", PoseStamped): _zenoh_transport("/odom", PoseStamped, latest_wins=True),
                ("tf", TFMessage): _zenoh_transport("/tf", TFMessage, latest_wins=True),
                ("imu", Imu): _zenoh_transport("/imu", Imu, latest_wins=True),
                ("gps", NavSatFix): _zenoh_transport("/gps", NavSatFix),
                ("battery", BatteryState): _zenoh_transport("/battery", BatteryState),
                ("rc", Joy): _zenoh_transport("/rc", Joy),
                ("vehicle_status", String): _zenoh_transport("/vehicle_status", String),
                ("gimbal_attitude", JointState): _zenoh_transport(
                    "/gimbal_attitude", JointState, latest_wins=True
                ),
                ("global_pose", PoseStamped): _zenoh_transport("/global_pose", PoseStamped),
                ("supervisor_status", String): _zenoh_transport("/supervisor_status", String),
                ("supervisor_state", String): _zenoh_transport("/supervisor_state", String),
                ("offboard_setpoint", Odometry): _zenoh_transport("/offboard_setpoint", Odometry),
                # robot_state (bytes) is left on the default transport; the hosted
                # blueprint binds it to the operator link.
                ("stop_movement", Bool): _zenoh_transport("/stop_movement", Bool),
            }
        )
        .global_config(transport="zenoh")
    )


px4_sitl = autoconnect(
    px4_visualization(),
    px4_control(mav_url=_SITL_MAV_URL, sitl=True),
).global_config(n_workers=2)


px4_sitl_follow = autoconnect(
    px4_visualization(),
    px4_control(mav_url=_SITL_MAV_URL, sitl=True),
    FakeTarget.blueprint(),
).global_config(n_workers=2)
