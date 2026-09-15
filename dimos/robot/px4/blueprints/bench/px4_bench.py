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

"""Props-off bench stack on the aircraft: connection, tracker, A8 video and gimbal, viewer.

Adds RtspCamera and SiyiA8Gimbal to the tracked base the way ``r1pro_nav`` adds modules
to ``r1pro_coordinator``. The gimbal module owns the whole gimbal tf chain, so the
connection's own copy of the mount edge is switched off here. Aim requests stay off:
the flown controller on component 191 keeps the A8 in phase 1.

Usage:
    dimos run px4-bench          # on the Jetson, props off, mavlink-router endpoint 14556
"""

from __future__ import annotations

from typing import Any

from dimos.core.coordination.blueprints import TransportSpec, autoconnect
from dimos.core.stream import Transport
from dimos.hardware.gimbal.siyi.gimbal import SiyiA8Gimbal
from dimos.hardware.sensors.camera.rtsp.camera import RtspCamera
from dimos.msgs.foxglove_msgs.CompressedVideo import CompressedVideo
from dimos.msgs.link_msgs.LinkPolicy import LinkPolicy
from dimos.msgs.px4_msgs.TrackedCommand import TrackedCommand
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.CompressedImage import CompressedImage
from dimos.msgs.std_msgs.Float32 import Float32
from dimos.msgs.std_msgs.String import String
from dimos.robot.px4.blueprints.basic.px4_basic import (
    px4_control,
    px4_visualization,
    zenoh_transport,
)
from dimos.robot.px4.command_tracker import CommandTracker


def bench_transports() -> dict[tuple[str, type], TransportSpec | Transport[Any]]:
    """Streams the bench modules add on top of the connection's port contract.

    ``color_image`` is deliberately absent: it stays on the default local transport, 2.8 MB
    a frame, on-Jetson only, never dialable over the link.
    """
    return {
        ("video", CompressedVideo): zenoh_transport("/video", CompressedVideo, latest_wins=True),
        ("color_jpeg", CompressedImage): zenoh_transport(
            "/color_jpeg", CompressedImage, latest_wins=True
        ),
        ("camera_info", CameraInfo): zenoh_transport("/camera_info", CameraInfo),
        ("link_policy", LinkPolicy): zenoh_transport("/link_policy", LinkPolicy),
        ("tracked_command", TrackedCommand): zenoh_transport("/tracked_command", TrackedCommand),
        ("command_report", String): zenoh_transport("/command_report", String),
        ("cmd_forward", Float32): zenoh_transport("/cmd_forward", Float32),
        ("meas_forward", Float32): zenoh_transport("/meas_forward", Float32),
    }


# n_workers: the viewer encode, the tracker and the gimbal share two interpreters; the
# connection and the camera are dedicated workers (the 20 Hz flight loop and a full core
# of H.265 decode each get a process of their own).
px4_bench = (
    autoconnect(
        px4_visualization(),
        px4_control(publish_gimbal_mount_tf=False),
        CommandTracker.blueprint(),
        RtspCamera.blueprint(),
        SiyiA8Gimbal.blueprint(),
    )
    .transports(bench_transports())
    .global_config(n_workers=2)
)
