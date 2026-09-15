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

"""PX4 SITL with the real perception chain on a replayed clip instead of FakeTarget.

RtspCamera replays a capture (``--rtspcamera.url=...``), PerceptionBridge runs the
detector, tracker, line of sight and target estimator on its frames, FakeA8 stands in for
the gimbal, and the connection consumes ``target_state``, ``target_valid`` and
``target_los`` exactly as it does from FakeTarget. ``tool_perception_gate.py`` runs this
with the blob detector on the synthetic clip.

Usage:
    dimos run px4-sitl-perception --rtspcamera.url=clip.mp4 --perceptionbridge.model=yolov8n.pt
"""

from __future__ import annotations

from dimos.core.coordination.blueprints import autoconnect
from dimos.hardware.sensors.camera.rtsp.camera import RtspCamera
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.vision_msgs.Detection2DArray import Detection2DArray
from dimos.robot.px4.blueprints.basic.px4_basic import (
    px4_control,
    px4_visualization,
    zenoh_transport,
)
from dimos.robot.px4.blueprints.bench.px4_bench import bench_transports
from dimos.robot.px4.config import SITL_MAV_URL
from dimos.robot.px4.perception_bridge import PerceptionBridge
from dimos.robot.px4.sitl.fake_a8 import FakeA8

# Same worker split as px4_bench; the bridge and the camera are dedicated workers.
px4_sitl_perception = (
    autoconnect(
        px4_visualization(),
        px4_control(mav_url=SITL_MAV_URL, sitl=True),
        RtspCamera.blueprint(),
        PerceptionBridge.blueprint(legacy_udp_fanout=False),
        FakeA8.blueprint(initial_pitch_deg=-20.0, initial_yaw_deg=30.0),
    )
    .transports(
        {
            **bench_transports(),
            ("tracks", Detection2DArray): zenoh_transport(
                "/tracks", Detection2DArray, latest_wins=True
            ),
            ("track_select", PointStamped): zenoh_transport("/track_select", PointStamped),
        }
    )
    .global_config(n_workers=2)
)
