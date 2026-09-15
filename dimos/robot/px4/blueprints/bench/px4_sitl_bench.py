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

"""The bench stack against PX4 SITL: a replayed clip for the camera and a fake A8.

SITL has no gimbal and no camera, so FakeA8 answers ``gimbal_target`` with
``gimbal_attitude`` and RtspCamera replays a file. Everything else is the aircraft
blueprint. ``tool_bench_gate.py`` runs this.

Usage:
    dimos run px4-sitl-bench --rtspcamera.url=/path/to/capture.mp4
"""

from __future__ import annotations

from dimos.core.coordination.blueprints import autoconnect
from dimos.hardware.gimbal.siyi.gimbal import SiyiA8Gimbal
from dimos.hardware.sensors.camera.rtsp.camera import RtspCamera
from dimos.robot.px4.blueprints.basic.px4_basic import px4_control, px4_visualization
from dimos.robot.px4.blueprints.bench.px4_bench import bench_transports
from dimos.robot.px4.command_tracker import CommandTracker
from dimos.robot.px4.config import SITL_MAV_URL
from dimos.robot.px4.sitl.fake_a8 import FakeA8
from dimos.robot.px4.sitl.fake_target import FakeTarget

# Same worker split as px4_bench.
px4_sitl_bench = (
    autoconnect(
        px4_visualization(),
        px4_control(mav_url=SITL_MAV_URL, sitl=True, publish_gimbal_mount_tf=False),
        CommandTracker.blueprint(),
        RtspCamera.blueprint(),
        SiyiA8Gimbal.blueprint(aim_enabled=True),
        FakeA8.blueprint(),
        FakeTarget.blueprint(),
    )
    .transports(bench_transports())
    .global_config(n_workers=2)
)
