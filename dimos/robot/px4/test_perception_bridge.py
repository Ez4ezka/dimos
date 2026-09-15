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

"""PerceptionBridge on synthetic frames with the blob detector. No camera, no GPU, no transports."""

from __future__ import annotations

from collections.abc import Iterator
import math
from typing import Any

import numpy as np
import pytest

from dimos.hardware.gimbal.siyi.replay import AttitudeSample
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.px4_msgs.VehicleStatus import VehicleStatus
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.robot.px4.perception.tracker import MIN_HITS
from dimos.robot.px4.perception_bridge import PerceptionBridge

T0 = 1_700_000_000.0
W, H = 320, 180
VEHICLE_YAW_DEG = 10.0  # NED heading
GIMBAL_YAW_DEG = 30.0
GIMBAL_PITCH_DEG = -20.0


@pytest.fixture
def bridge() -> Iterator[tuple[PerceptionBridge, dict[str, list[Any]]]]:
    b = PerceptionBridge(detector="blob", legacy_udp_fanout=False)
    b._detector = b.make_detector()
    published: dict[str, list[Any]] = {
        "tracks": [],
        "target_state": [],
        "target_valid": [],
        "target_los": [],
    }
    for name, sink in published.items():
        getattr(b, name).publish = sink.append
    # Vehicle 10 m up, heading 10 deg; gimbal 30 deg right, 20 deg down.
    yaw_flu = -math.radians(VEHICLE_YAW_DEG)
    b._on_odometry(
        Odometry(
            ts=T0,
            frame_id="odom",
            child_frame_id="base_link",
            pose=Pose(Vector3(0.0, 0.0, 10.0), Quaternion.from_euler(Vector3(0.0, 0.0, yaw_flu))),
            twist=Twist(),
        )
    )
    b._on_gimbal(AttitudeSample(T0, 0.0, GIMBAL_PITCH_DEG, GIMBAL_YAW_DEG).joint_state())
    b._on_global_pose(PoseStamped(ts=T0, frame_id="home", position=Vector3(0.0, 0.0, 10.0)))
    b._on_vehicle_status(VehicleStatus(armed=True, ts=T0))
    yield b, published
    b.stop()


def _frame(i: int, cx: int = W // 2, cy: int = H // 2) -> Image:
    data = np.full((H, W, 3), 96, dtype=np.uint8)
    data[cy - 12 : cy + 12, cx - 12 : cx + 12] = 255
    return Image(data=data, format=ImageFormat.RGB, frame_id="a8_optical", ts=T0 + i / 25)


def test_publishes_only_the_three_contract_streams_plus_tracks(
    bridge: tuple[PerceptionBridge, dict[str, list[Any]]],
) -> None:
    b, _ = bridge
    assert set(b.outputs) == {"tracks", "target_state", "target_valid", "target_los"}
    assert {
        "color_image",
        "odometry",
        "gimbal_attitude",
        "global_pose",
        "vehicle_status",
        "track_select",
    } == set(b.inputs)


def test_track_confirms_then_selection_yields_a_valid_target(
    bridge: tuple[PerceptionBridge, dict[str, list[Any]]],
) -> None:
    b, published = bridge
    for i in range(MIN_HITS - 1):
        b.process_frame(_frame(i), now=T0 + i / 25 + 0.01)
    assert published["tracks"][-1].detections_length == 0
    b.process_frame(_frame(MIN_HITS - 1), now=T0 + (MIN_HITS - 1) / 25 + 0.01)
    assert published["tracks"][-1].detections_length == 1
    assert published["target_valid"][-1].data is False  # nothing selected yet
    assert b.status()["los"]["reason"] == "no selection"

    b.select_track(1)
    state = b.process_frame(_frame(MIN_HITS), now=T0 + MIN_HITS / 25 + 0.01)
    assert state.valid, state.reason
    assert published["target_valid"][-1].data is True
    los = published["target_los"][-1]
    # Square at the image centre: azimuth is heading + gimbal yaw; the LOS pose carries the
    # gimbal body yaw as an FLU (counter-clockwise) yaw.
    assert b.status()["los"]["azimuth_deg"] == pytest.approx(
        VEHICLE_YAW_DEG + GIMBAL_YAW_DEG, abs=0.05
    )
    assert math.degrees(los.yaw) == pytest.approx(-GIMBAL_YAW_DEG, abs=0.05)
    target = published["target_state"][-1]
    horizontal = 8.9 / math.tan(math.radians(-GIMBAL_PITCH_DEG))  # 10 m up, aim 1.0, ref 0.1
    az = math.radians(VEHICLE_YAW_DEG + GIMBAL_YAW_DEG)
    assert target.x == pytest.approx(horizontal * math.cos(az), abs=0.05)
    assert target.y == pytest.approx(-horizontal * math.sin(az), abs=0.05)  # east is -y in FLU
    assert target.child_frame_id == "target"
    assert published["tracks"][-1].detections[0].id == "1*"  # selected first, starred


def test_click_selects_the_track_under_the_pixel_and_nan_clears(
    bridge: tuple[PerceptionBridge, dict[str, list[Any]]],
) -> None:
    b, _ = bridge
    for i in range(MIN_HITS):
        b.process_frame(_frame(i), now=T0 + i / 25 + 0.01)
    b._on_track_select(PointStamped(x=W / 2, y=H / 2, ts=T0, frame_id="a8_image"))
    assert b.status()["selected_track_id"] == 1
    b._on_track_select(
        PointStamped(x=5.0, y=5.0, ts=T0, frame_id="a8_image")
    )  # far away: unchanged
    assert b.status()["selected_track_id"] == 1
    b._on_track_select(PointStamped(x=math.nan, y=math.nan, ts=T0, frame_id="a8_image"))
    assert b.status()["selected_track_id"] is None


def test_selection_persists_while_the_target_is_lost(
    bridge: tuple[PerceptionBridge, dict[str, list[Any]]],
) -> None:
    b, published = bridge
    for i in range(MIN_HITS + 1):
        b.process_frame(_frame(i), now=T0 + i / 25 + 0.01)
    b.select_track(1)
    b.process_frame(_frame(MIN_HITS + 1), now=T0 + (MIN_HITS + 1) / 25 + 0.01)
    assert published["target_valid"][-1].data is True
    empty = Image(data=np.full((H, W, 3), 96, dtype=np.uint8), format=ImageFormat.RGB, ts=T0 + 0.3)
    state = b.process_frame(empty, now=T0 + 0.31)
    assert b.status()["selected_track_id"] == 1
    assert b.status()["los"]["reason"] == "only predicted track"
    assert state.valid  # the Kalman filter coasts within max_meas_age_s
