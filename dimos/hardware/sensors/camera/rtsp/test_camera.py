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

"""RtspCamera against a synthetic H.265 clip. No camera, no network."""

from __future__ import annotations

from collections.abc import Iterator
from pathlib import Path
import time
from typing import Any

import numpy as np
import pytest

from dimos.hardware.sensors.camera.rtsp.camera import RtspCamera, gst_nv_pipeline
from dimos.hardware.sensors.camera.rtsp.synthetic import SQUARE, square_origin, write_synthetic_h265
from dimos.msgs.link_msgs.LinkPolicy import LinkPolicy

pytestmark = pytest.mark.skipif_no_turbojpeg


@pytest.fixture(scope="module")
def clip(tmp_path_factory: pytest.TempPathFactory) -> tuple[Path, int]:
    path = tmp_path_factory.mktemp("rtsp") / "synthetic.mp4"
    n = write_synthetic_h265(path, width=320, height=180, fps=25, seconds=2.0)
    return path, n


@pytest.fixture
def camera(clip: tuple[Path, int]) -> Iterator[tuple[RtspCamera, dict[str, list[Any]]]]:
    cam = RtspCamera(
        url=str(clip[0]),
        replay_realtime=False,
        replay_loop=False,
        color_hz=1e9,
        jpeg_hz=1e9,
        capture_latency_s=0.08,
        sensor_stats_interval_s=0.0,
    )
    published: dict[str, list[Any]] = {"video": [], "color_image": [], "color_jpeg": []}
    for name, sink in published.items():
        getattr(cam, name).publish = sink.append
    yield cam, published
    cam.stop()


def test_every_access_unit_is_passed_through(
    camera: tuple[RtspCamera, dict[str, list[Any]]], clip: tuple[Path, int]
) -> None:
    cam, published = camera
    t0 = time.time()
    packets = cam.relay_once()
    assert packets == clip[1]
    assert len(published["video"]) == clip[1]
    first = published["video"][0]
    assert first.format == "h265" and first.frame_id == "a8_optical"
    assert first.data.size > 0
    # Stamped at the read minus the configured capture latency, on the wall clock.
    assert t0 - 0.08 - 0.5 <= first.ts <= time.time() - 0.08
    stamps = [v.ts for v in published["video"]]
    assert stamps == sorted(stamps)


def test_decoded_frames_carry_the_moving_square(
    camera: tuple[RtspCamera, dict[str, list[Any]]], clip: tuple[Path, int]
) -> None:
    cam, published = camera
    cam.relay_once()
    frames = published["color_image"]
    assert len(frames) == clip[1]
    img = frames[10]
    assert img.data.shape == (180, 320, 3)
    x, y = square_origin(10, 320, 180)
    assert np.mean(img.data[y : y + SQUARE, x : x + SQUARE]) > 200
    assert np.mean(img.data[:20, :20]) < 130
    assert img.ts == published["video"][10].ts


def test_jpeg_is_small_and_capped(clip: tuple[Path, int]) -> None:
    cam = RtspCamera(
        url=str(clip[0]), replay_realtime=False, replay_loop=False, color_hz=0.0, jpeg_hz=1e9
    )
    jpegs: list[Any] = []
    cam.video.publish = lambda _m: None
    cam.color_image.publish = lambda _m: None
    cam.color_jpeg.publish = jpegs.append
    try:
        cam.relay_once()
    finally:
        cam.stop()
    assert len(jpegs) == clip[1]
    assert jpegs[0].format == "jpeg" and 0 < len(jpegs[0].data) < 20_000
    assert cam.sensor_stats()["color_image"]["published"] == 0


def test_link_policy_stops_video_and_paces_jpeg(
    camera: tuple[RtspCamera, dict[str, list[Any]]], clip: tuple[Path, int]
) -> None:
    cam, published = camera
    cam._on_link_policy(LinkPolicy(video_allowed=False, jpeg_hz=0.5))
    cam.relay_once()
    assert published["video"] == []
    assert cam.sensor_stats()["video"]["dropped"] >= clip[1]
    assert len(published["color_jpeg"]) == 1  # 0.5 Hz over a clip replayed as fast as possible


def test_gst_pipeline_uses_nvdec_and_fixed_size_frames() -> None:
    cmd = gst_nv_pipeline("rtsp://192.168.144.25:8554/main.264", 50, 1280, 720, 10.0)
    assert cmd[0] == "gst-launch-1.0"
    assert "nvv4l2decoder" in cmd and "fdsink" in cmd
    assert "video/x-raw,format=BGRx,width=1280,height=720,framerate=10/1" in cmd
