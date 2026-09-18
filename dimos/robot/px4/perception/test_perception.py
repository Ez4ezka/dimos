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

"""The ported perception maths: camera rays, line of sight, ground intersection, tracker, KF."""

from __future__ import annotations

import math

import numpy as np
import pytest

from dimos.robot.px4.perception.detector import BrightBlobDetector
from dimos.robot.px4.perception.estimators import (
    EstimatorConfig,
    GimbalGeo,
    LosEstimator,
    TargetEstimator,
    VehicleGeo,
)
from dimos.robot.px4.perception.geometry import (
    CameraConfig,
    CameraModel,
    LOSSolver,
    intersect_ground,
    ned_to_az_el,
)
from dimos.robot.px4.perception.tracker import MIN_HITS, Detection, Tracker


def test_camera_model_from_hfov() -> None:
    cam = CameraModel(CameraConfig())
    assert cam.fx == pytest.approx(749.3, abs=0.2)  # 1280 px over 81 deg
    assert (cam.cx, cam.cy) == (640.0, 360.0)
    centre = cam.pixel_to_ray(640.0, 360.0)
    assert centre.tolist() == pytest.approx([1.0, 0.0, 0.0])
    h, v = cam.ray_to_angles(cam.pixel_to_ray(1280.0, 360.0))
    assert h == pytest.approx(40.5, abs=0.1) and v == pytest.approx(0.0)


def test_line_of_sight_adds_gimbal_yaw_to_vehicle_heading() -> None:
    solver = LOSSolver(CameraModel())
    r = solver.solve(640.0, 360.0, {"roll": 0.0, "pitch": -20.0, "yaw": 30.0}, vehicle_yaw_deg=10.0)
    assert r.azimuth_deg == pytest.approx(40.0, abs=1e-6)
    assert r.elevation_deg == pytest.approx(-20.0, abs=1e-6)
    assert (r.gimbal_yaw_body_deg, r.gimbal_yaw_ned_deg) == (30.0, 40.0)
    az, el = ned_to_az_el(np.array(r.los_ned))
    assert (az, el) == pytest.approx((40.0, -20.0), abs=1e-6)


def test_ground_intersection_range_and_rejections() -> None:
    pt, rng = intersect_ground(
        (0.0, 0.0, -10.0), (math.cos(math.radians(20)), 0.0, math.sin(math.radians(20))), 0.0
    )
    assert pt is not None and isinstance(rng, float)
    assert pt[0] == pytest.approx(10.0 / math.tan(math.radians(20)), abs=1e-6)
    assert rng == pytest.approx(10.0 / math.sin(math.radians(20)), abs=1e-6)
    assert intersect_ground((0.0, 0.0, -10.0), (1.0, 0.0, 0.01), 0.0)[0] is None  # too shallow
    assert intersect_ground((0.0, 0.0, 5.0), (0.7, 0.0, 0.7), 0.0)[0] is None  # below ground


def _det(x: float, y: float, conf: float = 0.9) -> Detection:
    return Detection(0, "person", conf, (x, y, 40.0, 80.0))


def test_tracker_confirms_after_min_hits_and_keeps_identity_through_a_miss() -> None:
    tr = Tracker()
    t = 100.0
    for i in range(MIN_HITS - 1):
        assert tr.update([_det(100 + 4 * i, 200)], t + i / 25) == []
    (track,) = tr.update([_det(100 + 4 * (MIN_HITS - 1), 200)], t + (MIN_HITS - 1) / 25)
    assert track.track_id == 1 and track.hits == MIN_HITS and not track.predicted
    (missed,) = tr.update([], t + MIN_HITS / 25)
    assert missed.track_id == 1 and missed.predicted and missed.missed_frames == 1
    (back,) = tr.update([_det(100 + 4 * (MIN_HITS + 1), 200)], t + (MIN_HITS + 1) / 25)
    assert back.track_id == 1 and not back.predicted


def test_tracker_low_confidence_updates_but_never_creates() -> None:
    tr = Tracker()
    assert tr.update([_det(100, 200, conf=0.5)], 1.0) == [] and tr.tracks == []
    for i in range(MIN_HITS):
        tr.update([_det(100 + 4 * i, 200, conf=0.9)], 1.0 + i / 25)
    (track,) = tr.update([_det(100 + 4 * MIN_HITS, 200, conf=0.4)], 1.0 + MIN_HITS / 25)
    assert track.track_id == 1 and track.hits == MIN_HITS + 1
    assert len(tr.tracks) == 1  # the low-confidence box updated, it did not spawn an ID


def test_los_estimator_rejects_predicted_and_stale_then_solves() -> None:
    cfg = EstimatorConfig()
    est = LosEstimator(LOSSolver(CameraModel()), cfg)
    tr = Tracker()
    tracks = []
    for i in range(MIN_HITS):
        tracks = tr.update(
            [Detection(0, "person", 0.9, (620.0, 320.0, 40.0, 80.0))], 100.0 + i / 25
        )
    veh = VehicleGeo(
        yaw_deg=10.0, roll_deg=0.0, pitch_deg=0.0, attitude_age_s=0.1, n=0, e=0, d=-10, rel_alt=10
    )
    gim = GimbalGeo(0.0, -20.0, 30.0, age_s=0.2)
    assert (
        est.process(tracks, None, 100.1, (1280, 720), veh, gim, 1.0, 100.2).reason == "no selection"
    )
    assert (
        est.process(tracks, 9, 100.1, (1280, 720), veh, gim, 1.0, 100.2).reason
        == "selected target not visible"
    )
    stale = GimbalGeo(0.0, -20.0, 30.0, age_s=5.0)
    assert (
        est.process(tracks, 1, 100.1, (1280, 720), veh, stale, 1.0, 100.2).reason
        == "gimbal attitude stale"
    )
    los = est.process(tracks, 1, 100.1, (1280, 720), veh, gim, 1.0, 100.2)
    assert los.valid and los.observation == "real"
    assert los.azimuth_deg == pytest.approx(40.0, abs=1e-6)
    assert los.gimbal_yaw_body_deg == 30.0


def test_target_estimator_intersects_ground_and_filters() -> None:
    cfg = EstimatorConfig()
    est = LosEstimator(LOSSolver(CameraModel()), cfg)
    target = TargetEstimator(cfg)
    tr = Tracker()
    veh = VehicleGeo(
        yaw_deg=0.0,
        roll_deg=0.0,
        pitch_deg=0.0,
        attitude_age_s=0.1,
        n=0.0,
        e=0.0,
        d=-10.0,
        rel_alt=10.0,
    )
    gim = GimbalGeo(0.0, -20.0, 30.0, age_s=0.2)
    state = None
    for i in range(6):
        t = 100.0 + i / 25
        tracks = tr.update([Detection(0, "person", 0.9, (620.0, 320.0, 40.0, 80.0))], t)
        los = est.process(tracks, 1, t, (1280, 720), veh, gim, 1.0, t + 0.01)
        state = target.process(los, veh, t + 0.01)
    assert state is not None and state.valid, state.reason
    # Camera 10 m up, person aim height 1.0, camera_below_ref 0.1: plane 8.9 m below the camera.
    horizontal = 8.9 / math.tan(math.radians(20.0))
    assert state.n == pytest.approx(horizontal * math.cos(math.radians(30)), abs=0.05)
    assert state.e == pytest.approx(horizontal * math.sin(math.radians(30)), abs=0.05)
    assert state.bearing_deg == pytest.approx(30.0, abs=0.05)
    assert state.agl_source == "relative_alt"
    # Then loss: predicted for a while, dropped after drop_after_s.
    lost = target.process(None, veh, 101.5)  # 1.3 s after the last measurement: past max_meas_age_s
    assert not lost.valid and lost.n is not None
    dropped = target.process(None, veh, 100.24 + cfg.drop_after_s + 1.0)
    assert dropped.n is None


def test_blob_detector_boxes_the_bright_square() -> None:
    img = np.full((180, 320, 3), 96, dtype=np.uint8)
    img[78:102, 148:172] = 255
    (det,) = BrightBlobDetector().detect(img)
    assert det.class_name == "person"
    assert det.bbox == (148.0, 78.0, 24.0, 24.0)
    assert BrightBlobDetector().detect(np.zeros((10, 10, 3), dtype=np.uint8)) == []
