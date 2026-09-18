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

"""Persistent-ID multi-object tracker: per-track Kalman filter, Hungarian association.

Ported from drone-autonomy ``jetson_patches/track_manager_select.py`` (flown 2026-09-09):
same parameters, same gates, same duplicate suppression. A track needs MIN_HITS
observations before it is shown, keeps its identity for MAX_MISSED frames, and leaves the
visible output after MAX_OUTPUT_MISSED misses. Predicted (missed) tracks are identity
memory only; the gimbal and the line of sight never act on them.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any

import numpy as np
from numpy.typing import NDArray
from scipy.optimize import linear_sum_assignment

# Track must get this many successful observations before we consider it confirmed.
MIN_HITS = 3
# A detection must be this confident to CREATE a brand-new ID; lower-confidence detections
# still UPDATE an existing track through association.
NEW_TRACK_MIN_CONF = 0.65
# Identity memory for about 2 seconds at 25 FPS.
MAX_MISSED = 50
# Predicted tracks disappear from the visible output quickly.
MAX_OUTPUT_MISSED = 2
# Association gates.
MIN_IOU_GATE = 0.05
CENTER_GATE_MULTIPLIER = 1.75
MIN_CENTER_GATE_PX = 80.0
# A second detector box around the SAME physical object must not spawn a new ID; the
# thresholds are conservative so two people standing near each other keep separate IDs.
DUPLICATE_IOU = 0.35
DUPLICATE_CENTER_RATIO = 0.22
_GATED_OUT = 1e6


@dataclass(frozen=True)
class Detection:
    class_id: int
    class_name: str
    confidence: float
    bbox: tuple[float, float, float, float]  # x, y, w, h in pixels of the detector's frame


@dataclass(frozen=True)
class TrackOutput:
    track_id: int
    class_id: int
    class_name: str
    confidence: float
    bbox: tuple[float, float, float, float]
    center: tuple[float, float]
    velocity_px_s: tuple[float, float]
    age: int
    hits: int
    missed_frames: int
    predicted: bool
    selected: bool = False


def xywh_to_center(box: tuple[float, float, float, float]) -> NDArray[np.float32]:
    x, y, w, h = box
    return np.array([x + w / 2.0, y + h / 2.0, w, h], dtype=np.float32)


def center_to_xywh(state: NDArray[Any]) -> list[float]:
    cx, cy, w, h = (float(v) for v in state[:4])
    return [cx - w / 2.0, cy - h / 2.0, w, h]


def iou_xywh(a: Any, b: Any) -> float:
    ax1, ay1, aw, ah = a
    bx1, by1, bw, bh = b
    ax2, ay2, bx2, by2 = ax1 + aw, ay1 + ah, bx1 + bw, by1 + bh
    iw = max(0.0, min(ax2, bx2) - max(ax1, bx1))
    ih = max(0.0, min(ay2, by2) - max(ay1, by1))
    inter = iw * ih
    union = aw * ah + bw * bh - inter
    return float(inter / union) if union > 0 else 0.0


def center_distance(a: Any, b: Any) -> float:
    ca, cb = xywh_to_center(a), xywh_to_center(b)
    return float(math.hypot(ca[0] - cb[0], ca[1] - cb[1]))


class Track:
    """One identity: an 8-state constant-velocity Kalman filter over [cx, cy, w, h, v...]."""

    def __init__(self, track_id: int, detection: Detection) -> None:
        # Heavy optional dependency; the flown tracker used OpenCV's filter and so does this.
        import cv2

        self.track_id = track_id
        self.class_id = detection.class_id
        self.class_name = detection.class_name
        self.confidence = detection.confidence
        self.age = 1
        self.hits = 1
        self.missed = 0
        self.last_bbox = [float(v) for v in detection.bbox]
        self.kf = cv2.KalmanFilter(8, 4)
        self.kf.measurementMatrix = np.zeros((4, 8), dtype=np.float32)
        for i in range(4):
            self.kf.measurementMatrix[i, i] = 1.0
        self.kf.transitionMatrix = np.eye(8, dtype=np.float32)
        self.kf.processNoiseCov = np.eye(8, dtype=np.float32) * 0.05
        for i in range(4, 8):
            self.kf.processNoiseCov[i, i] = 1.0  # velocity may vary more
        self.kf.measurementNoiseCov = np.eye(4, dtype=np.float32) * 2.0
        self.kf.errorCovPost = np.eye(8, dtype=np.float32) * 10.0
        self.kf.statePost = np.zeros((8, 1), dtype=np.float32)
        self.kf.statePost[:4, 0] = xywh_to_center(detection.bbox)
        self.predicted_bbox = list(self.last_bbox)

    def predict(self, dt: float) -> list[float]:
        dt = max(0.001, min(0.2, float(dt)))
        transition = np.eye(8, dtype=np.float32)
        transition[0, 4] = transition[1, 5] = transition[2, 6] = transition[3, 7] = dt
        self.kf.transitionMatrix = transition
        prediction = self.kf.predict()
        self.predicted_bbox = center_to_xywh(prediction[:, 0])
        self.age += 1
        return self.predicted_bbox

    def update(self, detection: Detection) -> None:
        measurement = xywh_to_center(detection.bbox).reshape(4, 1)
        corrected = self.kf.correct(measurement)
        self.last_bbox = center_to_xywh(corrected[:, 0])
        self.predicted_bbox = list(self.last_bbox)
        self.confidence = 0.70 * self.confidence + 0.30 * detection.confidence
        self.hits += 1
        self.missed = 0

    def mark_missed(self) -> None:
        self.missed += 1
        self.last_bbox = list(self.predicted_bbox)

    def confirmed(self) -> bool:
        return self.hits >= MIN_HITS

    def output(self, selected: bool = False) -> TrackOutput:
        state = self.kf.statePost[:, 0]
        b = self.last_bbox
        return TrackOutput(
            track_id=self.track_id,
            class_id=self.class_id,
            class_name=self.class_name,
            confidence=float(self.confidence),
            bbox=(float(b[0]), float(b[1]), float(b[2]), float(b[3])),
            center=(float(state[0]), float(state[1])),
            velocity_px_s=(float(state[4]), float(state[5])),
            age=self.age,
            hits=self.hits,
            missed_frames=self.missed,
            predicted=bool(self.missed > 0),
            selected=selected,
        )


class Tracker:
    def __init__(self) -> None:
        self.tracks: list[Track] = []
        self.next_id = 1
        self.last_timestamp: float | None = None

    def _looks_like_existing_track(self, detection: Detection) -> bool:
        """An unmatched detection that is a duplicate box around an existing same-class object.
        Only consulted before creating a NEW ID; normal association is unaffected."""
        det_box = detection.bbox
        for track in self.tracks:
            if track.class_id != detection.class_id:
                continue
            for track_box in (track.last_bbox, track.predicted_bbox):
                if iou_xywh(track_box, det_box) >= DUPLICATE_IOU:
                    return True
                distance = center_distance(track_box, det_box)
                diag = math.hypot(max(track_box[2], det_box[2]), max(track_box[3], det_box[3]))
                if diag > 1.0:
                    track_area = max(1.0, track_box[2] * track_box[3])
                    det_area = max(1.0, det_box[2] * det_box[3])
                    area_ratio = det_area / track_area
                    if distance / diag <= DUPLICATE_CENTER_RATIO and 0.50 <= area_ratio <= 2.00:
                        return True
        return False

    def _new_track(self, detection: Detection) -> None:
        self.tracks.append(Track(self.next_id, detection))
        self.next_id += 1

    def update(self, detections: list[Detection], timestamp: float) -> list[TrackOutput]:
        dt = 1.0 / 25.0 if self.last_timestamp is None else timestamp - self.last_timestamp
        self.last_timestamp = timestamp
        predicted = [track.predict(dt) for track in self.tracks]

        if not self.tracks:
            for det in detections:
                if det.confidence >= NEW_TRACK_MIN_CONF and not self._looks_like_existing_track(
                    det
                ):
                    self._new_track(det)
            return self.visible_tracks()

        if not detections:
            for track in self.tracks:
                track.mark_missed()
            self.remove_dead()
            return self.visible_tracks()

        cost = np.full((len(self.tracks), len(detections)), _GATED_OUT, dtype=np.float32)
        for ti, track in enumerate(self.tracks):
            track_box = predicted[ti]
            for di, det in enumerate(detections):
                if det.class_id != track.class_id:
                    continue  # never associate different classes
                overlap = iou_xywh(track_box, det.bbox)
                distance = center_distance(track_box, det.bbox)
                diag = math.hypot(max(track_box[2], det.bbox[2]), max(track_box[3], det.bbox[3]))
                gate_distance = max(MIN_CENTER_GATE_PX, CENTER_GATE_MULTIPLIER * diag)
                if overlap < MIN_IOU_GATE and distance > gate_distance:
                    continue  # implausible
                normalized_distance = min(1.0, distance / max(gate_distance, 1.0))
                # IoU dominates; centre distance helps during motion and imperfect boxes.
                cost[ti, di] = 0.75 * (1.0 - overlap) + 0.25 * normalized_distance

        track_indices, det_indices = linear_sum_assignment(cost)
        matched_tracks: set[int] = set()
        matched_detections: set[int] = set()
        for ti, di in zip(track_indices, det_indices, strict=True):
            if cost[ti, di] >= 1e5:
                continue  # gated out
            self.tracks[ti].update(detections[di])
            matched_tracks.add(int(ti))
            matched_detections.add(int(di))
        for ti, track in enumerate(self.tracks):
            if ti not in matched_tracks:
                track.mark_missed()
        for di, det in enumerate(detections):
            if di not in matched_detections and det.confidence >= NEW_TRACK_MIN_CONF:
                if not self._looks_like_existing_track(det):
                    self._new_track(det)
        self.remove_dead()
        return self.visible_tracks()

    def remove_dead(self) -> None:
        self.tracks = [track for track in self.tracks if track.missed <= MAX_MISSED]

    def visible_tracks(self, selected_id: int | None = None) -> list[TrackOutput]:
        return [
            track.output(selected=(selected_id is not None and track.track_id == selected_id))
            for track in self.tracks
            if track.confirmed() and track.missed <= MAX_OUTPUT_MISSED
        ]
