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

"""Detectors behind one protocol: the tracker never knows which one ran.

``UltralyticsDetector`` runs a YOLO model (``.pt`` anywhere, or the same TensorRT
``.engine`` the flown ``yolo_live_trackfeed.py`` built on the Jetson, which ultralytics
loads directly). ``BrightBlobDetector`` is the fake for tests and gates: it finds the
white square of the synthetic clip by thresholding, so the tracker, line of sight and
ground intersection run on real pixels without a GPU or a model.
"""

from __future__ import annotations

from typing import Any, Protocol

import numpy as np
from numpy.typing import NDArray

from dimos.robot.px4.perception.tracker import Detection

# COCO ids of the classes the flown detector emitted (yolo_live_trackfeed.py CLASS_NAMES).
CLASS_NAMES: dict[int, str] = {
    0: "person",
    1: "bicycle",
    2: "car",
    3: "motorcycle",
    5: "bus",
    7: "truck",
    16: "dog",
}


class Detector(Protocol):
    def detect(self, rgb: NDArray[np.uint8]) -> list[Detection]: ...


class BrightBlobDetector:
    """Everything brighter than ``threshold`` is one ``person``. For synthetic footage only."""

    def __init__(self, threshold: int = 200, confidence: float = 0.9, min_pixels: int = 16) -> None:
        self._threshold = threshold
        self._confidence = confidence
        self._min_pixels = min_pixels

    def detect(self, rgb: NDArray[np.uint8]) -> list[Detection]:
        mask = rgb.mean(axis=2) > self._threshold
        ys, xs = np.nonzero(mask)
        if xs.size < self._min_pixels:
            return []
        x0, x1 = float(xs.min()), float(xs.max()) + 1.0
        y0, y1 = float(ys.min()), float(ys.max()) + 1.0
        return [Detection(0, "person", self._confidence, (x0, y0, x1 - x0, y1 - y0))]


class UltralyticsDetector:
    """YOLO through ultralytics. ``model`` is a ``.pt`` or a TensorRT ``.engine``."""

    def __init__(
        self,
        model: str,
        conf_threshold: float = 0.25,
        allowed_classes: dict[int, str] | None = None,
        imgsz: int = 640,
    ) -> None:
        # Heavy optional dependency (torch); load here, never at import.
        from ultralytics import YOLO  # type: ignore[attr-defined]

        self._model = YOLO(model)
        self._conf = conf_threshold
        self._classes = allowed_classes or CLASS_NAMES
        self._imgsz = imgsz

    def detect(self, rgb: NDArray[np.uint8]) -> list[Detection]:
        results: Any = self._model.predict(
            rgb, conf=self._conf, classes=list(self._classes), imgsz=self._imgsz, verbose=False
        )
        out: list[Detection] = []
        for r in results:
            boxes = r.boxes
            if boxes is None:
                continue
            for xyxy, cls, conf in zip(
                boxes.xyxy.tolist(), boxes.cls.tolist(), boxes.conf.tolist(), strict=True
            ):
                class_id = int(cls)
                x1, y1, x2, y2 = (float(v) for v in xyxy)
                out.append(
                    Detection(
                        class_id,
                        self._classes.get(class_id, str(class_id)),
                        float(conf),
                        (x1, y1, x2 - x1, y2 - y1),
                    )
                )
        out.sort(key=lambda d: d.confidence, reverse=True)
        return out
