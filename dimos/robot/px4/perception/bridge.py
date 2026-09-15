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

"""PerceptionBridge: the flown detector -> tracker -> line of sight -> target chain, in one process.

Consumes the camera's decoded frames and the connection's odometry, gimbal attitude,
home-relative pose and status, and publishes the exact three streams the connection
reads for FOLLOW and YAW_TRACK: ``target_state``, ``target_valid``, ``target_los``.
Without it the connection simply never leaves HOVER for FOLLOW or YAW_TRACK.

The four flown scripts (``yolo_live_trackfeed``, ``track_manager_select``,
``los_estimator``, ``target_estimator``) talked over UDP because they were processes; in
one process a frame goes detector, tracker, LOS, Kalman as function calls, so the UDP
ports are gone. The geometry and tracker parameters are the ones that flew (the sibling
files in this directory).

Operator click-to-select arrives on ``track_select`` as a pixel in the published frame;
a NaN point clears the selection, the convention MovementManager uses to cancel a goal.
"""

from __future__ import annotations

import math
import queue
import threading
import time
from typing import Any, Literal

from dimos_lcm.std_msgs import Bool  # type: ignore[import-untyped]
from dimos_lcm.std_msgs.Header import Header
from dimos_lcm.vision_msgs import BoundingBox2D, Detection2D, ObjectHypothesisWithPose
from pydantic import Field
from reactivex.disposable import Disposable

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.px4_msgs.VehicleStatus import VehicleStatus
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.vision_msgs.Detection2DArray import Detection2DArray
from dimos.robot.px4.mavlink import TimedBuffer, flu_to_ned, ned_to_flu
from dimos.robot.px4.perception.detector import BrightBlobDetector, Detector, UltralyticsDetector
from dimos.robot.px4.perception.estimators import (
    EstimatorConfig,
    GimbalGeo,
    LosEstimator,
    LosResult,
    TargetEstimator,
    TargetState,
    VehicleGeo,
)
from dimos.robot.px4.perception.geometry import (
    CameraConfig,
    CameraModel,
    GimbalFrameConfig,
    LOSSolver,
)
from dimos.robot.px4.perception.tracker import Tracker, TrackOutput
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class PerceptionBridgeConfig(ModuleConfig):
    # "ultralytics" runs YOLO (a .pt anywhere, or the Jetson's TensorRT .engine); "blob"
    # is the synthetic-footage fake for tests and gates.
    detector: Literal["ultralytics", "blob"] = Field(default="ultralytics")
    model: str = Field(default="yolov8n.pt")
    conf_threshold: float = Field(default=0.25)
    camera: CameraConfig = Field(default_factory=CameraConfig)
    gimbal_frame: GimbalFrameConfig = Field(default_factory=GimbalFrameConfig)
    estimator: EstimatorConfig = Field(default_factory=EstimatorConfig)
    odom_frame_id: str = Field(default="odom")
    base_frame_id: str = Field(default="base_link")
    optical_frame_id: str = Field(default="a8_optical")
    # Intrinsics hold only at 1x; the gimbal module withholds camera_info otherwise.
    assume_zoom_1x: bool = Field(default=True)
    # A click selects the track whose box contains it, else the nearest centre within this.
    select_radius_px: float = Field(default=80.0)
    sensor_stats_interval_s: float = Field(default=10.0)


class PerceptionBridge(Module):
    """Detector, tracker, line of sight and target estimator on the camera's frames."""

    # YOLO and the decode-heavy frames want their own interpreter.
    dedicated_worker = True

    config: PerceptionBridgeConfig

    color_image: In[Image]
    odometry: In[Odometry]
    gimbal_attitude: In[JointState]
    global_pose: In[PoseStamped]
    vehicle_status: In[VehicleStatus]
    track_select: In[PointStamped]

    tracks: Out[Detection2DArray]
    target_state: Out[Odometry]
    target_valid: Out[Bool]
    target_los: Out[PoseStamped]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        cfg = self.config
        self._lock = threading.Lock()
        self._detector: Detector | None = None
        self._tracker = Tracker()
        self._los = LosEstimator(
            LOSSolver(CameraModel(cfg.camera), cfg.gimbal_frame), cfg.estimator
        )
        self._target = TargetEstimator(cfg.estimator)
        self._attitude = TimedBuffer(2.0, angular=("roll", "pitch", "yaw"))
        self._gimbal = TimedBuffer(2.0, angular=("roll", "pitch", "yaw"))
        self._gimbal_flags = 0
        self._gimbal_failure = 0
        self._position: tuple[float, float, float] | None = None
        self._rel_alt: float | None = None
        self._armed = False
        self._selected: int | None = None
        self._last_tracks: list[TrackOutput] = []
        self._last_los: LosResult | None = None
        self._last_state: TargetState | None = None
        self._frames = 0
        self._dropped = 0
        self._queue: queue.Queue[Image | None] = queue.Queue(maxsize=1)
        self._stop_event = threading.Event()
        self._worker: threading.Thread | None = None

    # Lifecycle

    @rpc
    def start(self) -> None:
        super().start()
        self._detector = self.make_detector()
        self.register_disposable(Disposable(self.color_image.subscribe(self._on_frame)))
        self.register_disposable(Disposable(self.odometry.subscribe(self._on_odometry)))
        self.register_disposable(Disposable(self.gimbal_attitude.subscribe(self._on_gimbal)))
        self.register_disposable(Disposable(self.global_pose.subscribe(self._on_global_pose)))
        self.register_disposable(Disposable(self.vehicle_status.subscribe(self._on_vehicle_status)))
        self.register_disposable(Disposable(self.track_select.subscribe(self._on_track_select)))
        self._stop_event.clear()
        self._worker = threading.Thread(
            target=self._worker_loop, name="perception-worker", daemon=True
        )
        self._worker.start()

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        try:
            self._queue.put_nowait(None)
        except queue.Full:
            pass
        if self._worker is not None:
            self._worker.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
            self._worker = None
        super().stop()

    def make_detector(self) -> Detector:
        cfg = self.config
        if cfg.detector == "blob":
            return BrightBlobDetector()
        return UltralyticsDetector(cfg.model, cfg.conf_threshold)

    # Inputs

    def _on_frame(self, image: Image) -> None:
        # Latest frame wins: a tracker behind the camera is worse than a skipped frame.
        try:
            self._queue.put_nowait(image)
        except queue.Full:
            try:
                self._queue.get_nowait()
            except queue.Empty:
                pass
            self._dropped += 1
            try:
                self._queue.put_nowait(image)
            except queue.Full:
                pass

    def _on_odometry(self, msg: Odometry) -> None:
        e = msg.orientation.to_euler()
        # dimOS FLU euler -> the NED/FRD convention the flown geometry uses.
        with self._lock:
            self._attitude.push(
                msg.ts,
                {"roll": math.degrees(e.x), "pitch": -math.degrees(e.y), "yaw": -math.degrees(e.z)},
            )
            self._position = flu_to_ned(msg.x, msg.y, msg.z)

    def _on_gimbal(self, msg: JointState) -> None:
        angles = dict(zip(msg.name, msg.position, strict=False))
        if not {"gimbal_roll", "gimbal_pitch", "gimbal_yaw"} <= set(angles):
            return
        with self._lock:
            self._gimbal.push(
                msg.ts,
                {
                    "roll": math.degrees(angles["gimbal_roll"]),
                    "pitch": math.degrees(angles["gimbal_pitch"]),
                    "yaw": math.degrees(angles["gimbal_yaw"]),
                },
            )
            self._gimbal_flags = int(msg.effort[0]) if msg.effort else 0
            self._gimbal_failure = int(msg.effort[1]) if len(msg.effort) > 1 else 0

    def _on_global_pose(self, msg: PoseStamped) -> None:
        with self._lock:
            self._rel_alt = float(msg.z)

    def _on_vehicle_status(self, msg: VehicleStatus) -> None:
        with self._lock:
            self._armed = msg.armed

    def _on_track_select(self, msg: PointStamped) -> None:
        if math.isnan(msg.x) or math.isnan(msg.y):
            self.clear_selection()
            return
        with self._lock:
            tracks = list(self._last_tracks)
        chosen: int | None = None
        best = self.config.select_radius_px
        for t in tracks:
            x, y, w, h = t.bbox
            if x <= msg.x <= x + w and y <= msg.y <= y + h:
                chosen = t.track_id
                break
            d = math.hypot(t.center[0] - msg.x, t.center[1] - msg.y)
            if d < best:
                best, chosen = d, t.track_id
        if chosen is not None:
            self.select_track(chosen)

    # Processing

    def _worker_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                image = self._queue.get(timeout=0.5)
            except queue.Empty:
                continue
            if image is None:
                break
            try:
                self.process_frame(image)
            except Exception:
                logger.exception("perception frame failed")

    def process_frame(self, image: Image, now: float | None = None) -> TargetState:
        """Run the whole chain on one frame and publish. Returns the target state."""
        wall = time.time() if now is None else now
        detector = self._detector or self.make_detector()
        self._detector = detector
        rgb = image.data if image.format is ImageFormat.RGB else image.to_rgb().data
        detections = detector.detect(rgb)
        with self._lock:
            tracks = self._tracker.update(detections, image.ts)
            selected = self._selected
            tracks = [
                t.output(selected=t.track_id == selected)
                for t in self._tracker.tracks
                if t.confirmed() and t.missed <= 2
            ]
            self._last_tracks = tracks
            veh, gimbal = self._geo(image.ts, wall)
        zoom = 1.0 if self.config.assume_zoom_1x else None
        h, w = rgb.shape[0], rgb.shape[1]
        los = self._los.process(tracks, selected, image.ts, (w, h), veh, gimbal, zoom, wall)
        state = self._target.process(los, veh, wall)
        with self._lock:
            self._last_los, self._last_state = los, state
            self._frames += 1
        self._publish(tracks, los, state, image)
        return state

    def _geo(self, capture_time: float, now: float) -> tuple[VehicleGeo, GimbalGeo | None]:
        att = self._attitude.at(capture_time)
        t_att, _ = self._attitude.latest()
        gim = self._gimbal.at(capture_time)
        t_gim, _ = self._gimbal.latest()
        pos = self._position
        veh = VehicleGeo(
            yaw_deg=att["yaw"] if att else None,
            roll_deg=att["roll"] if att else None,
            pitch_deg=att["pitch"] if att else None,
            attitude_age_s=(now - t_att) if t_att is not None else math.inf,
            n=pos[0] if pos else None,
            e=pos[1] if pos else None,
            d=pos[2] if pos else None,
            rel_alt=self._rel_alt,
            armed=self._armed,
        )
        gimbal = None
        if gim is not None and t_gim is not None:
            gimbal = GimbalGeo(
                gim["roll"],
                gim["pitch"],
                gim["yaw"],
                now - t_gim,
                self._gimbal_flags,
                self._gimbal_failure,
            )
        return veh, gimbal

    # Outputs

    def _publish(
        self,
        tracks: list[TrackOutput],
        los: LosResult,
        state: TargetState,
        image: Image,
    ) -> None:
        self.tracks.publish(_detections(tracks, image.ts, self.config.optical_frame_id))
        self.target_valid.publish(Bool(data=state.valid))
        if state.n is not None and state.e is not None:
            x, y, _ = ned_to_flu(state.n, state.e, 0.0)
            vx, vy, _ = ned_to_flu(state.vn or 0.0, state.ve or 0.0, 0.0)
            self.target_state.publish(
                Odometry(
                    ts=state.t,
                    frame_id=self.config.odom_frame_id,
                    child_frame_id="target",
                    pose=Pose(Vector3(x, y, 0.0), Quaternion()),
                    twist=Twist(Vector3(vx, vy, 0.0), Vector3()),
                )
            )
        if los.valid and los.gimbal_yaw_body_deg is not None and los.elevation_deg is not None:
            # base_link FLU: yaw counter-clockwise, pitch nose-down positive.
            self.target_los.publish(
                PoseStamped(
                    ts=los.capture_time,
                    frame_id=self.config.base_frame_id,
                    position=Vector3(),
                    orientation=Quaternion.from_euler(
                        Vector3(
                            0.0,
                            -math.radians(los.elevation_deg),
                            -math.radians(los.gimbal_yaw_body_deg),
                        )
                    ),
                )
            )

    # RPCs

    @rpc
    def select_track(self, track_id: int) -> dict[str, Any]:
        """Follow this track id. The selection persists while the track is lost."""
        with self._lock:
            self._selected = int(track_id)
        logger.info("target selected", track_id=track_id)
        return {"selected_track_id": int(track_id)}

    @rpc
    def clear_selection(self) -> dict[str, Any]:
        """Drop the selection; target_valid goes false."""
        with self._lock:
            self._selected = None
        return {"selected_track_id": None}

    @rpc
    def status(self) -> dict[str, Any]:
        """Selection, visible tracks, the last line-of-sight verdict and target state."""
        with self._lock:
            los, state, tracks = self._last_los, self._last_state, list(self._last_tracks)
            return {
                "selected_track_id": self._selected,
                "frames": self._frames,
                "dropped": self._dropped,
                "tracks": [
                    {
                        "track_id": t.track_id,
                        "class_name": t.class_name,
                        "confidence": round(t.confidence, 2),
                    }
                    for t in tracks
                ],
                "los": None
                if los is None
                else {
                    "valid": los.valid,
                    "reason": los.reason,
                    "azimuth_deg": los.azimuth_deg,
                    "elevation_deg": los.elevation_deg,
                },
                "target": None
                if state is None
                else {
                    "valid": state.valid,
                    "reason": state.reason,
                    "n": state.n,
                    "e": state.e,
                    "range_m": state.range_m,
                    "bearing_deg": state.bearing_deg,
                },
            }


def _detections(tracks: list[TrackOutput], ts: float, frame_id: str) -> Detection2DArray:
    """The visible tracks as a Detection2DArray, selected track first."""
    ordered = sorted(tracks, key=lambda t: not t.selected)
    msg = Detection2DArray()
    header = Header()
    header.frame_id = frame_id
    header.stamp.sec = int(ts)
    header.stamp.nsec = int((ts - int(ts)) * 1e9)
    msg.header = header
    dets = []
    for t in ordered:
        d = Detection2D()
        d.header = header
        d.id = f"{t.track_id}{'*' if t.selected else ''}"
        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = t.class_name
        hyp.hypothesis.score = t.confidence
        d.results = [hyp]
        d.results_length = 1
        box = BoundingBox2D()
        box.center.position.x = t.center[0]
        box.center.position.y = t.center[1]
        box.center.theta = 0.0
        box.size_x = t.bbox[2]
        box.size_y = t.bbox[3]
        d.bbox = box
        dets.append(d)
    msg.detections = dets
    msg.detections_length = len(dets)
    return msg
