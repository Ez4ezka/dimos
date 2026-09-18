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

"""Line-of-sight estimator and target ground-position estimator. Pure, no I/O.

Ported from drone-autonomy ``los_estimator.py`` (the ``process`` function) and
``target_estimator.py`` (TargetKF, agl_metres, build_output), flown 2026-09-09. The LOS
is valid only for a selected track with a fresh REAL observation (not predicted), fresh
vehicle and gimbal attitude, no gimbal failure and 1x zoom, the same rule the gimbal
controller applied. The target estimator intersects the ray with a flat ground plane at the
takeoff height and runs a constant-velocity Kalman filter on [N, E, VN, VE].
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
import math

import numpy as np
from numpy.typing import NDArray

from dimos.robot.px4.perception.geometry import LOSSolver, intersect_ground, ne_to_latlon
from dimos.robot.px4.perception.tracker import TrackOutput


@dataclass(frozen=True)
class EstimatorConfig:
    """``config/follow.json`` minus the guidance block."""

    agl_source: str = "relative_alt"  # relative_alt | local_arm | fixed
    fixed_agl_m: float = 1.0
    camera_below_ref_m: float = 0.10
    aim_height_m: dict[str, float] = field(
        default_factory=lambda: {
            "person": 1.0,
            "bicycle": 0.9,
            "motorcycle": 0.8,
            "car": 0.8,
            "bus": 1.5,
            "truck": 1.5,
            "dog": 0.3,
            "default": 0.8,
        }
    )
    min_depression_deg: float = 5.0
    max_range_m: float = 150.0
    kf_q_pos: float = 0.05
    kf_q_vel: float = 0.8
    kf_r_base_m: float = 0.5
    kf_r_per_m: float = 0.06
    kf_init_pos_sigma_m: float = 5.0
    kf_init_vel_sigma_mps: float = 2.0
    max_meas_age_s: float = 1.0
    drop_after_s: float = 5.0
    max_state_age_s: float = 0.5
    max_gimbal_age_s: float = 1.0


@dataclass(frozen=True)
class VehicleGeo:
    """The vehicle as the estimators need it: attitude, position, height, armed."""

    yaw_deg: float | None = None
    roll_deg: float | None = None
    pitch_deg: float | None = None
    attitude_age_s: float = math.inf
    n: float | None = None
    e: float | None = None
    d: float | None = None
    lat: float | None = None
    lon: float | None = None
    rel_alt: float | None = None
    armed: bool = False


@dataclass(frozen=True)
class GimbalGeo:
    roll_deg: float
    pitch_deg: float
    yaw_deg: float
    age_s: float
    flags: int = 0
    failure_flags: int = 0


@dataclass(frozen=True)
class LosResult:
    """``target_los_v1``."""

    t: float
    capture_time: float
    selected_track_id: int | None
    track_id: int | None = None
    class_name: str | None = None
    valid: bool = False
    reason: str = ""
    observation: str = "none"
    bbox: tuple[float, float, float, float] | None = None
    zoom: float = 1.0
    los_ned: tuple[float, float, float] | None = None
    azimuth_deg: float | None = None
    elevation_deg: float | None = None
    gimbal_yaw_body_deg: float | None = None
    gimbal_yaw_ned_deg: float | None = None
    gimbal_pitch_deg: float | None = None
    cam_h_deg: float | None = None
    cam_v_deg: float | None = None
    pixel: tuple[float, float] | None = None


class LosEstimator:
    def __init__(self, solver: LOSSolver, cfg: EstimatorConfig) -> None:
        self.solver = solver
        self.cfg = cfg

    def process(
        self,
        tracks: list[TrackOutput],
        selected: int | None,
        capture_time: float,
        frame_size: tuple[int, int],
        vehicle: VehicleGeo,
        gimbal: GimbalGeo | None,
        zoom: float | None,
        now: float,
    ) -> LosResult:
        base = LosResult(
            t=now,
            capture_time=capture_time,
            selected_track_id=selected,
            zoom=zoom or 1.0,
            gimbal_pitch_deg=gimbal.pitch_deg if gimbal is not None else None,
        )
        if selected is None:
            return replace(base, reason="no selection")
        track = next((t for t in tracks if t.track_id == selected), None)
        if track is None:
            return replace(base, reason="selected target not visible")
        base = replace(base, track_id=track.track_id, class_name=track.class_name, bbox=track.bbox)
        if track.missed_frames > 0 or track.predicted:
            return replace(base, observation="predicted", reason="only predicted track")
        base = replace(base, observation="real")
        if vehicle.yaw_deg is None or vehicle.attitude_age_s > self.cfg.max_state_age_s:
            return replace(base, reason="vehicle attitude stale")
        if gimbal is None or gimbal.age_s > self.cfg.max_gimbal_age_s:
            return replace(base, reason="gimbal attitude stale")
        if gimbal.failure_flags:
            return replace(base, reason=f"gimbal failure_flags={gimbal.failure_flags}")
        if zoom is not None and abs(zoom - 1.0) > 0.05:
            return replace(base, reason=f"zoom {zoom} != 1.0 (intrinsics only valid at 1x)")
        x, y, w, h = track.bbox
        u, v = x + w / 2.0, y + h / 2.0
        sw, sh = float(frame_size[0]), float(frame_size[1])
        u *= self.solver.cam.w / sw
        v *= self.solver.cam.h / sh
        r = self.solver.solve(
            u,
            v,
            {"roll": gimbal.roll_deg, "pitch": gimbal.pitch_deg, "yaw": gimbal.yaw_deg},
            vehicle.yaw_deg,
            gimbal.flags,
            zoom=1.0,
        )
        return replace(
            base,
            valid=True,
            reason="ok",
            los_ned=r.los_ned,
            azimuth_deg=r.azimuth_deg,
            elevation_deg=r.elevation_deg,
            gimbal_yaw_body_deg=r.gimbal_yaw_body_deg,
            gimbal_yaw_ned_deg=r.gimbal_yaw_ned_deg,
            cam_h_deg=r.cam_h_deg,
            cam_v_deg=r.cam_v_deg,
            pixel=(u, v),
        )


class TargetKF:
    """[n, e, vn, ve] constant-velocity filter."""

    def __init__(self, cfg: EstimatorConfig) -> None:
        self.cfg = cfg
        self.x: NDArray[np.float64] | None = None
        self.P: NDArray[np.float64] = np.eye(4)
        self.t = 0.0
        self.t_meas: float | None = None
        self.track_id: int | None = None

    def reset(self, track_id: int | None, n: float, e: float, t: float) -> None:
        c = self.cfg
        self.x = np.array([n, e, 0.0, 0.0])
        self.P = np.diag([c.kf_init_pos_sigma_m**2] * 2 + [c.kf_init_vel_sigma_mps**2] * 2)
        self.t = t
        self.t_meas = t
        self.track_id = track_id

    def predict(self, t: float) -> None:
        if self.x is None:
            return
        dt = max(0.0, min(1.0, t - self.t))
        f = np.eye(4)
        f[0, 2] = f[1, 3] = dt
        q_p, q_v = self.cfg.kf_q_pos, self.cfg.kf_q_vel
        q = np.diag([q_p * dt, q_p * dt, q_v * dt, q_v * dt])
        self.x = f @ self.x
        self.P = f @ self.P @ f.T + q
        self.t = t

    def update(self, n: float, e: float, range_m: float, t: float) -> None:
        self.predict(t)
        assert self.x is not None
        sigma = self.cfg.kf_r_base_m + self.cfg.kf_r_per_m * range_m
        r = np.eye(2) * sigma**2
        h = np.zeros((2, 4))
        h[0, 0] = h[1, 1] = 1.0
        z = np.array([n, e])
        y = z - h @ self.x
        s = h @ self.P @ h.T + r
        k = self.P @ h.T @ np.linalg.inv(s)
        self.x = self.x + k @ y
        self.P = (np.eye(4) - k @ h) @ self.P
        self.t_meas = t


@dataclass(frozen=True)
class TargetState:
    """``target_state_v1``."""

    t: float
    track_id: int | None
    valid: bool
    reason: str
    los_valid: bool
    los_age_s: float | None
    observation: str | None
    gimbal_yaw_body_deg: float | None
    gimbal_pitch_deg: float | None
    azimuth_deg: float | None
    elevation_deg: float | None
    agl_m: float | None
    agl_source: str | None
    n: float | None = None
    e: float | None = None
    vn: float | None = None
    ve: float | None = None
    speed_mps: float | None = None
    age_s: float | None = None
    range_m: float | None = None
    bearing_deg: float | None = None
    pos_sigma_m: float | None = None
    lat: float | None = None
    lon: float | None = None
    ground_model: str = "flat plane at takeoff height"


def agl_metres(
    cfg: EstimatorConfig, veh: VehicleGeo, z_at_arm: float | None
) -> tuple[float | None, str]:
    if cfg.agl_source == "fixed":
        return cfg.fixed_agl_m, "fixed"
    if cfg.agl_source == "local_arm":
        if veh.d is None or z_at_arm is None:
            return None, "local z unavailable"
        return (z_at_arm - veh.d) - cfg.camera_below_ref_m, "local_arm"
    if veh.rel_alt is None:
        return None, "relative_alt unavailable"
    return veh.rel_alt - cfg.camera_below_ref_m, "relative_alt"


class TargetEstimator:
    def __init__(self, cfg: EstimatorConfig) -> None:
        self.cfg = cfg
        self.kf = TargetKF(cfg)
        self.z_at_arm: float | None = None
        self._was_armed = False

    def process(self, los: LosResult | None, veh: VehicleGeo, now: float) -> TargetState:
        cfg = self.cfg
        if veh.armed and not self._was_armed and veh.d is not None:
            self.z_at_arm = veh.d
        self._was_armed = veh.armed
        reason: str | None = None
        meas: dict[str, float] | None = None
        agl_val: float | None = None
        agl_src: str | None = None
        if los is None:
            reason = "no LOS packets"
        elif not los.valid or los.los_ned is None:
            reason = f"LOS: {los.reason}"
        else:
            agl_val, agl_src = agl_metres(cfg, veh, self.z_at_arm)
            if agl_val is None:
                reason = f"AGL: {agl_src}"
            elif agl_val < 0.3:
                reason = f"AGL {agl_val:.2f} m too small ({agl_src})"
            else:
                aim = cfg.aim_height_m.get(los.class_name or "", cfg.aim_height_m["default"])
                d_cam = veh.d if veh.d is not None else 0.0
                n_cam = veh.n if veh.n is not None else 0.0
                e_cam = veh.e if veh.e is not None else 0.0
                # Camera at d_cam, ground plane at d_cam + agl, aim plane aim m above ground.
                plane_d = d_cam + agl_val - aim
                pt, rng = intersect_ground(
                    (n_cam, e_cam, d_cam), los.los_ned, plane_d, cfg.min_depression_deg
                )
                if pt is None:
                    reason = f"intersect: {rng}"
                elif isinstance(rng, float) and rng > cfg.max_range_m:
                    reason = f"range {rng:.0f} m > max"
                else:
                    assert isinstance(rng, float)
                    meas = {"n": pt[0], "e": pt[1], "range_m": rng, "t": los.capture_time or now}
        if meas is not None and los is not None:
            if self.kf.track_id != los.track_id or self.kf.x is None:
                self.kf.reset(los.track_id, meas["n"], meas["e"], meas["t"])
            else:
                self.kf.update(meas["n"], meas["e"], meas["range_m"], meas["t"])
        else:
            self.kf.predict(now)
            if self.kf.t_meas is not None and now - self.kf.t_meas > cfg.drop_after_s:
                self.kf.x = None
        return self._output(los, veh, meas, agl_val, agl_src, reason, now)

    def _output(
        self,
        los: LosResult | None,
        veh: VehicleGeo,
        meas: dict[str, float] | None,
        agl_val: float | None,
        agl_src: str | None,
        reason: str | None,
        now: float,
    ) -> TargetState:
        kf = self.kf
        out: dict[str, object] = dict(
            t=now,
            track_id=kf.track_id if kf.x is not None else (los.track_id if los else None),
            valid=False,
            reason=reason or "ok",
            los_valid=bool(los.valid) if los else False,
            los_age_s=(now - los.t) if los else None,
            observation=los.observation if los else None,
            gimbal_yaw_body_deg=los.gimbal_yaw_body_deg if los else None,
            gimbal_pitch_deg=los.gimbal_pitch_deg if los else None,
            azimuth_deg=los.azimuth_deg if los else None,
            elevation_deg=los.elevation_deg if los else None,
            agl_m=agl_val if meas else None,
            agl_source=agl_src if meas else None,
        )
        if kf.x is not None and kf.t_meas is not None:
            age = now - kf.t_meas
            n, e, vn, ve = (float(v) for v in kf.x)
            dn = n - (veh.n or 0.0)
            de = e - (veh.e or 0.0)
            valid = age <= self.cfg.max_meas_age_s
            out.update(
                n=n,
                e=e,
                vn=vn,
                ve=ve,
                speed_mps=math.hypot(vn, ve),
                age_s=age,
                range_m=math.hypot(dn, de),
                bearing_deg=(math.degrees(math.atan2(de, dn)) + 360) % 360,
                pos_sigma_m=float(math.sqrt(max(kf.P[0, 0], kf.P[1, 1]))),
                valid=valid,
            )
            if not valid and out["reason"] == "ok":
                out["reason"] = f"no measurement for {age:.1f}s"
            if veh.lat is not None and veh.lon is not None and veh.n is not None:
                lat, lon = ne_to_latlon(veh.lat, veh.lon, dn, de)
                out.update(lat=lat, lon=lon)
        return TargetState(**out)  # type: ignore[arg-type]
