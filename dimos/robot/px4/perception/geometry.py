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

"""Pixel -> camera ray -> gimbal -> vehicle/NED line of sight, and ground intersection.

Frames (all right-handed): camera x = optical axis, y = right in image, z = down in
image; NED x = North, y = East, z = Down. Euler convention is aerospace ZYX,
R = Rz(yaw) @ Ry(pitch) @ Rx(roll), pitch positive = camera up, yaw positive = clockwise.

Ported from drone-autonomy ``common/geometry.py`` (flown 2026-09-09) with the maths
unchanged; the JSON configs became dataclasses with the verified values as defaults.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Literal

import numpy as np
from numpy.typing import NDArray

from dimos.hardware.gimbal.siyi.frame import (
    FLAG_YAW_IN_EARTH_FRAME,
    FLAG_YAW_IN_VEHICLE_FRAME,
    FLAG_YAW_LOCK,
)
from dimos.utils.angles import wrap180


@dataclass(frozen=True)
class CameraConfig:
    """``config/camera_a8_1x.json``: the A8 main stream at 1x. fx/fy None -> from the HFOV."""

    width: int = 1280
    height: int = 720
    hfov_deg: float = 81.0
    fx: float | None = None
    fy: float | None = None
    cx: float | None = None
    cy: float | None = None
    k1: float = 0.0
    k2: float = 0.0
    p1: float = 0.0
    p2: float = 0.0
    k3: float = 0.0


@dataclass(frozen=True)
class GimbalFrameConfig:
    """``config/gimbal_frame.json``, VERIFIED 2026-09-04 in the flight mount."""

    yaw_frame: Literal["body", "earth", "auto"] = "body"
    use_roll: bool = False
    pitch_sign: float = 1.0
    yaw_sign: float = 1.0
    yaw_offset_deg: float = 0.0


class CameraModel:
    def __init__(self, cfg: CameraConfig | None = None) -> None:
        c = cfg or CameraConfig()
        self.w, self.h = float(c.width), float(c.height)
        f_from_fov = (self.w / 2.0) / math.tan(math.radians(c.hfov_deg) / 2.0)
        self.fx = float(c.fx or f_from_fov)
        self.fy = float(c.fy or self.fx)
        self.cx = float(c.cx if c.cx is not None else self.w / 2.0)
        self.cy = float(c.cy if c.cy is not None else self.h / 2.0)
        self.dist = np.array([c.k1, c.k2, c.p1, c.p2, c.k3], dtype=float)

    def scaled(self, zoom: float) -> CameraModel:
        """Digital zoom multiplies the effective focal length."""
        m = CameraModel.__new__(CameraModel)
        m.__dict__.update(self.__dict__)
        m.fx, m.fy = self.fx * zoom, self.fy * zoom
        return m

    def pixel_to_ray(self, u: float, v: float) -> NDArray[np.float64]:
        """Unit ray in the camera frame (x forward, y right, z down)."""
        xn = (u - self.cx) / self.fx
        yn = (v - self.cy) / self.fy
        if np.any(self.dist):
            xn, yn = _undistort(xn, yn, self.dist)
        r = np.array([1.0, xn, yn])
        return np.asarray(r / np.linalg.norm(r))

    def ray_to_angles(self, ray_cam: NDArray[np.float64]) -> tuple[float, float]:
        """(horizontal, vertical) angle of a camera-frame ray, degrees (+right, +down)."""
        return (
            math.degrees(math.atan2(ray_cam[1], ray_cam[0])),
            math.degrees(math.atan2(ray_cam[2], ray_cam[0])),
        )


def _undistort(
    xn: float, yn: float, dist: NDArray[np.float64], iters: int = 5
) -> tuple[float, float]:
    k1, k2, p1, p2, k3 = dist
    x, y = xn, yn
    for _ in range(iters):
        r2 = x * x + y * y
        rad = 1 + k1 * r2 + k2 * r2 * r2 + k3 * r2**3
        dx = 2 * p1 * x * y + p2 * (r2 + 2 * x * x)
        dy = p1 * (r2 + 2 * y * y) + 2 * p2 * x * y
        x = (xn - dx) / rad
        y = (yn - dy) / rad
    return float(x), float(y)


def rot_x(a: float) -> NDArray[np.float64]:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=float)


def rot_y(a: float) -> NDArray[np.float64]:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=float)


def rot_z(a: float) -> NDArray[np.float64]:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=float)


def euler_to_rot(roll_deg: float, pitch_deg: float, yaw_deg: float) -> NDArray[np.float64]:
    """Body->reference rotation, aerospace ZYX."""
    return np.asarray(
        rot_z(math.radians(yaw_deg))
        @ rot_y(math.radians(pitch_deg))
        @ rot_x(math.radians(roll_deg))
    )


def ned_to_az_el(v: NDArray[np.float64]) -> tuple[float, float]:
    """Azimuth (0=N, 90=E) and elevation (+ = above horizon) of a NED vector, degrees."""
    n, e, d = (float(x) for x in v)
    az = (math.degrees(math.atan2(e, n)) + 360.0) % 360.0
    el = math.degrees(math.atan2(-d, math.hypot(n, e)))
    return az, el


@dataclass(frozen=True)
class LosSolution:
    los_ned: tuple[float, float, float]
    azimuth_deg: float
    elevation_deg: float
    gimbal_yaw_body_deg: float
    gimbal_yaw_ned_deg: float
    cam_h_deg: float
    cam_v_deg: float


class LOSSolver:
    """Combines camera model, gimbal attitude and vehicle attitude into a NED line of sight."""

    def __init__(self, camera: CameraModel, frame_cfg: GimbalFrameConfig | None = None) -> None:
        self.cam = camera
        self.cfg = frame_cfg or GimbalFrameConfig()

    def gimbal_rotation(self, gimbal: dict[str, float]) -> tuple[NDArray[np.float64], float]:
        """Rotation camera->reference from normalised gimbal angles dict(roll, pitch, yaw)."""
        c = self.cfg
        roll = gimbal.get("roll", 0.0) if c.use_roll else 0.0
        pitch = c.pitch_sign * gimbal["pitch"]
        yaw = wrap180(c.yaw_sign * gimbal["yaw"] + c.yaw_offset_deg)
        return euler_to_rot(roll, pitch, yaw), yaw

    def yaw_is_body(self, gimbal_flags: int) -> bool:
        mode = self.cfg.yaw_frame
        if mode == "body":
            return True
        if mode == "earth":
            return False
        if gimbal_flags & FLAG_YAW_IN_EARTH_FRAME:
            return False
        if gimbal_flags & FLAG_YAW_IN_VEHICLE_FRAME:
            return True
        return not bool(gimbal_flags & FLAG_YAW_LOCK)

    def solve(
        self,
        u: float,
        v: float,
        gimbal: dict[str, float],
        vehicle_yaw_deg: float,
        gimbal_flags: int = 0,
        zoom: float = 1.0,
    ) -> LosSolution:
        cam = self.cam if zoom == 1.0 else self.cam.scaled(zoom)
        ray_cam = cam.pixel_to_ray(u, v)
        r_g, g_yaw = self.gimbal_rotation(gimbal)
        if self.yaw_is_body(gimbal_flags):
            los = rot_z(math.radians(vehicle_yaw_deg)) @ r_g @ ray_cam
            yaw_body, yaw_ned = g_yaw, wrap180(g_yaw + vehicle_yaw_deg)
        else:
            los = r_g @ ray_cam
            yaw_body, yaw_ned = wrap180(g_yaw - vehicle_yaw_deg), g_yaw
        az, el = ned_to_az_el(los)
        h_ang, v_ang = cam.ray_to_angles(ray_cam)
        return LosSolution(
            los_ned=(float(los[0]), float(los[1]), float(los[2])),
            azimuth_deg=az,
            elevation_deg=el,
            gimbal_yaw_body_deg=yaw_body,
            gimbal_yaw_ned_deg=yaw_ned,
            cam_h_deg=h_ang,
            cam_v_deg=v_ang,
        )


def intersect_ground(
    p_ned: tuple[float, float, float],
    los_ned: tuple[float, float, float],
    ground_d: float,
    min_depression_deg: float = 5.0,
) -> tuple[tuple[float, float, float] | None, float | str]:
    """Intersect ray p + s*los with the horizontal plane D = ground_d.

    Returns (point_ned, range_m) or (None, reason). Rejects rays that look up or are
    shallower than min_depression_deg (range explodes).
    """
    p = np.asarray(p_ned, dtype=float)
    r = np.asarray(los_ned, dtype=float)
    _, el = ned_to_az_el(r)
    if el > -min_depression_deg:
        return None, f"elevation {el:.1f} deg too shallow"
    if ground_d <= p[2]:
        return None, "camera at/below ground plane"
    s = (ground_d - p[2]) / r[2]
    pt = p + s * r
    return (float(pt[0]), float(pt[1]), float(pt[2])), float(s)


def ne_to_latlon(lat0: float, lon0: float, dn: float, de: float) -> tuple[float, float]:
    """Small-offset NED -> lat/lon (degrees)."""
    radius = 6378137.0
    lat = lat0 + math.degrees(dn / radius)
    lon = lon0 + math.degrees(de / (radius * math.cos(math.radians(lat0))))
    return lat, lon
