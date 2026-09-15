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

"""High-level guidance laws for the PX4 drone. Pure functions, no I/O.

YAW_TRACK: unwind the gimbal toward the airframe centre-line by yawing the vehicle.
FOLLOW: hold a standoff distance and altitude to the estimated target with velocity
feed-forward. Neither ever commands from raw pixel error.

Ported from drone-autonomy ``common/guidance.py`` (flown 2026-09-09). The maths is
byte-identical; the config dicts became the dataclasses in ``config.py``.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

from dimos.robot.px4.config import FollowConfig, YawTrackConfig
from dimos.utils.angles import clamp, wrap180


@dataclass
class TargetEstimate:
    """One ``target_state_v1`` packet as the supervisor sees it (NED, metres, m/s)."""

    valid: bool = False
    n: float | None = None
    e: float | None = None
    vn: float = 0.0
    ve: float = 0.0
    los_valid: bool = False
    gimbal_yaw_body_deg: float | None = None
    track_id: int | None = None
    range_m: float | None = None
    bearing_deg: float | None = None
    reason: str = ""


@dataclass(frozen=True)
class FollowCommand:
    vn: float
    ve: float
    vd: float
    yaw_deg: float
    range_m: float
    bearing_deg: float


def yaw_track_rate(
    gimbal_yaw_body_deg: float | None, target_visible: bool, cfg: YawTrackConfig
) -> float:
    """deg/s of vehicle yaw to command. Positive = clockwise (right), toward the gimbal."""
    if not target_visible or gimbal_yaw_body_deg is None:
        return 0.0
    g = gimbal_yaw_body_deg
    db = cfg.deadband_deg
    if abs(g) <= db:
        return 0.0
    err = g - math.copysign(db, g)
    return clamp(cfg.k_yaw * err, -cfg.max_yaw_rate_dps, cfg.max_yaw_rate_dps)


def follow_velocity(
    veh_n: float,
    veh_e: float,
    veh_d: float,
    d_takeoff: float,
    target: TargetEstimate,
    cfg: FollowConfig,
) -> FollowCommand:
    """Velocity + yaw command to keep standoff geometry to the target."""
    assert target.n is not None and target.e is not None, "follow_velocity needs a position"
    dn, de = target.n - veh_n, target.e - veh_e
    rng = math.hypot(dn, de)
    bearing = (math.degrees(math.atan2(de, dn)) + 360.0) % 360.0
    if rng < 1e-3:
        un, ue = 1.0, 0.0
    else:
        un, ue = dn / rng, de / rng
    err = rng - cfg.standoff_m
    if abs(err) <= cfg.range_deadband_m:
        v_rad = 0.0
    else:
        v_rad = cfg.k_range * (err - math.copysign(cfg.range_deadband_m, err))
    vn = v_rad * un + cfg.ff_gain * target.vn
    ve = v_rad * ue + cfg.ff_gain * target.ve
    speed = math.hypot(vn, ve)
    if speed > cfg.v_max_mps:
        vn, ve = vn * cfg.v_max_mps / speed, ve * cfg.v_max_mps / speed
    d_des = d_takeoff - cfg.altitude_m
    vd = clamp(cfg.k_alt * (d_des - veh_d), -cfg.vz_max_mps, cfg.vz_max_mps)
    return FollowCommand(vn=vn, ve=ve, vd=vd, yaw_deg=bearing, range_m=rng, bearing_deg=bearing)


def rate_limit_yaw(current_deg: float, desired_deg: float, max_rate_dps: float, dt: float) -> float:
    step = clamp(wrap180(desired_deg - current_deg), -max_rate_dps * dt, max_rate_dps * dt)
    return wrap180(current_deg + step)
