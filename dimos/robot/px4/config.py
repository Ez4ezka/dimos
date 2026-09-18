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

"""Hardware constants and flight limits (module wiring lives in ``connection.py``).

Everything here was measured or flown on the aircraft; the module config in
``connection.py`` takes its defaults from this file so the numbers exist once.
"""

from __future__ import annotations

from dataclasses import dataclass, field

# MAVLink identities on the vehicle bus. PX4 is 1/1; the SIYI A8 answers as 1/154 through
# PX4's second MAVLink instance; the flown onboard services sit on 191..194.
PX4_SYSID = 1
PX4_COMPID = 1
A8_COMPID = 154
DIMOS_COMPID = 195

# mavlink-routerd endpoints on the Jetson (config/mavlink-router.conf of the flown stack).
# Every service owns one; never reuse another service's port.
ROUTER_ENDPOINTS: dict[int, str] = {
    14550: "gimbal controller (191)",
    14551: "line-of-sight estimator (192)",
    14552: "flown flight_supervisor.py (193)",
    14553: "logger / sniffer (194)",
    14554: "QGroundControl throttle",
    14556: "dimOS Px4DroneConnection (195)",
}
ROUTER_MAV_URL = "udpin:127.0.0.1:14556"
# PX4 SITL (`make px4_sitl gz_x500`) sends its onboard-computer stream here instead.
SITL_MAV_URL = "udpin:0.0.0.0:14540"

# The flown supervisor binds this UDP port for target packets. Px4DroneConnection holds
# it as a writer lock so the two can never stream setpoints at once.
LEGACY_SUPERVISOR_LOCK_PORT = 5610

# base_link -> gimbal_base, metres, FLU. UNMEASURED placeholder: the module warns at start
# until a tape-measured value replaces it.
GIMBAL_MOUNT_XYZ_UNMEASURED = (0.0, 0.0, -0.08)


@dataclass(frozen=True)
class Px4Hardware:
    """Bus identity and stream rates of the connection."""

    mav_url: str = ROUTER_MAV_URL
    source_system: int = PX4_SYSID
    source_component: int = DIMOS_COMPID
    target_system: int = PX4_SYSID
    target_component: int = PX4_COMPID
    gimbal_component: int = A8_COMPID
    connect_timeout_s: float = 30.0
    heartbeat_hz: float = 1.0
    ack_timeout_s: float = 3.0
    # Output rates. PX4 streams LOCAL_POSITION_NED at 30 Hz and HIGHRES_IMU at 50 Hz on the
    # Onboard profile; the rest are as fast as anyone downstream needs.
    odom_hz: float = 30.0
    imu_hz: float = 50.0
    motor_outputs_hz: float = 10.0
    rc_hz: float = 5.0
    status_hz: float = 5.0
    gps_hz: float = 5.0
    battery_hz: float = 1.0
    gimbal_hz: float = 10.0
    statustext_hz: float = 10.0
    robot_state_hz: float = 2.0
    tick_hz: float = 20.0
    gimbal_mount_preset: str = "flight"
    gimbal_mount_xyz: tuple[float, float, float] = GIMBAL_MOUNT_XYZ_UNMEASURED
    # Aim commands to the A8 go at most this fast; it reports attitude at 10 Hz.
    gimbal_command_hz: float = 10.0


PX4_HARDWARE = Px4Hardware()


@dataclass(frozen=True)
class YawTrackConfig:
    """``config/follow.json`` ``guidance.yaw_track`` of the flown stack."""

    deadband_deg: float = 15.0
    k_yaw: float = 0.6
    max_yaw_rate_dps: float = 30.0


@dataclass(frozen=True)
class FollowConfig:
    """``config/follow.json`` ``guidance.follow`` of the flown stack."""

    standoff_m: float = 12.0
    altitude_m: float = 10.0
    k_range: float = 0.4
    k_alt: float = 0.6
    v_max_mps: float = 2.0
    vz_max_mps: float = 0.7
    ff_gain: float = 0.8
    range_deadband_m: float = 1.5
    loss_hold_s: float = 1.0
    loss_hover_s: float = 5.0


@dataclass(frozen=True)
class GotoConfig:
    """Operator go-to. Not flown yet: walking-pace caps, the same altitude law as FOLLOW."""

    k_pos: float = 0.6
    k_alt: float = 0.6
    v_max_mps: float = 1.0
    vz_max_mps: float = 0.7
    yaw_tolerance_deg: float = 5.0
    # A goal never reached (wind, a PX4 limit) ends in HOVER where the vehicle is. The fence
    # keeps every goal under 30 s of flight at v_max_mps.
    timeout_s: float = 60.0


@dataclass(frozen=True)
class GuidanceConfig:
    yaw_track: YawTrackConfig = field(default_factory=YawTrackConfig)
    follow: FollowConfig = field(default_factory=FollowConfig)
    goto: GotoConfig = field(default_factory=GotoConfig)


@dataclass(frozen=True)
class SupervisorLimits:
    """``config/flight.json`` of the flown supervisor plus the teleop caps.

    Conservative first-flight values; raise only after each flight gate passes.
    """

    takeoff_alt_m: float = 3.0
    climb_rate_mps: float = 0.7
    max_alt_m: float = 15.0
    geofence_radius_m: float = 30.0
    # Operator-chosen altitudes and go-to goals: no lower than min_alt_m, and this far
    # inside the ceiling and the fence so an overshoot never trips the abort rule.
    min_alt_m: float = 1.0
    goal_margin_m: float = 2.0
    min_batt_pct: int = 40
    min_fix_type: int = 3
    max_eph_m: float = 1.5
    # RC enable switch: TX16S channel 7, high above 1500 us.
    enable_channel: int = 7
    enable_threshold_us: int = 1500
    rc_stale_s: float = 1.0
    px4_stale_s: float = 1.0
    target_stale_s: float = 1.0
    setpoint_hz: float = 20.0
    prestream_s: float = 1.5
    hover_tolerance_m: float = 0.5
    hover_settle_s: float = 3.0
    ack_timeout_s: float = 3.0
    teleop_v_xy_mps: float = 1.5
    teleop_v_z_mps: float = 0.7
    teleop_yaw_rate_rps: float = 0.8
    teleop_stale_s: float = 0.5
    # Locked, TELEOP holds the altitude it started at (by teleop_k_alt) and ignores the
    # up axis; the viewer's keyboard has no up or down key.
    teleop_lock_altitude: bool = True
    teleop_k_alt: float = 0.6
