# PX4 drone (dimosdrone-2)

Holybro X500, Pixhawk 6C on PX4 v1.17.0 (custom CRSF build), Jetson Orin Nano
companion (JetPack 6), SIYI A8 mini gimbal camera on a hanging mount, RTK GNSS, ELRS
from a RadioMaster TX16S, Quectel RM520N-GL 5G modem. Everything flight-critical runs
on the Jetson; the operator link is never in the control loop.

This package ports the bespoke stack that flew on 2026-09-09 (`~/drone-autonomy`)
into dimOS modules. The flight logic is byte-for-byte the flown code; only the
plumbing changed.

## Modules

| Module | File | Role |
|---|---|---|
| `Px4Drone` | `px4_drone.py` | The MAVLink connection and the Offboard flight supervisor in one process. Publishes odometry, IMU, GPS, battery, RC, gimbal attitude and status; runs the takeoff / hover / yaw-track / follow / teleop / land state machine at 20 Hz. |
| `FakeTarget` | `sitl/fake_target.py` | Scripted target (line, circle, drop windows) for SITL FOLLOW and YAW_TRACK tests. |

Inside `Px4Drone` the classes stay separate: `mavlink/io.py` owns the socket and the
reader thread, `mavlink/vehicle_state.py` keeps the timed telemetry buffers,
`mavlink/timebase.py` converts vehicle boot time to UTC, `supervisor_core.py` is the
pure state machine (no socket, no port, no pymavlink) and `guidance.py` holds the
yaw-track and follow maths.

## Safety invariants

1. The RC pilot always wins. If PX4 is not in OFFBOARD the software stands down
   silently and sends no mode command. Pilot escape: flight-mode switch to Position or
   Hold, or any stick past `COM_RC_STICK_OV`.
2. Exactly one thing produces Offboard setpoints: the `Px4Drone` tick thread, through
   `MavlinkIO`. There is no arm, mode or setpoint RPC (`test_px4_drone.py` asserts it).
3. If the tick stops, the stream stops and PX4's Offboard-loss failsafe (`COM_OBL_RC_ACT`,
   `COM_OF_LOSS_T`) puts the aircraft in Hold. Never add a fallback that keeps commanding.
4. E-STOP is Hold plus a latch. Nothing moves the aircraft until `estop_clear`, which
   only works in IDLE.
5. `Px4Drone` refuses to start while the flown `flight_supervisor.py` holds its UDP
   port (5610), so two writers can never coexist.

## Running

```bash
# SITL on the laptop: PX4 v1.16 tree, `make px4_sitl gz_x500` in another terminal.
dimos run px4-sitl
dimos run px4-sitl-follow      # adds a scripted target
dimos shell                    # then: px4_drone.sitl_enable(True); px4_drone.takeoff()
```

On the Jetson `mavlink-routerd` owns the UART (`/dev/ttyTHS1`, 921600) and fans out
one UDP endpoint per service. Add `14556` for this module (component 195) next to the
existing 14550 gimbal (191), 14551 line-of-sight (192), 14552 old supervisor (193),
14553 logger, 14554 QGroundControl throttle. Do not reuse any of those.

PX4 parameters for companion Offboard flight (set in QGroundControl, then reboot):
`COM_OBL_RC_ACT=5` (Hold on setpoint loss), `COM_OF_LOSS_T=0.5`, `COM_RC_OVERRIDE=3`,
`NAV_RCL_ACT=2`, `COM_RC_LOSS_T=0.5`, `GF_ACTION=2`, `GF_MAX_HOR_DIST=50`,
`GF_MAX_VER_DIST=30`, `COM_DISARM_LAND=2`, `MPC_XY_VEL_MAX=3`, `MPC_Z_VEL_MAX_UP=1.5`,
`MPC_Z_VEL_MAX_DN=1.0`, `COM_ARM_WO_GPS=0`, `COM_HOME_EN=1`. These were not yet set on
the vehicle as of 2026-09-09; verify the failsafe parameter names against v1.17 first.

## Flight gates

Each must pass before the next, props off first, then a pilot present.

1. Bench, props off: `takeoff` with the enable switch (channel 7) off stays in
   PREFLIGHT with reason `enable switch off`, returns to IDLE after 10 s, sends zero
   setpoints and no arm command.
2. Pilot Position-mode hover, gimbal tracking only.
3. `takeoff` to 3 m hover, `land`. Check altitude error, setpoint rate, no ABORT.
4. `set_guidance_mode("YAW_TRACK")`: the airframe yaws to keep the gimbal within 15 deg.
5. `set_guidance_mode("FOLLOW")`: standoff 12 m at 10 m altitude, 2 m/s cap.

## Frames and conventions

dimOS is FLU (x forward/north, y left/west, z up). MAVLink LOCAL_NED is converted with
`mavlink/frames.py`, the same signs as `dimos/robot/drone/mavlink_connection.py`. The
gimbal reports body-relative yaw and earth-stabilised pitch (verified 2026-09-04);
`gimbal_attitude` carries radians in `position` and the MAVLink flag bits in `effort`.
`rc` carries raw microseconds in `axes` so the 1500 us enable threshold keeps meaning.
`offboard_setpoint` is stamped when the datagram left the process, not when published.

## Network reality (measured 2026-09-09)

Wi-Fi direct 7 ms. Both ends on cellular through the Tailscale relay: 86 ms average
with throttled telemetry, 413 ms and 5 % loss with raw telemetry, and no headroom for
video on that cell. Nothing flight-critical crosses the link.
