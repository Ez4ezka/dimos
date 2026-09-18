# PX4 drone

Fly a PX4 quadcopter from dimOS over MAVLink Offboard.

Status: passes PX4 SITL (v1.17). Bench-tested on a Pixhawk 6C with props off. Not flown
outdoors yet.

## How it works

- `Px4DroneConnection` is the only module that talks to PX4, over one MAVLink UDP socket.
- It publishes the vehicle as streams: odometry, imu, gps, battery, rc, gimbal attitude,
  status.
- It runs the flight supervisor, a 20 Hz state machine that streams Offboard setpoints:
  `IDLE > PREFLIGHT > STREAMING > OFFBOARD_REQ > ARMING > TAKEOFF > HOVER > LANDING > IDLE`.
  From `HOVER` the operator selects `YAW_TRACK`, `FOLLOW`, `TELEOP` or `GOTO`.
- Commands are RPCs on the connection. There is no arm, mode or raw-setpoint RPC.
- Every other module is optional and binds to these streams by name. Without one you lose
  what it provides; everything else still starts.

| Module | Does | Reads | Writes |
|---|---|---|---|
| `Px4DroneConnection` | MAVLink bridge, flight supervisor | cmd_vel, estop_in, target_state, target_valid, target_los, gimbal_target | odometry, odom, tf, imu, gps, global_pose, battery, rc, motor_outputs, gimbal_attitude, vehicle_status, statustext, supervisor_state, command_event, offboard_setpoint, stop_movement |
| `CommandTracker` | Scores each command: did the vehicle do it, how late | command_event, offboard_setpoint, odometry, vehicle_status, supervisor_state | tracked_command, command_report, cmd_forward, meas_forward |
| `RtspCamera` | H.265 stream in; passthrough video, decoded frames, small JPEG out | link_policy | video, color_image, color_jpeg |
| `SiyiA8Gimbal` | Gimbal tf chain, camera intrinsics, aim requests | gimbal_attitude, target_los | tf, camera_info, gimbal_target |
| `LinkMonitor` | Measures the operator link, sets video and telemetry rates | none | link_status, link_policy |
| `PerceptionBridge` | Detector, tracker, line of sight, target position | color_image, odometry, gimbal_attitude, global_pose, vehicle_status, track_select | tracks, target_state, target_valid, target_los |
| `FakeA8` | SITL only: answers as the gimbal | gimbal_target | gimbal_attitude |

`YAW_TRACK` and `FOLLOW` need the target from `PerceptionBridge`, which needs the camera.
Without a target they hold position.

## Files

```
dimos/robot/px4/
  config.py              MAVLink ids, ports, flight limits, guidance gains
  mavlink.py             socket, vehicle state, PX4 modes, NED/FLU frames, timebase
  supervisor_core.py     flight state machine and guidance laws; no I/O
  connection.py          Px4DroneConnection
  command_tracker.py     CommandTracker
  link_monitor.py        LinkMonitor
  perception/            PerceptionBridge: detector, tracker, geometry, estimators
  sitl.py                FakeA8
  blueprints.py          px4-basic, px4-drone, px4-sitl, px4-teleop, px4-sitl-teleop
  tool_*_gate.py         gates: print the numbers, end with GATE PASS or GATE FAIL
dimos/hardware/sensors/camera/rtsp/   RtspCamera
dimos/hardware/gimbal/siyi/           SiyiA8Gimbal, frame maths, SIYI SDK
dimos/msgs/px4_msgs/, link_msgs/      typed messages
```

## Install

```bash
uv sync --extra px4
```

## Run

| Blueprint | Runs |
|---|---|
| `px4-basic` | connection, viewer |
| `px4-drone` | connection, tracker, camera, gimbal, link monitor, perception, viewer |
| `px4-teleop` | `px4-drone` with the viewer's keyboard on `cmd_vel` |
| `px4-sitl`, `px4-sitl-teleop` | the same against PX4 SITL: synthetic camera, fake gimbal, replayed link |

```bash
dimos run px4-drone
dimos --record sqlite run px4-drone                          # every stream to recordings/<run-id>/memory.db
dimos --rerun-open none --rerun-host 0.0.0.0 run px4-drone   # on the aircraft, viewer on a laptop
```

With the last form, run the `dimos-viewer --connect ...` line it logs on the laptop.

Each module also runs alone and binds to whatever else is running:
`dimos run px4-drone-connection`, `command-tracker`, `rtsp-camera`, `siyi-a8-gimbal`,
`link-monitor`, `perception-bridge`, `fake-a8`.

## Commands

From `dimos shell`:

```python
drone = app.Px4DroneConnection
drone.sitl_enable(True)                      # SITL only: stands in for the RC enable switch
drone.takeoff(2.0)                           # metres above ground; default 3.0
drone.go_to(north_m=-2, altitude_m=3)        # 2 m south of here, 3 m above the takeoff point
drone.go_to(relative=False, heading_deg=90)  # back over the takeoff point, facing east
drone.set_guidance_mode("TELEOP")            # HOVER, YAW_TRACK, FOLLOW, TELEOP
drone.land()
drone.estop()                                # Hold and latch; estop_clear() works in IDLE
drone.status()
app.CommandTracker.recent()
app.PerceptionBridge.select_track(1)
app.LinkMonitor.status()
```

- Every command returns `{"accepted": bool, "rejection": str | None, "state": str}`.
- `takeoff` runs preflight first: enable switch on, 3D GPS fix with eph under 1.5 m, valid
  position estimate, battery at 40 % or more, disarmed, on the ground.
- `go_to` flies at 1 m/s or less, ends in `HOVER`, and gives up into `HOVER` after 60 s.
  It is refused unless the goal is 2 m inside the fence (30 m) and the ceiling (15 m).
  `set_guidance_mode("HOVER")` stops it.
- Limits are in `config.py` (`SupervisorLimits`, `GotoConfig`).

## Keyboard

1. `dimos run px4-teleop` (or `px4-sitl-teleop`).
2. `drone.takeoff(2.0)`, then `drone.set_guidance_mode("TELEOP")`.
3. Click the keyboard overlay in the viewer.
4. W/S forward and back, Q/E strafe, A/D turn, Shift faster, Space stop.

Keys are ignored outside `TELEOP`. Speeds are clamped to 1.5 m/s and 0.8 rad/s. Altitude
stays where `TELEOP` began. When keys stop for 0.5 s the vehicle holds position.

## Test

### 1. Unit tests

No hardware, no simulator.

```bash
uv sync --extra px4
uv run pytest dimos/robot/px4 dimos/hardware/gimbal/siyi dimos/hardware/sensors/camera/rtsp \
    dimos/msgs/px4_msgs dimos/msgs/link_msgs
```

### 2. Gates without a simulator

```bash
uv run python dimos/robot/px4/tool_tracker_gate.py    # scripted commands through zenoh
uv run python dimos/robot/px4/tool_link_gate.py       # recorded link scenarios
```

Each ends with `GATE PASS`.

### 3. SITL gate

1. Close QGroundControl. The gate binds UDP 14550 to send the ground-station heartbeat PX4
   needs before it arms.
2. In a PX4-Autopilot v1.17 checkout: `HEADLESS=1 make px4_sitl gz_x500`. Wait for
   `Startup script returned successfully`.
3. `uv run python dimos/robot/px4/tool_sitl_gate.py --fly`
4. The last line must be `GATE PASS`.

It runs `px4-sitl` and checks:

- odometry at 25 Hz or more, stamps within 50 ms
- the gimbal chain in `tf`, the gimbal module reporting the fake A8's attitude
- video frames stamped within 50 ms of the vehicle clock
- a track confirmed, a valid target after selection, line of sight within 2 deg
- the gimbal aimed at the target
- the link policy allows video
- the flight: takeoff to 2 m, go 2 m south at 3 m, a go-to past the fence refused, a held
  key moving the vehicle at a locked altitude, land
- the tracker scoring each of those commands

### 4. Fly SITL by hand

1. `make px4_sitl gz_x500` in the PX4 checkout.
2. Open QGroundControl. PX4 will not arm without a ground station.
3. `dimos run px4-sitl-teleop`
4. `dimos shell`, then the commands above, starting with `drone.sitl_enable(True)`.

## Aircraft setup

1. Add a mavlink-router endpoint on the companion computer and restart the router:

   ```ini
   [UdpEndpoint dimos]
   Mode = Normal
   Address = 127.0.0.1
   Port = 14556
   ```

2. Set these PX4 parameters in QGC and reboot. Check the names against your PX4 version;
   they have not been verified on a vehicle yet.

   `COM_OBL_RC_ACT=5`, `COM_OF_LOSS_T=0.5`, `COM_RC_OVERRIDE=3`, `NAV_RCL_ACT=2`,
   `COM_RC_LOSS_T=0.5`, `GF_ACTION=2`, `GF_MAX_HOR_DIST=50`, `GF_MAX_VER_DIST=30`,
   `COM_DISARM_LAND=2`, `MPC_XY_VEL_MAX=3`, `MPC_Z_VEL_MAX_UP=1.5`, `MPC_Z_VEL_MAX_DN=1.0`,
   `COM_ARM_WO_GPS=0`, `COM_HOME_EN=1`.

3. Put the enable switch on RC channel 7 (`SupervisorLimits.enable_channel`). High allows
   the software to fly. Low refuses takeoff and, in flight, aborts to Hold.
4. Leave the RC arm switch off. The supervisor arms over MAVLink and refuses if the vehicle
   is already armed.
5. Measure the gimbal mount offset and set `gimbal_mount_xyz`. The default is a placeholder
   and the module warns at start.
6. Bench test with props off before flying: refusal with the enable switch low, a full
   takeoff-to-land sequence with it high, and pilot takeover by moving the mode switch.

Only one Offboard writer may run. The connection binds UDP 5610 as a lock and refuses to
start while another process holds it.

## Safety rules

1. The RC pilot always wins. When PX4 leaves Offboard the supervisor stops sending, goes to
   `IDLE` (`PILOT_OVERRIDE`) and never sends a mode command.
2. One writer of setpoints: the connection's tick thread. If it stops, PX4's Offboard-loss
   failsafe takes over.
3. E-STOP is Hold plus a latch. `estop_clear` only works in `IDLE`.
4. Gimbal aim commands go out only when `gimbal_commands_enabled` is set on the connection.
5. Takeoff altitudes and go-to goals are checked against the fence, the ceiling and
   `min_alt_m` before anything moves. `GOTO` obeys the same abort rules as every armed
   state.
