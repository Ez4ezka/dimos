# PX4 drone

dimOS modules for a PX4 quadcopter: Holybro X500, Pixhawk 6C on PX4 1.17, Jetson Orin
Nano companion, SIYI A8 mini gimbal camera, Quectel 5G modem. The flight logic is the
code that flew on 2026-09-09; the plumbing is dimOS.

## How it works

`Px4DroneConnection` is the only process that talks to PX4. It opens one MAVLink socket,
publishes the vehicle as dimOS streams (odometry, imu, gps, battery, rc, gimbal attitude,
status) and runs the Offboard flight supervisor: a state machine that streams setpoints at
20 Hz through takeoff, hover, yaw-track, follow, teleop, go-to and landing. Operator commands
are RPCs. There is no arm, mode or raw setpoint RPC.

Everything else is a separate module that reads or feeds the connection's streams by name.

| Module | Required | What it does | Reads | Writes |
|---|---|---|---|---|
| `Px4DroneConnection` | yes | MAVLink bridge + flight supervisor | cmd_vel, estop_in, target_state, target_valid, target_los, gimbal_target | odometry, odom, tf, imu, gps, global_pose, battery, rc, motor_outputs, gimbal_attitude, vehicle_status, statustext, supervisor_state, command_event, offboard_setpoint, stop_movement |
| `RtspCamera` | no | A8 H.265 stream in, `video`, decoded `color_image`, small `color_jpeg` out | link_policy | video, color_image, color_jpeg |
| `SiyiA8Gimbal` | no | gimbal tf chain, camera intrinsics, aim requests | gimbal_attitude, target_los | tf, camera_info, gimbal_target |
| `PerceptionBridge` | no | detector, tracker, line of sight, target position | color_image, odometry, gimbal_attitude, global_pose, vehicle_status, track_select | tracks, target_state, target_valid, target_los |
| `LinkMonitor` | no | 5G link quality and the video/telemetry policy | nothing | link_status, link_policy |
| `CommandTracker` | no | scores every operator command: did it take effect, if not why | command_event, offboard_setpoint, odometry, vehicle_status, supervisor_state | tracked_command, command_report, cmd_forward, meas_forward |
| `Px4SkillContainer` | no | the flight commands as agent skills, each waiting for its outcome | nothing (calls the connection's RPCs) | nothing |
| `FakeA8` | SITL only | stands in for the gimbal hardware | gimbal_target | gimbal_attitude |

A missing optional module never stops anything from starting. You lose what it provided:

- No camera: no video, perception gets no frames, so no target. FOLLOW and YAW_TRACK
  hold position.
- No gimbal module: no gimbal tf chain, no camera_info, nothing aims the gimbal.
- No perception: no target. Same as no camera for FOLLOW and YAW_TRACK.
- No link monitor: the camera keeps its configured rates.
- No tracker: no command scoring.

## Files

```
dimos/robot/px4/
  config.py            numbers: MAVLink ids, router ports, flight limits, guidance gains
  mavlink.py           MAVLink layer: PX4 modes, NED/FLU frames, timebase, vehicle state, the socket
  supervisor_core.py   flight state machine and guidance laws, pure logic, no socket
  connection.py        Px4DroneConnection, the module
  connection_spec.py   the connection RPCs the skills call
  skill_container.py   Px4SkillContainer and the agent's system prompt
  command_tracker.py   CommandTracker
  link_monitor.py      LinkMonitor
  perception/          PerceptionBridge (bridge.py) plus detector, tracker, geometry, estimators
  sitl.py              FakeA8
  blueprints.py        px4-basic, px4-drone, px4-sitl, px4-teleop, px4-sitl-teleop
  blueprints_agentic.py  px4-agentic, px4-sitl-agentic (needs the `agents` extra)
  tool_*_gate.py       gates, print PASS or FAIL with numbers
dimos/hardware/sensors/camera/rtsp/   RtspCamera
dimos/hardware/gimbal/siyi/           SiyiA8Gimbal, frame maths, SIYI SDK
dimos/msgs/px4_msgs/, dimos/msgs/link_msgs/   typed messages
```

## Run

```bash
dimos run px4-basic    # connection + viewer, on the Jetson
dimos run px4-drone    # everything on the aircraft
dimos run px4-teleop   # px4-drone + the viewer's keyboard
dimos run px4-agentic  # px4-teleop + the skills, the MCP server and the LLM agent
dimos run px4-sitl     # px4-drone against PX4 SITL, with synthetic camera, fake gimbal, replayed link
```

`px4-sitl`, `px4-sitl-teleop` and `px4-sitl-agentic` need `make px4_sitl gz_x500` running in
a PX4 tree, and a ground station on 14550 (QGC) or PX4 refuses to arm. Each module also runs
alone (`dimos run px4-drone-connection`, `command-tracker`, `rtsp-camera`, `siyi-a8-gimbal`,
`link-monitor`, `perception-bridge`, `fake-a8`) and binds to whatever else is running.
`dimos --record sqlite run <blueprint>` keeps every stream of the run in
`recordings/<run-id>/memory.db`.

### Commands

From `dimos shell`, in any blueprint:

```
drone = app.Px4DroneConnection
drone.sitl_enable(True)                  # SITL only, fakes the RC enable switch
drone.takeoff(2.0)                       # metres above the ground; no argument = limits.takeoff_alt_m
drone.go_to(north_m=-2, altitude_m=3)    # 2 m south of here, at 3 m above the takeoff point
drone.go_to(relative=False, heading_deg=90)   # back over the takeoff point, facing east
drone.set_guidance_mode("TELEOP")        # HOVER, YAW_TRACK, FOLLOW, TELEOP
drone.land()
drone.estop()
drone.status()
app.CommandTracker.recent()
app.PerceptionBridge.select_track(1)
app.LinkMonitor.status()
```

A go-to flies at walking pace (`GotoConfig`), ends in HOVER at the goal, and is refused
unless the goal is `goal_margin_m` inside the fence and the ceiling (`SupervisorLimits`).
`set_guidance_mode("HOVER")` stops one on the spot.

### Keyboard

`px4-teleop` wires the dimos-viewer's keyboard to `cmd_vel` the way `r1pro-teleop` does.
On the Jetson run `dimos --rerun-open none --rerun-host 0.0.0.0 run px4-teleop` (the
viewer servers listen on localhost otherwise) and connect the viewer from the laptop with
the `dimos-viewer --connect ... --ws-url ...` line it logs. Then take off, select TELEOP,
click the keyboard overlay in the viewer and fly: W/S forward and back, Q/E strafe, A/D
turn, Shift faster, Space stop. Keys do nothing outside TELEOP (the tracker shows them as
`rejected/not_teleop`), speeds are clamped to the teleop limits, the altitude stays where
TELEOP started, and when the keys stop arriving the vehicle holds position.

### Agent

`px4-agentic` adds `Px4SkillContainer` (takeoff, go_to, land, set_guidance_mode,
flight_status), the MCP server and the LLM agent, so the same commands work in words.
It needs `uv sync --extra agents` and `OPENAI_API_KEY`:

```bash
dimos agent-send "take off to 2 meters"
dimos agent-send "go to 2 meters south at 3 m altitude"
dimos mcp call go_to --arg north_m=-2 --arg altitude_m=3    # the same skill, no LLM
```

A skill is a connection RPC plus a wait for the outcome. It adds no authority: the
supervisor refuses a skill exactly as it refuses the RPC.

## Test

No hardware, no simulator:

```bash
uv sync --extra px4
uv run pytest dimos/robot/px4 dimos/hardware/gimbal/siyi dimos/hardware/sensors/camera/rtsp \
    dimos/msgs/px4_msgs dimos/msgs/link_msgs
uv run python dimos/robot/px4/tool_link_gate.py       # recorded link scenarios
uv run python dimos/robot/px4/tool_tracker_gate.py    # scripted commands through zenoh
```

With PX4 SITL (`make px4_sitl gz_x500` in the PX4 tree):

```bash
uv run python dimos/robot/px4/tool_sitl_gate.py --fly
```

This runs `px4-sitl` and checks odometry rate and stamps, the gimbal tf chain, frame
stamps, track confirmation and line of sight, the link policy, then flies the operator
commands: takeoff to 2 m, go 2 m south at 3 m, a go-to past the fence refused, a held
teleop key, land, and the tracker's verdict on each.

## Aircraft setup

On the Jetson, add a mavlink-router endpoint for this stack and restart the router:

```ini
[UdpEndpoint dimos]
Mode = Normal
Address = 127.0.0.1
Port = 14556
```

Ports 14550 to 14554 belong to the other services (see `ROUTER_ENDPOINTS` in
`config.py`).

PX4 parameters, set in QGC, then reboot: `COM_OBL_RC_ACT=5`, `COM_OF_LOSS_T=0.5`,
`COM_RC_OVERRIDE=3`, `NAV_RCL_ACT=2`, `COM_RC_LOSS_T=0.5`, `GF_ACTION=2`,
`GF_MAX_HOR_DIST=50`, `GF_MAX_VER_DIST=30`, `COM_DISARM_LAND=2`, `MPC_XY_VEL_MAX=3`,
`MPC_Z_VEL_MAX_UP=1.5`, `MPC_Z_VEL_MAX_DN=1.0`, `COM_ARM_WO_GPS=0`, `COM_HOME_EN=1`.
Check the names against PX4 1.17 first; they were not set as of 2026-09-09.

The flown `flight_supervisor.py` must not run at the same time; the connection holds its
UDP port 5610 and refuses to start while it is taken. `gimbal_mount_xyz` is a placeholder
until measured.

## Safety rules

1. The RC pilot always wins. If PX4 leaves OFFBOARD the software stops sending and
   never sends a mode command.
2. One writer of setpoints: the connection's tick thread. If it stops, PX4's Offboard-loss
   failsafe holds.
3. E-STOP is Hold plus a latch. `estop_clear` only works in IDLE.
4. Gimbal aim commands only go out when `gimbal_commands_enabled` is set on the
   connection.
5. Operator numbers are checked before anything moves: a takeoff altitude or a go-to goal
   outside the fence, the ceiling or `min_alt_m` is refused, and GOTO obeys the same abort
   rules as every other armed state.
