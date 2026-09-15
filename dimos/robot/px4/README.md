# PX4 drone (dimosdrone-2)

Holybro X500 on a Pixhawk 6C running PX4 v1.17.0 (custom CRSF build), Jetson Orin Nano
companion (JetPack 6), SIYI A8 mini gimbal camera on a hanging mount, RTK GNSS, ELRS
from a RadioMaster TX16S, Quectel RM520N-GL 5G modem. Everything flight-critical runs on
the Jetson; the operator link is never in the control loop.

This package ports the bespoke stack that flew on 2026-09-09 (`~/drone-autonomy`) into
dimOS modules. The flight logic is the flown code; only the plumbing changed.

## Aircraft-side setup

Everything in this section runs **on the Jetson**, over ssh (`ezendimos@100.110.224.46`),
not on your workstation.

`mavlink-routerd` owns the Pixhawk UART (`/dev/ttyTHS1`, 921600) and fans out one UDP
endpoint per service. Add the dimOS endpoint next to the existing ones in
`/etc/mavlink-router/main.conf` (or `~/autonomy/config/mavlink-router.conf` if the router
is still started by hand), then restart the router:

```ini
[UdpEndpoint dimos]
Mode = Normal
Address = 127.0.0.1
Port = 14556
```

The full endpoint table is `ROUTER_ENDPOINTS` in `config.py`: 14550 gimbal controller
(component 191), 14551 line of sight (192), 14552 the flown supervisor (193), 14553 logger,
14554 QGroundControl throttle, 14556 this package (195). Never reuse another service's port.

PX4 parameters for companion Offboard flight, set in QGroundControl then reboot:
`COM_OBL_RC_ACT=5` (Hold on setpoint loss), `COM_OF_LOSS_T=0.5`, `COM_RC_OVERRIDE=3`,
`NAV_RCL_ACT=2`, `COM_RC_LOSS_T=0.5`, `GF_ACTION=2`, `GF_MAX_HOR_DIST=50`,
`GF_MAX_VER_DIST=30`, `COM_DISARM_LAND=2`, `MPC_XY_VEL_MAX=3`, `MPC_Z_VEL_MAX_UP=1.5`,
`MPC_Z_VEL_MAX_DN=1.0`, `COM_ARM_WO_GPS=0`, `COM_HOME_EN=1`. Not yet set on the vehicle as
of 2026-09-09; verify the failsafe parameter names against v1.17 first. Keep the existing
`MAV_1_CONFIG=102 MAV_1_MODE=2 MAV_1_FORWARD=1 SER_TEL2_BAUD=921600`, the `MAV_2_*` set
for the A8, and `MNT_MODE_IN=4 MNT_MODE_OUT=2`.

The flown `flight_supervisor.py` and `Px4DroneConnection` must never run together.
The connection holds the supervisor's UDP port 5610 as a lock and refuses to start
while it is taken.

## Environment

- Install with the `px4` extra (pymavlink, PyAV, pyserial):
  `uv sync --extra px4` or, for everything, `uv sync --extra all`.
- On the Jetson use the system Python 3.10 the way the R1 Pro README describes
  (`uv sync --python /usr/bin/python3.10 --python-preference only-system --extra px4`).
  The TensorRT detector for the perception bridge only exists there.
- PX4 SITL on a workstation: the PX4 v1.16 tree with `make px4_sitl gz_x500`. PX4 refuses
  to arm without a ground station; `tool_sitl_gate.py` runs a stand-in heartbeat on 14550,
  and in the field that role is QGroundControl.
- The gimbal mount offset (`gimbal_mount_xyz`) is an UNMEASURED placeholder and the
  connection warns at start until a measured value replaces it.

## Blueprints

```bash
dimos run px4-basic             # connection + viewer, on the Jetson against endpoint 14556
dimos run px4-sitl              # same against PX4 SITL (`make px4_sitl gz_x500`)
dimos run px4-sitl-follow       # + a scripted target for FOLLOW and YAW_TRACK
dimos run px4-drone-connection  # the connection alone, no viewer
dimos run px4-sitl-tracked      # px4-sitl + CommandTracker scoring every command
dimos run px4-bench             # aircraft, props off: + A8 video, gimbal chain, tracker
dimos run px4-sitl-bench        # the bench stack against SITL with a replayed clip and a fake A8
dimos run px4-field             # aircraft in the field: px4-bench + link monitor
dimos run px4-sitl-perception   # px4-sitl with the real perception chain on a replayed clip
```

Then, from `dimos shell`: `px4_drone_connection.sitl_enable(True)`, `.takeoff()`,
`.set_guidance_mode("FOLLOW")`, `.land()`, `.estop()`, `.status()`, `.sensor_stats()`.

## Tests and gates

```bash
uv run pytest dimos/robot/px4 dimos/msgs/px4_msgs        # no hardware, no simulator
uv run python dimos/robot/px4/tool_sitl_gate.py --fly    # PX4 SITL: takeoff, hover, land
uv run python dimos/robot/px4/tool_bench_gate.py         # PX4 SITL: gimbal tf chain, frame stamps, aim
uv run mypy dimos/robot/px4 && uv run ruff check dimos/robot/px4
```

## Port contract

Fixed after Round 1; new modules bind by exact name and type. Topics are `dimos/<port>`.

| Direction | Port | Type | Rate |
|---|---|---|---|
| out | odometry, odom, tf | Odometry, PoseStamped, TFMessage | 30 Hz, latest-wins |
| out | imu | Imu | 50 Hz, latest-wins |
| out | motor_outputs | JointState (PWM us) | on change, up to 10 Hz |
| out | gps, global_pose | NavSatFix, PoseStamped (frame `home`) | 5 Hz |
| out | battery | BatteryState | 1 Hz |
| out | rc | Joy (raw microseconds) | 5 Hz |
| out | gimbal_attitude | JointState (radians, flags in effort) | 10 Hz, latest-wins |
| out | vehicle_status | VehicleStatus | 5 Hz |
| out | statustext | String, PX4's own messages | as they arrive |
| out | supervisor_status | String, the flown JSON, transitional | 5 Hz |
| out | supervisor_state | String | on change |
| out | command_event | CommandEvent | per operator command |
| out | offboard_setpoint | Odometry, stamped at the write | 20 Hz |
| out | robot_state | bytes, hosted UI plane | 2 Hz |
| out | stop_movement | Bool | event |
| in | cmd_vel | Twist, body FLU, honoured only in TELEOP | 20 Hz |
| in | gimbal_target | JointState (`gimbal_pitch`, `gimbal_yaw`) | up to 10 Hz |
| in | target_state, target_valid, target_los | Odometry, Bool, PoseStamped | 25 Hz |
| in | estop_in | Bool | event |

RPCs: `takeoff`, `land`, `hold`, `set_guidance_mode`, `estop`, `estop_land`,
`estop_clear`, `status`, `snapshot`, `sensor_stats`, `sitl_enable`. There is no arm, mode
or setpoint RPC, and `test_connection.py` asserts it.

## Command tracker

`command_tracker.py` answers "did that operator command take effect, and if not, why
not" without reading logs. It is read-only: it taps `command_event`, `offboard_setpoint`,
`odometry`, `vehicle_status` and `supervisor_state`, never opens MAVLink, is never
imported by the connection, and if it dies flight is unaffected (`test_command_tracker.py`
asserts it exposes nothing that actuates). Per command it publishes one typed
`tracked_command` (`TrackedCommand`): the verdict (`ok`, `rejected` with the SupervisorCore
enum value, `clamped`, `no_setpoint`, `no_motion`, `mode_not_offboard`,
`not_expected_to_move`, `held`), the supervisor state before and after, and three latency
segments measured separately and never summed: request to verdict, verdict to the first
Offboard setpoint reflecting it, that setpoint to the observed odometry response. A held
teleop key is one event, opened and closed on the connection's motion edges.

```bash
uv run pytest dimos/robot/px4/test_command_tracker.py     # synthetic streams, one test per rejection
uv run python dimos/robot/px4/tool_tracker_gate.py         # four scripted commands through zenoh, no simulator
dimos run px4-sitl-tracked                                 # px4-sitl + the tracker
dimos run command-tracker                                  # the tracker alone, binding to a running connection
```

From `dimos shell`: `command_tracker.recent()`, `.by_verdict("rejected")`, `.summary()`.

## Link monitor

`link_monitor.py` reports, continuously, what the operator link can carry right now, so
other modules decide instead of guessing. Advisory only: it never touches the flight path.
A timer thread polls the Quectel RM520N-GL over AT on `/dev/ttyUSB2` (`AT+QCSQ`,
`AT+QENG="servingcell"`, the port opened per poll so nothing else is locked out), the
overlay's own `tailscale status --json` for direct-or-relayed, an ICMP round trip to the
ground station and the interface counters. It publishes two typed streams: `link_status`
(signal, band, cell, interface, path, round trip, loss, measured throughput and one derived
`usable_uplink_bps`) and `link_policy` (video allowed, max video bitrate, JPEG rate,
telemetry profile). RtspCamera obeys the policy. `set_policy(...)` overrides it,
`clear_policy()` returns to the derived one. With no modem, no overlay and no serial port
it publishes NaNs and `healthy=False`, never an exception, so it starts on any laptop.

Peers are configured explicitly (`peers` in the config: name, tailnet address, role);
multicast discovery finds nothing on the overlay. One aircraft today; peer transport is a
later round. Once a second embodiment uses it this module moves to `dimos/network/`.

```bash
uv run pytest dimos/robot/px4/test_link_monitor.py       # replayed AT and overlay readings
uv run python dimos/robot/px4/tool_link_gate.py          # every recorded scenario against expected bands
dimos run link-monitor                                   # the module alone (hardware sources)
dimos run link-monitor --linkmonitor.source=replay       # replayed scenario, no modem
dimos run px4-field                                      # aircraft in the field: px4-bench + link monitor
dimos run px4-sitl-field                                 # the same against SITL with a replayed link
```

## Perception bridge

`perception_bridge.py` is the flown perception stack in one process: detector, the
persistent-ID tracker, the line-of-sight solver and the target ground-position estimator
(`perception/`), with the geometry and tracker parameters that flew. It consumes
RtspCamera's `color_image` plus the connection's `odometry`, `gimbal_attitude`,
`global_pose` and `vehicle_status`, and publishes the exact three streams the connection
reads for FOLLOW and YAW_TRACK, `target_state`, `target_valid` and `target_los`, so it is
interchangeable with `FakeTarget` in a blueprint. It also publishes `tracks`
(Detection2DArray, selected track first) for the viewer.

Operator click-to-select arrives on `track_select` as a pixel in the published frame; a
NaN point clears the selection. The selection persists while the track is lost. The four
flown scripts talked over UDP because they were processes; in one process the inbound
fan-out ports are gone, and the outbound JSON the flown gimbal controller (5608) and the
laptop viewer (5605, 5613) read is still emitted behind `legacy_udp_fanout`.

Detectors: `ultralytics` runs a YOLO `.pt` anywhere, or the same TensorRT `.engine` the
flown `yolo_live_trackfeed.py` built on the Jetson. `blob` is the test double: it finds the
synthetic clip's bright square by thresholding, so the tracker, line of sight and ground
intersection run on real pixels without a GPU. A recorded walking-person capture from the
bench replaces the synthetic clip in the gate when it exists.

```bash
uv run pytest dimos/robot/px4/perception dimos/robot/px4/test_perception_bridge.py
uv run python dimos/robot/px4/tool_perception_gate.py    # PX4 SITL + replayed clip + fake A8
dimos run px4-sitl-perception --rtspcamera.url=clip.mp4  # px4-sitl with the chain instead of FakeTarget
dimos run perception-bridge                              # the module alone
```

## Safety invariants

1. The RC pilot always wins. If PX4 is not in OFFBOARD the software stands down silently
   and sends no mode command. Pilot escape: flight-mode switch to Position or Hold, or any
   stick past `COM_RC_STICK_OV`.
2. Exactly one thing produces Offboard setpoints: the connection's tick thread through
   `MavlinkIO`.
3. If the tick stops, the stream stops and PX4's Offboard-loss failsafe puts the aircraft
   in Hold. Never add a fallback that keeps commanding.
4. E-STOP is Hold plus a latch. Nothing moves the aircraft until `estop_clear`, which only
   works in IDLE.
5. Gimbal aim commands go through the connection (`gimbal_target`) and only when
   `gimbal_commands_enabled` is set; until then the flown controller on component 191
   keeps control of the A8.

## Flight gates

Each must pass before the next, props off first, then with a pilot present.

1. Bench, props off: `takeoff` with the enable switch (channel 7) off stays in PREFLIGHT
   with reason `enable switch off`, returns to IDLE after 10 s, sends zero setpoints and
   no arm command.
2. Pilot Position-mode hover, gimbal tracking only.
3. `takeoff` to 3 m hover, `land`. Check altitude error, setpoint rate, no ABORT.
4. `set_guidance_mode("YAW_TRACK")`: the airframe yaws to keep the gimbal within 15 deg.
5. `set_guidance_mode("FOLLOW")`: standoff 12 m at 10 m altitude, 2 m/s cap.

## Frames and conventions

dimOS is FLU (x forward/north, y left/west, z up). MAVLink LOCAL_NED is converted in
`frames.py` with the same signs as `dimos/robot/drone/mavlink_connection.py`. The gimbal
reports body-relative yaw and earth-stabilised pitch (verified 2026-09-04). Vehicle boot
time becomes UTC through `mavlink/timebase.py` from SYSTEM_TIME.

## Network reality (measured 2026-09-09)

Wi-Fi direct 7 ms. Both ends on cellular through the Tailscale relay: 86 ms average with
throttled telemetry, 413 ms and 5 % loss with raw telemetry, and no headroom for video on
that cell. Nothing flight-critical crosses the link.
