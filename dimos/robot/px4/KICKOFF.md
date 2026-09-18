# Kickoff context — dimOS PX4 package for dimosdrone-2

You are building a new dimOS package, `dimos/robot/px4/`, that turns a working bespoke
PX4 drone stack into first-class dimOS modules.

Read this whole file before you touch anything. Then follow the reading protocol in
section 2 before you write a single line of code. Do not skip it — the point of this
task is to match an existing architecture, not to invent one.

---

## 1. Ground rules

**Architecture reference is non-negotiable.**
`dimos/robot/galaxea/r1pro/connection.py` and
`dimos/robot/galaxea/r1pro/blueprints/basic/r1pro_coordinator.py` are the template.
Every structural decision you make should be traceable to something those two files do.
If you find yourself doing something they don't do, stop and justify it in a comment,
or don't do it.

**Read before you write.** You do not know what dimOS idioms look like from memory.
Open the reference files, read them properly, and take notes on the patterns before
you start.

**Port, don't rewrite.** Most of the logic you need already exists and has flown or
passed tests. Your job is mostly translation into dimOS shapes, not new algorithms.
When you port something, keep the maths byte-identical and cite the source file and
line range in a comment.

**Repo hygiene.**
- Work in the clean `~/dimos` checkout. Fetch it to current upstream `main` and branch
  `feat/px4-drone` from there.
- **Never touch `~/Work/dimos`.** It is a fork checkout on `feat/mixed-embodiment-sim`
  with unpushed work.
- Before doing anything, verify these branches are pushed and warn me if they are not:
  `feat/drone-px4-swarm-ns` and `feat/drone-tello-tt-integration` in `~/Work/dimos`.
  They exist on no remote.
- `DroneReference/` is 14 GB and untracked. Never run `git clean` anywhere near it.

**Safety invariants that must hold in every commit.**
1. The RC pilot always wins. If the PX4 main mode is not OFFBOARD, the software stands
   down silently and sends no mode command.
2. Exactly one thing produces Offboard setpoints, ever.
3. If that thing stops, the setpoint stream stops, and PX4's own Offboard-loss failsafe
   takes over. That is the correct failure direction — never add a fallback that keeps
   commanding.
4. E-STOP means Hold (AUTO.LOITER) plus a latch. No software command can move the
   aircraft again until `estop_clear`, and `estop_clear` only works in IDLE.
5. Software kill is refused unless the aircraft is landed.

---

## 2. Reading protocol — do this first

Read these and tell me what you learned from each before writing code.

### A. The architecture template (read closely, take notes)

| File | What to extract |
|---|---|
| `dimos/robot/galaxea/r1pro/connection.py` | Module class shape, `dedicated_worker`, config class with `Field` defaults and comments, lazy imports inside `start()`, socket opened in `start()` never `__init__`, per-message-type stat counters plus a `sensor_stats` RPC, drift-free publish loop, how the static transform edge is republished every tick |
| `dimos/robot/galaxea/r1pro/blueprints/basic/r1pro_coordinator.py` | `autoconnect`, `.transports()`, the `_zenoh_transport` helper, rerun blueprint construction, rate caps, visual overrides, `.global_config()`, how a coordinator composes visualization and control |
| `dimos/robot/deeprobotics/m20/camera.py` | PyAV demux pattern for an RTSP stream |
| `dimos/navigation/movement_manager/movement_manager.py` | How the teleop-versus-autonomy mux is wired; reuse unchanged |
| `dimos/teleop/hosted/go2_command.py` | The hosted command module you will transpose 1:1 |
| `dimos/memory/tap.py` | How `--record` decides a port is recordable (this is why no port may be `Out[Any]`) |

### B. The anti-reference — read it to know what NOT to copy

`dimos/robot/drone/mavlink_connection.py` is the existing DJI/RosettaDrone MAVLink module.
Do **not** derive from it. Specifically it:
- binds `udp:0.0.0.0:14550`, which would collide with our gimbal controller
- uses source system 255
- calls `wait_heartbeat()`, which mis-latches on a multi-component bus
- ships an ArduCopter mode table, wrong for PX4
- calls blocking `recv_match` from RPC threads

The one thing to take from it is the NED-to-dimOS coordinate conversion at
`mavlink_connection.py:170`, which is guarded upstream by
`test_ned_to_ros_coordinate_conversion`. Match it exactly.

### C. The source material to port from

Repo `~/drone-autonomy` at `e7c640c`. This code has flown.

| Source | Port into | Notes |
|---|---|---|
| `common/mavconn.py:14-93` | `dimos/robot/px4/mavlink/io.py` | sysid/compid constants, URL resolution, heartbeat, `wait_for_px4` filtering on srcSystem 1 **and** srcComponent 1 |
| `common/px4_offboard.py:11-58` | `dimos/robot/px4/mavlink/px4_modes.py` | mode enums, `MASK_POS_YAW` / `MASK_VEL_YAW` / `MASK_VEL_YAWRATE`. Pure data, no I/O |
| `common/px4_offboard.py:65-78` | `mavlink/io.py` | the two setpoint senders, exact port |
| `common/telemetry.py:18-137` | `mavlink/vehicle_state.py` | wrap-aware interpolating buffers; keep both receive time and vehicle boot time per entry |
| `common/guidance.py` | `dimos/robot/px4/guidance.py` | `yaw_track_rate`, `follow_velocity`, `rate_limit_yaw` — maths unchanged |
| `common/gimbal.py:17-86` | `dimos/hardware/gimbal/siyi/frame.py` | limits, flag bits, quaternion to euler, mount inversion. Use `config/gimbal_frame.json` values as a `MountPreset` (flight / bench) instead of per-sample sign detection |
| `flight_supervisor.py:43-303` | `dimos/robot/px4/supervisor_core.py` | **The critical one.** See section 4 |
| `tests/test_guidance.py:92-213` | `dimos/robot/px4/test_supervisor_core.py` | Must port 1:1 and keep passing |
| `sitl/fake_target.py` | `dimos/robot/px4/sitl/fake_target.py` | line / circle / drop windows |
| `docs/SYSTEM.md`, `docs/RUNBOOK.md`, `docs/PX4_PARAMS.md`, `docs/PREFLIGHT.md` | `dimos/robot/px4/README.md` | condense |

Also read, reference only, never edit: local branch `feat/drone-px4-swarm-ns` in
`~/Work/dimos`, file `px4_sitl_connection.py:87-98,701-725` for the PX4 mode table and
the `custom_mode` decoder, plus the AMSL takeoff-altitude conversion.

---

## 3. The hardware you are writing against

Everything below is measured, not assumed.

**Aircraft `dimosdrone-2`:** Holybro X500, Pixhawk 6C on PX4 v1.17.0 (custom CRSF build),
Jetson Orin Nano (JetPack 6, L4T R36.4.3, CUDA 12.6, TensorRT 10.3), SIYI A8 mini on a
hanging mount, RTK GNSS (fix type 4, 25–27 sats, 0.6 m), ELRS from a RadioMaster TX16S,
Quectel RM520N-GL 5G modem on T-Mobile band n41.

**Fixed addresses on every network:** Jetson `100.110.224.46`, laptop `100.100.114.31`,
A8 camera `192.168.144.25`.

**MAVLink transport:** Pixhawk TELEM2 to Jetson `/dev/ttyTHS1`, 921600 8N1, MAVLink v2,
PX4 "Onboard" profile. Measured 418 msg/s at 205 kbit/s. Stream rates: ATTITUDE 100 Hz,
GLOBAL_POSITION_INT 50, HIGHRES_IMU 50, ODOMETRY 30, LOCAL_POSITION_NED 30,
RC_CHANNELS 20, gimbal status 10, GPS_RAW_INT and SYS_STATUS 5.

**Existing fan-out — do not disturb any of these.** `mavlink-routerd` owns the UART with
`SnifferSysid=1`. UDP 14550 gimbal control (component 191), 14551 line-of-sight (192),
14552 the old supervisor (193), 14553 logger, 14554 QGroundControl throttle.
**You add one endpoint: 14556, component 195.**

**Component IDs on the bus:** 1/1 is PX4. 154 is the SIYI A8, reached through PX4's second
MAVLink instance using gimbal protocol v2, body-relative yaw, earth-stabilised pitch.
Services sit on 191–194. Filter accordingly — never assume a message came from PX4 just
because it arrived.

**Offboard:** `SET_POSITION_TARGET_LOCAL_NED` at 20 Hz. PX4 falls to Hold 0.5 s after the
stream stops.

**Known-open:** the PX4 parameter list in `docs/PX4_PARAMS.md` is not yet set on the
vehicle. Verify the offboard-loss failsafe parameter names against v1.17 before relying
on them — PX4 has reworked these in recent releases.

**Link reality (measured 9 Sep):** Wi-Fi direct 7 ms. Both cellular through the DERP relay,
throttled: 86 ms average, 0% loss, 115 kbit/s. Both cellular with raw telemetry:
413 ms average, 5% loss, QGroundControl stalls. Both cellular with video: 100% loss,
only 0.26 Mbit/s spare on that cell. **This is why nothing flight-critical crosses the
link.**

---

## 4. The architecture decision you are implementing

An earlier plan split this into `Px4Connection` (driver plus a gate token) and
`Px4FlightSupervisor` (state machine), both as dedicated workers. That put a process
boundary inside a 20 Hz flight loop. **We are merging them.**

Build one module, `Px4Drone`, and keep the classes inside it separate:

```
dimos/robot/px4/px4_drone.py        Module shell: config, ports, threads, wiring. NO LOGIC.
dimos/robot/px4/mavlink/io.py       MavlinkIO — pymavlink socket, reader thread, ack futures
dimos/robot/px4/supervisor_core.py  SupervisorCore — pure state machine, NO I/O
dimos/robot/px4/mavlink/vehicle_state.py   VehicleState — timed buffers, pure
dimos/robot/px4/mavlink/timebase.py        Px4Timebase — boot to UTC, pure
dimos/robot/px4/mavlink/frames.py          NED/FRD conversions, pure
dimos/robot/px4/guidance.py                ported guidance maths, pure
```

`SupervisorCore` takes a `VehicleSnapshot` dataclass and drives a `Px4Actuator` protocol.
It must contain no socket, no port, no import from pymavlink. This is what lets the
ported test suite run with no hardware, and it is the hard gate on this work:

> **`test_supervisor_core.py` must pass without being edited. If you needed to change
> that file, the merge went wrong — stop and tell me.**

**The gate token is gone.** It existed only to guard a cross-process call. Instead:
- The setpoint senders are private methods on `MavlinkIO`. They appear on **no** RPC surface.
- `arm`, `set_px4_mode`, `send_position_setpoint`, `send_velocity_setpoint` and
  `offboard_gate_acquire` do not exist as RPCs. Assert this in a test.
- Keep a `writer` field in `vehicle_status` so the single-writer invariant stays observable
  in recordings.
- The module refuses to start if another system-1 component is already streaming
  `SET_POSITION_TARGET_LOCAL_NED`. Prefer detecting the old supervisor process directly
  (pidfile, socket bind on 5610) over inferring it from traffic, because the old one only
  streams when armed.

**Rejection reasons are a closed enum, not free strings**, because the command tracker
classifies on them: `not_teleop`, `estop_latched`, `enable_switch_off`, `stale_input`,
`preflight_failed`, `not_armed`, `mode_not_offboard`, `fence`, `ceiling`, `battery`.

---

## 5. Module specifications

### 5.1 `Px4Drone` — `dimos/robot/px4/px4_drone.py`, `dedicated_worker = True`

**Config** (mirror the R1 Pro style: `Field` defaults with a comment on each)
`mav_url="udpin:127.0.0.1:14556"`, `source_system=1`, `source_component=195`,
target 1/1, `connect_timeout_s=30`, `heartbeat_hz=1.0`, `ack_timeout_s=3`,
frame ids (`odom`, `base_link`, `gimbal_base`), `gimbal_mount_xyz` with a loud
UNMEASURED default, `gimbal_mount_preset="flight"`, publish rates
(odom 30, imu 50, rc 5, status 5, gps 5, battery 1), `timebase_source="system_time"`,
`gimbal_commands_enabled=False`, `sitl=False`, `sensor_stats_interval_s=10`.

Limits: takeoff 3 m, climb 0.7 m/s, ceiling 15 m, fence 30 m, battery ≥ 40%,
GPS fix ≥ 3, eph ≤ 1.5 m, RC enable channel 7 ≥ 1500 µs, stale 1 s, setpoint 20 Hz,
prestream 1.5 s, hover tolerance 0.5 m.

Follow guidance: yaw deadband 15°, k 0.6, 30°/s; standoff 12 m, altitude 10 m,
2 m/s cap, 0.7 m/s vertical, loss hover 5 s.

Teleop: 1.5 m/s horizontal, 0.7 m/s vertical, 0.8 rad/s, stale 0.5 s,
`teleop_lock_altitude=True`.

**Foreign I/O (the module owns this socket)**
MAVLink v2 both directions on one UDP socket at `127.0.0.1:14556`.

**Input ports**
| Port | Type | Rate | Note |
|---|---|---|---|
| `cmd_vel` | `Twist` | 20 Hz | from MovementManager, honoured only in TELEOP |
| `target_state` | `Odometry` | 25 Hz | from the perception bridge |
| `target_valid` | `Bool` | 25 Hz | |
| `target_los` | `PoseStamped` | 25 Hz | |
| `estop_in` | `Bool` | event | mirror only; the RPC is the guaranteed path |

Note what is **not** an input: odometry, rc, gps, battery, vehicle_status,
gimbal_attitude. Those were only ports because the supervisor lived elsewhere. The core
now reads `VehicleState` directly. They remain outputs for everyone else.

**Output ports**
| Port | Type | Rate | Source |
|---|---|---|---|
| `odometry` | `Odometry` | 30 Hz | LOCAL_POSITION_NED + ATTITUDE interpolated to the same boot time, `odom`→`base_link` |
| `odom` | `PoseStamped` | 30 Hz | pose only |
| `tf` | `TFMessage` | 30 Hz | moving `odom`→`base_link` plus the static `base_link`→`gimbal_base` mount edge republished every tick |
| `imu` | `Imu` | 50 Hz | HIGHRES_IMU, FRD to FLU |
| `gps` | `NavSatFix` | 5 Hz | GPS_RAW_INT; fix type to status, eph/epv to covariance |
| `battery` | `BatteryState` | 1 Hz | SYS_STATUS, percentage 0..1 |
| `rc` | `Joy` | 5 Hz | RC_CHANNELS, **raw microseconds in axes** so the 1500 µs threshold keeps meaning |
| `vehicle_status` | `String` | 5 Hz | `px4_status_v1` JSON: armed, main/sub mode and name, landed state, battery, GPS fix and sats and eph, RC and heartbeat ages, home, timebase quality, `writer`, `state`, `estop_latched`, `tick_jitter_ms` |
| `gimbal_attitude` | `JointState` | 10 Hz | GIMBAL_DEVICE_ATTITUDE_STATUS, **component 154 only**, normalised via the mount preset, radians, flags in `effort` |
| `global_pose` | `PoseStamped` | 5 Hz | relative_alt and heading, frame `home` |
| `supervisor_status` | `String` | 5 Hz | `supervisor_status_v1`, field-compatible with today's packet so `opcmd.py watch` keeps working |
| `supervisor_state` | `String` | on change | |
| `offboard_setpoint` | `Odometry` | 20 Hz | **stamped at the moment MavlinkIO wrote the datagram, not at publish time** — the tracker uses this as its reference and publish congestion would corrupt every measurement |
| `robot_state` | `bytes` | 2 Hz | hosted UI plane |
| `stop_movement` | `Bool` | event | |

**RPC surface**
Skills: `takeoff`, `land`, `hold`, `set_guidance_mode(HOVER|YAW_TRACK|FOLLOW|TELEOP)` —
each returns acceptance or a rejection enum.
Safety: `estop()` synchronous and latching, `estop_land()`, `estop_clear()` (IDLE only).
Query: `status()`, `snapshot()`, `sensor_stats()`. SITL only: `sitl_enable(value)`.

**Threads**
`px4-reader` — the only `recv_match` caller in the process; dispatches into `VehicleState`
and resolves COMMAND_ACK futures keyed by command id so no RPC blocks on the socket.
`px4-publish` — 100 Hz drift-free loop, each stream at its own divisor.
`px4-heartbeat` — 1 Hz.
`px4-tick` — 20 Hz: snapshot → `SupervisorCore.step()` → apply via a direct `MavlinkIO`
method call → publish the mirror → record jitter.

**Lifecycle**
`start()`: resolve URL, open socket, spawn reader, block for a heartbeat from **1/1
specifically** with timeout, collect 30 SYSTEM_TIME samples for the boot-to-UTC offset
(median with a jump guard, min-filtered fallback with a quality flag), then spawn publish,
heartbeat and tick.
`stop()`: tick first, then heartbeat, then reader, then close. Reversed, you can emit a
setpoint after teardown starts. Write a test for this.

### 5.2 `Px4CommandTracker` — `dimos/robot/px4/command_tracker.py`

Read-only. No RPC that can touch the aircraft. Runs on the Jetson.

**Why it exists:** right now "teleop works" is unverifiable except by watching the drone.
This turns every keypress into a recorded event with a verdict and a latency breakdown,
which is also the artifact we need for the 5G benchmark.

**Correlation strategy: edge detection.** `Twist` has no identity and a held key is 20
messages a second, not 20 commands. A rising edge past the deadband opens an event; a
falling edge or `event_max_s` closes it.

**Inputs:** `tele_cmd_vel` 20 Hz, `cmd_vel` 20 Hz, `offboard_setpoint` 20 Hz,
`odometry` 30 Hz, `vehicle_status` 5 Hz, `supervisor_status` 5 Hz, `cmd_ack` event.

**Outputs:** `command_events` (String, per event), `teleop_health` (String, 1 Hz rolling
window), `command_report` (String, one readable line per event),
`cmd_forward` / `meas_forward` (Float32, 20 Hz, for a rerun scalar overlay).

**The four stages, measured as three separate segments — never summed:**
```
t_intent      tele_cmd_vel received on the Jetson        (monotonic)
t_accepted    cmd_vel reached the module, state = TELEOP (monotonic)
t_commanded   MavlinkIO wrote the datagram               (monotonic)
t_moved       measured body velocity crossed threshold   (vehicle time → UTC)
```
`mux` = accepted − intent. `onboard` = commanded − accepted. `response` = moved − commanded.
Operator-to-Jetson link latency is a **different clock** — report it as a separate
`link_ms` from the broker's latency stamp on the hosted path, and `null` on the viewer
path. Do not add them together.

**Verdicts:** `ok`, `clamped` (with ratio), `rejected` (carries the enum),
`no_setpoint`, `no_motion`, `mode_not_offboard`, `not_expected_to_move`, `held`.

`not_expected_to_move` is derived from `vehicle_status` (disarmed or on-ground), **not** a
config flag, so the bench and the flight use the identical blueprint. Check
`mode_not_offboard` before falling through to `no_motion` — otherwise a PX4 refusal looks
like an unresponsive airframe.

**Event schema `command_event_v1`** — include `event_id`, `source`, `axis`, `sign`,
`opened_utc`, `duration_s`, `intent`, `commanded_peak`, `measured_peak_body`, `verdict`,
`rejection`, `clamp_ratio`, `latency_ms{mux,onboard,response,link}`,
`vehicle{armed,mode,state,landed_state,estop_latched}`, `setpoint_count`,
`setpoint_gap_max_ms`, `scoring_version`.

`setpoint_gap_max_ms` is the cheapest early warning we have for tick jitter — if it
approaches 500 ms the Offboard-loss failsafe is about to fire. `scoring_version` exists
because `response_frac` will be retuned after the first flight and we need old events to
stay comparable.

**Config:** `deadband_mps=0.05`, `deadband_radps=0.05`, `response_frac=0.3`,
`response_min_mps=0.15`, `response_timeout_s=1.5`, `event_max_s=10`,
`health_window_s=30`, `health_hz=1`.

### 5.3 Supporting modules (build after the first two work)

**`SiyiA8Gimbal`** — `dimos/robot/px4/gimbal.py`.
In: `gimbal_attitude` 10 Hz, `odometry` 30 Hz (the A8's pitch and roll are
earth-stabilised but yaw is body-relative, so the transform edge needs both),
optional `gimbal_target`.
Out: `tf` 10 Hz (`gimbal_base`→`gimbal_link`→`a8_optical`, with
`Quaternion(-0.5, 0.5, -0.5, 0.5)` on the optical edge like every dimOS camera),
`gimbal_state`, `camera_info` (fx = fy ≈ 749.3, cx 640, cy 360, 1280×720, HFOV 81°,
gated invalid when zoom ≠ 1.0), `gimbal_yaw_body`.
`take_control=False` in phase 1 — `selected_gimbal_control.py` on component 191 keeps
control and must not be fought. No transform from attitude older than 1 s.

**`RtspVideoCamera`** — `dimos/hardware/sensors/camera/rtsp/rtsp_video_camera.py`,
dedicated worker.
In: RTSP from `192.168.144.25` (H.265, 1280×720, 25 fps, `rtspsrc latency=50`),
`link_profile` String.
Out: `video` (CompressedVideo, H.265 passthrough, **every access unit** — the A8 GOP is
unknown so keyframe-only breaks decode), `color_image` (Image, ≤10 Hz, **bound to LCM,
on-Jetson only** — it is 2.8 MB a frame and must never be dialable), `color_jpeg`
(CompressedImage, 2 Hz, q50, ≤640 px), `camera_info`.
Decode via PyAV demux plus a `gst-launch-1.0` subprocess
(`rtspsrc ! rtph265depay ! h265parse ! nvv4l2decoder ! nvvidconv ! videorate !
video/x-raw,format=BGRx ! fdsink`) read as fixed-size frames. Do **not** use the
in-process `gstreamer_camera.py` skeleton — its wall-clock PTS check drops every frame,
and the system `gi` bindings do not load in a Python 3.12 venv.

**`Px4PerceptionBridge`** — `dimos/robot/px4/perception_bridge.py`.
Owns three non-blocking UDP sockets: 5616 `tracks_select_v1`, 5617 `target_los_v1`,
5618 `target_state_v1`. These are **new** fan-out destinations added to the existing
tracker and estimators — the current 5605 / 5608 / 5612 destinations keep working so the
old supervisor survives the transition.
Out: `tracks` (Detection2DArray, frame `a8_optical`, stamp = `capture_time`, selected
track first), `target_los`, `target_state` (frame `odom`, child `target`, covariance from
`pos_sigma_m`), `target_valid`, `target_status` (raw JSON passthrough), `target_geo`.
Malformed packets are counted, never raised.

**`Px4LinkMonitor`** — `dimos/robot/px4/link_monitor.py`. Advisory only, never touches the
flight path. Polls `AT+QCSQ` / `AT+QENG` on `/dev/ttyUSB2` (ModemManager is disabled),
`tailscale status --json`, ICMP to `100.100.114.31`. Out: `link_status` 1 Hz,
`link_profile` (low on relay, or RTT > 250 ms, or headroom < 300 kbit/s for 5 s; high
again only after 20 s), `link_rtt`, `link_rsrp`.

**New message wrappers** in `dimos/msgs/sensor_msgs/`: `NavSatFix.py` and
`BatteryState.py`, thin wrappers over the already-shipped `dimos_lcm.sensor_msgs` types.
Follow the `CompressedImage.py` recipe: `msg_name`, `lcm_encode`, `lcm_decode`,
`to_rerun`.

---

## 6. Blueprints

Mirror `r1pro_coordinator.py` exactly in shape. Define `_px4_rerun_blueprint()`,
`_RERUN_MAX_HZ` (color_jpeg 2, odometry/odom/tf 10, imu 5, tracks 10, gimbal_state 10,
**no cap on `world/video`** — dropping access units breaks in-viewer decode, control the
rate at the source instead), `_RERUN_VISUAL_OVERRIDE` (`color_image`, `vehicle_status`,
`target_status` → None), and the verbatim `_zenoh_transport()` helper.

Blueprints to produce: `px4-sitl`, `px4-sitl-follow`, `px4-teleop`, `px4-teleop-verify`
(teleop plus the tracker plus a commanded-versus-measured scalar plot — this is the demo
configuration), `px4-follow`, `px4-record`, `px4-remote-5g`, `teleop-hosted-px4`,
`px4-hil-basic`.

Every derived blueprint calls `px4_control()` afresh — `.transports()` is applied by value
and does not propagate into blueprints built from an earlier value.

`color_image` is the only port bound to `LCMTransport`; everything else is zenoh.
Heavy streams get `latest_wins=True`. Nothing crossing the link uses blocking congestion
control.

---

## 7. Build order and gates

**N0 — merge and scaffold.** Package skeleton, `mavlink/*` ports, message wrappers, stubs,
`pyproject.toml` extra `px4 = ["pymavlink", "av", "pyserial"]`, `Px4Drone`, blueprint
registry regenerated via `pytest dimos/robot/test_all_blueprints_generation.py`.

> Gate: `test_supervisor_core.py` passes **unedited**. `pytest dimos/robot/px4` green.
> mypy strict, ruff and `dimos/codebase_checks` clean. Against SITL (`gz_x500`),
> `dimos run px4-sitl` shows odometry at 30 Hz with UTC stamps within 50 ms of laptop
> NTP time. Tick jitter p99 recorded — report it to me before moving on.

**N1 — tracker in SITL.** `command_events.py`, `command_tracker.py`, `px4-teleop-verify`.

> Gate: a scripted sequence — W held 2 s, A tapped, W while in HOVER, W while E-STOP is
> latched — produces exactly four events with verdicts `ok`, `ok`, `rejected/not_teleop`,
> `rejected/estop_latched`. Latency segments populated and non-negative.

**N2 — bench, props off.** Gimbal, camera, operator doc. Run beside the old supervisor
first, then alone.

> Gate: every event reads `not_expected_to_move` with `t_commanded` populated.
> `setpoint_gap_max_ms` under 80. Takeoff with the enable switch off stays in PREFLIGHT
> with reason `enable_switch_off`, goes IDLE after 10 s, and sends zero setpoints and no
> arm command.

**N3 — flight and benchmark.** Only after I confirm the PX4 parameters are set and a pilot
is present.

---

## 8. Codebase checks that will fail you

No `__init__.py`. No `__all__`. No `logging.getLogger` — use structlog. No module-level
`cv2` or `rerun` imports. Blueprint kwargs must be config fields. mypy strict passes,
which means extending `stubs/pymavlink/mavutil.pyi` with `heartbeat_send`,
`command_int_send`, `mavlink_connection` kwargs, and the gimbal-manager and landed-state
constants. Test fixtures must call `stop()` or the thread-leak detector fails.

---

## 9. How to start

Do not write code yet. Do this:

1. Confirm the two unpushed branches in `~/Work/dimos` are safe, and tell me if they are not.
2. Fetch `~/dimos` to upstream `main` and create `feat/px4-drone`.
3. Read everything in section 2A and 2B. Summarise, in your own words: how an R1 Pro style
   connection module is structured, how its coordinator blueprint composes, and the three
   specific things the DJI module does that we must not copy.
4. Read the port sources in section 2C and tell me which parts of `flight_supervisor.py`
   are pure logic and which are I/O, so we agree on the `SupervisorCore` boundary before
   you split it.
5. Propose the file list you intend to create, and wait for me to confirm.

Then build N0.
