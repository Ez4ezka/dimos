# SIYI A8 mini gimbal

`SiyiA8Gimbal` publishes the gimbal frame chain `base_link -> gimbal_base -> gimbal_link
-> a8_optical`, the camera intrinsics, and aim requests. It opens no MAVLink socket: on
the aircraft PX4 is the gimbal manager and the one MAVLink writer is
`Px4DroneConnection`, so aim requests go out as `gimbal_target` and the connection
forwards them only when its `gimbal_commands_enabled` flag is set. Until then the flown
controller on component 191 keeps the A8 and this module is observe-only.

`frame.py` holds the verified frame maths (mount presets, limits, quaternion handling),
`sdk.py` the SIYI Ethernet SDK on port 37260 (zoom, attitude, codec specs), `replay.py`
the recorded and synthetic `gimbal_attitude` streams for tests.

## Mount

`mount_xyz` (base_link to gimbal_base) is an UNMEASURED placeholder and the module warns
at start until a tape-measured value replaces it. The A8 reports yaw body-relative and
pitch earth-stabilised (flight mount, verified 2026-09-04).

## Tests and gates

```bash
uv run pytest dimos/hardware/gimbal                        # frame maths, SDK packets, the module; no A8
uv run python dimos/robot/px4/tool_sitl_gate.py            # with PX4 SITL and FakeA8: tf chain, aim
dimos run siyi-a8-gimbal                                   # the module alone, beside a running connection
dimos run px4-drone                                        # on the aircraft
```

A bench capture of `gimbal_attitude` (JSONL, degrees) loads with
`replay.load_attitude_record`; until one exists `synthetic_attitude_record()` is used.
