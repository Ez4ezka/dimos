# SIYI A8 mini gimbal

`SiyiA8Gimbal` publishes the gimbal tf chain, the camera intrinsics and aim requests.

## How it works

- Reads `gimbal_attitude` (JointState, degrees) and publishes
  `base_link > gimbal_base > gimbal_link > a8_optical` on `tf` at 10 Hz.
- Publishes `camera_info` at 1 Hz. With `sdk_enabled=True` it polls the zoom over the SIYI
  SDK (UDP 37260) and withholds `camera_info` while the zoom is not 1x.
- With `aim_enabled=True` it turns `target_los` into `gimbal_target` aim requests.
- It opens no MAVLink socket. The module that owns the MAVLink link reads `gimbal_target`
  and commands the gimbal.
- The A8 reports yaw relative to the body and pitch stabilised to the earth
  (`mount_preset="flight"`).

| File | Holds |
|---|---|
| `gimbal.py` | the module |
| `frame.py` | frame maths: mount presets, limits, quaternions |
| `sdk.py` | SIYI SDK packets: zoom, attitude, codec specs |
| `replay.py` | recorded and synthetic `gimbal_attitude` streams for tests |

## Run

```bash
dimos run siyi-a8-gimbal
```

RPCs: `aim(pitch_deg, yaw_deg)`, `center()`, `state()`.

## Setup

Measure the offset from `base_link` to the gimbal base and set `mount_xyz`. The default is
a placeholder and the module warns at start.

## Test

No gimbal needed.

```bash
uv run pytest dimos/hardware/gimbal/siyi
```
