# RTSP camera (SIYI A8 mini)

`RtspCamera` turns an RTSP H.265 stream into dimos streams: the encoded access units
untouched (`video`), a capped decoded `color_image` for on-Jetson consumers, and a small
`color_jpeg` for the operator link. It obeys `link_policy` from the link monitor. Video is
its own interface on the aircraft (Ethernet to `192.168.144.25`), separate from gimbal
control over MAVLink, so this module knows nothing about the gimbal.

## Aircraft-side facts

- The Jetson Orin Nano decodes H.265 in hardware (NVDEC, up to 4K60) and has **no
  hardware encoder**. Anything re-encoded onboard costs one to two CPU cores per 1080p30
  stream, so this module never re-encodes: the link bitrate must come from the camera.
- The A8 mini serves a main stream on `rtsp://192.168.144.25:8554/main.264` (H.265,
  1280x720, 25 fps) and a sub stream. The SIYI SDK codec-spec commands (0x20 read, 0x21
  write: stream, codec, resolution, bitrate) let the camera emit a lower bitrate itself.
  Not yet verified on this unit: run the `codec_specs()` RPC with `sdk_enabled=True` on
  the bench and record the answer here.
- Decoding on the Jetson: `decoder="gst-nv"` runs `rtspsrc ! rtph265depay ! h265parse !
  nvv4l2decoder ! nvvidconv ! videorate ! fdsink` as a subprocess and reads fixed-size
  BGRx frames. On any other machine `decoder="av"` decodes in-process with PyAV.

## Timestamps

Every frame is stamped at the packet read minus `capture_latency_s`, on the Jetson wall
clock, the same UTC clock the connection stamps odometry with, so frames align with
vehicle attitude for line-of-sight geometry. The bench gate measures the residual.

## Tests and gates

```bash
uv run pytest dimos/hardware/sensors/camera/rtsp           # synthetic H.265 clip, no camera
uv run python dimos/robot/px4/tool_bench_gate.py           # with PX4 SITL: stamps, tf, aim
dimos run rtsp-camera-vis                                  # the A8, or --rtspcamera.url=clip.mp4
dimos run rtsp-camera                                      # the module alone
dimos run px4-bench                                        # on the aircraft, props off
```

The synthetic clip (`synthetic.py`) stands in until a real A8 capture is recorded on the
bench; a capture replays through the same code by passing its path as `url`.
