# RTSP camera

`RtspCamera` turns an RTSP H.265 stream into dimOS streams. Defaults are for the SIYI A8
mini.

## How it works

- `video`: the encoded H.265 access units, untouched. Nothing is re-encoded.
- `color_image`: decoded frames, capped at `color_hz`, for consumers on the same machine.
- `color_jpeg`: a small JPEG (`jpeg_hz`, `jpeg_max_width`) for a slow operator link.
- It obeys `link_policy` (video on or off, rates) when a module publishes one. Set
  `obey_link_policy=False` to ignore it.
- Frames are stamped at packet read minus `capture_latency_s`, on the system clock.
- `url` is an RTSP URL, a file path (replayed through the same code) or `synthetic` (a clip
  generated at start).
- `decoder="av"` decodes in-process with PyAV. `decoder="gst-nv"` uses the Jetson hardware
  decoder through a `gst-launch-1.0` subprocess.
- With `sdk_enabled=True`, `codec_specs()` and `set_codec()` read and set the A8's stream
  resolution and bitrate over the SIYI SDK. Not verified on hardware yet.

## Run

```bash
dimos run rtsp-camera-vis                               # the A8 at rtsp://192.168.144.25:8554/main.264
dimos run rtsp-camera-vis --rtspcamera.url=clip.mp4     # replay a capture
dimos run rtsp-camera-vis --rtspcamera.url=synthetic    # no camera
dimos run rtsp-camera                                   # the module alone, no viewer
```

The JPEG path needs the system library: `sudo apt install libturbojpeg`.

## Test

No camera needed.

```bash
uv sync --extra px4
uv run pytest dimos/hardware/sensors/camera/rtsp
```
