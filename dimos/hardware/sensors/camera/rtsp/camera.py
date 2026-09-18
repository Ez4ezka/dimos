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

"""RtspCamera: an RTSP H.265 camera (the SIYI A8 mini) as dimos streams.

Video over Ethernet is a separate interface from gimbal control over MAVLink, and the two
fail independently, so this Module knows nothing about the gimbal. It publishes the
encoded stream untouched (every access unit, because the A8 GOP is unknown and dropping
one breaks decode), a capped decoded ``color_image`` for on-Jetson consumers such as the
perception bridge, and a small JPEG for the operator link.

Every frame is stamped at the moment the packet was read, minus ``capture_latency_s``,
on the Jetson wall clock, the same UTC clock the connection stamps odometry with, so a
frame can be aligned with vehicle attitude for line-of-sight geometry.

Decoding: PyAV in-process (``decoder="av"``, any machine) or the Jetson's NVDEC through a
``gst-launch-1.0`` subprocess (``decoder="gst-nv"``). The Orin Nano decodes H.265 in
hardware but has no hardware encoder, so nothing here ever re-encodes: a lower bitrate for
the link comes from the camera's own codec settings (:mod:`dimos.hardware.gimbal.siyi.sdk`).
A local file path instead of an RTSP URL replays a capture through the same code, and
``url="synthetic"`` generates a short clip at start (a white square parked at the image
centre) so the simulator twin has a camera without any file on disk.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from pathlib import Path
import subprocess
import tempfile
import threading
import time
from typing import Any, Literal

import av
from pydantic import Field
from reactivex.disposable import Disposable

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.hardware.gimbal.siyi.sdk import A8_IP, STREAM_MAIN, STREAM_SUB, CodecSpec, SiyiSdk
from dimos.hardware.sensors.camera.rtsp.synthetic import write_synthetic_h265
from dimos.msgs.foxglove_msgs.CompressedVideo import CompressedVideo
from dimos.msgs.link_msgs.LinkPolicy import LinkPolicy
from dimos.msgs.sensor_msgs.CompressedImage import CompressedImage
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

A8_RTSP_URL = f"rtsp://{A8_IP}:8554/main.264"
SYNTHETIC_URL = "synthetic"
_RECONNECT_WAIT_S = 2.0


@dataclass
class _StreamStat:
    """Per-stream counters; see the ``sensor_stats`` rpc."""

    received: int = 0
    published: int = 0
    dropped: int = 0
    errors: int = 0
    bytes_in: int = 0
    last_mono: float = 0.0


class RtspCameraConfig(ModuleConfig):
    # RTSP URL of the camera, a local file path to replay a capture, or "synthetic" for a
    # generated clip (SITL twin, tests).
    url: str = Field(default=A8_RTSP_URL)
    rtsp_transport: Literal["tcp", "udp"] = Field(default="tcp")
    # rtspsrc / ffmpeg jitter buffer. 50 ms was flown; larger only if the wired link drops.
    rtsp_latency_ms: int = Field(default=50)
    # "av" decodes in-process with PyAV; "gst-nv" uses the Jetson NVDEC through gst-launch.
    decoder: Literal["av", "gst-nv"] = Field(default="av")
    expected_codec: str = Field(default="hevc")
    frame_id: str = Field(default="a8_optical")
    # Decoded frames are 2.8 MB each at 720p: cap them and keep them on-Jetson.
    color_hz: float = Field(default=10.0)
    jpeg_hz: float = Field(default=2.0)
    jpeg_quality: int = Field(default=50)
    jpeg_max_width: int = Field(default=640)
    # Seconds between the sensor exposure and our packet read (RTSP buffer + camera
    # encode). Measured in the bench gate; subtracted from the read time for the stamp.
    capture_latency_s: float = Field(default=0.08)
    # File replay: pace by the clip's frame rate and loop, so a 2 s capture stands in for a
    # live camera.
    replay_realtime: bool = Field(default=True)
    replay_loop: bool = Field(default=True)
    # SIYI SDK on port 37260: zoom and codec queries, and the sub-stream bitrate setter.
    sdk_enabled: bool = Field(default=False)
    sdk_ip: str = Field(default=A8_IP)
    # Stop publishing `video` when LinkPolicy says so; JPEG follows the policy's rate.
    obey_link_policy: bool = Field(default=True)
    sensor_stats_interval_s: float = Field(default=10.0)


def gst_nv_pipeline(url: str, latency_ms: int, width: int, height: int, hz: float) -> list[str]:
    """The Jetson decode pipeline: NVDEC, then fixed-size BGRx frames on stdout."""
    return [
        "gst-launch-1.0",
        "-q",
        "rtspsrc",
        f"location={url}",
        f"latency={latency_ms}",
        "!",
        "rtph265depay",
        "!",
        "h265parse",
        "!",
        "nvv4l2decoder",
        "!",
        "nvvidconv",
        "!",
        "videorate",
        "!",
        f"video/x-raw,format=BGRx,width={width},height={height},framerate={int(hz)}/1",
        "!",
        "fdsink",
        "fd=1",
    ]


class RtspCamera(Module):
    """RTSP H.265 camera relay: encoded passthrough, capped decoded frames, link JPEGs."""

    # Decoding 720p H.265 in software is a full core; keep it away from every other module.
    dedicated_worker = True

    config: RtspCameraConfig

    link_policy: In[LinkPolicy]

    video: Out[CompressedVideo]
    color_image: Out[Image]
    color_jpeg: Out[CompressedImage]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._stop_event = threading.Event()
        self._threads: list[threading.Thread] = []
        self._stats: dict[str, _StreamStat] = {}
        self._stats_lock = threading.Lock()
        self._policy_lock = threading.Lock()
        self._video_allowed = True
        self._policy_jpeg_hz: float | None = None
        self._last_color_pub = 0.0
        self._last_jpeg_pub = 0.0
        self._sdk: SiyiSdk | None = None
        self._gst: subprocess.Popen[bytes] | None = None
        self._frame_size: tuple[int, int] | None = None
        # What av.open gets: the URL, the file, or the generated clip once start() wrote it.
        self._source = self.config.url

    # Lifecycle

    @rpc
    def start(self) -> None:
        super().start()
        cfg = self.config
        if cfg.url == SYNTHETIC_URL:
            clip = Path(tempfile.mkdtemp(prefix="rtsp-synthetic-")) / "synthetic.mp4"
            write_synthetic_h265(clip, width=320, height=180, fps=25, seconds=2.0, centered=True)
            self._source = str(clip)
        if cfg.sdk_enabled:
            self._sdk = SiyiSdk(cfg.sdk_ip)
            self._sdk.open()
        self.register_disposable(Disposable(self.link_policy.subscribe(self._on_link_policy)))
        self._stop_event.clear()
        self._threads = [threading.Thread(target=self._relay_loop, name="rtsp-relay", daemon=True)]
        if cfg.decoder == "gst-nv":
            self._threads.append(
                threading.Thread(target=self._gst_loop, name="rtsp-gst-decode", daemon=True)
            )
        if cfg.sensor_stats_interval_s > 0:
            self._threads.append(
                threading.Thread(target=self._stats_report_loop, name="rtsp-stats", daemon=True)
            )
        for t in self._threads:
            t.start()

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        if self._gst is not None:
            self._gst.terminate()
        for t in self._threads:
            t.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        self._threads.clear()
        if self._gst is not None:
            self._gst.wait(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
            self._gst = None
        if self._sdk is not None:
            self._sdk.close()
            self._sdk = None
        super().stop()

    # Link policy

    def _on_link_policy(self, msg: LinkPolicy) -> None:
        if not self.config.obey_link_policy:
            return
        with self._policy_lock:
            self._video_allowed = msg.video_allowed
            self._policy_jpeg_hz = msg.jpeg_hz if msg.jpeg_hz > 0 else None

    def _jpeg_period(self) -> float:
        hz = self.config.jpeg_hz
        with self._policy_lock:
            if self._policy_jpeg_hz is not None:
                hz = min(hz, self._policy_jpeg_hz)
        return 1.0 / hz if hz > 0 else 0.0

    # Relay

    def _is_file(self) -> bool:
        return "://" not in self._source

    def _relay_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                self.relay_once()
            except (av.FFmpegError, OSError, IndexError, ValueError) as exc:
                self._count("video", errors=1)
                logger.warning("camera stream unavailable", url=self._source, error=str(exc))
                self._stop_event.wait(_RECONNECT_WAIT_S)
                continue
            if self._is_file() and not self.config.replay_loop:
                return

    def relay_once(self) -> int:
        """One pass over the stream (until it ends or stop is requested). Returns packets read."""
        cfg = self.config
        options: dict[str, str] = {}
        if not self._is_file():
            options = {
                "rtsp_transport": cfg.rtsp_transport,
                "fflags": "nobuffer",
                "max_delay": str(cfg.rtsp_latency_ms * 1000),
            }
        packets = 0
        decode_here = cfg.decoder == "av"
        # A frame comes out of the decoder from a later packet than its own (lookahead),
        # so stamps are kept by pts and handed to the frame they belong to.
        stamps: dict[int | None, float] = {}
        with av.open(self._source, options=options, timeout=(3.0, 3.0)) as container:
            stream = container.streams.video[0]
            if stream.codec_context.name != cfg.expected_codec:
                raise ValueError(f"expected {cfg.expected_codec}, got {stream.codec_context.name}")
            self._frame_size = (stream.codec_context.width, stream.codec_context.height)
            fps = float(stream.average_rate or 25)
            t_start = time.monotonic()
            for packet in container.demux(stream):
                if self._stop_event.is_set():
                    break
                if not packet.size or packet.is_corrupt:
                    self._count("video", dropped=1)
                    continue
                if self._is_file() and cfg.replay_realtime:
                    due = t_start + packets / fps
                    delay = due - time.monotonic()
                    if delay > 0:
                        time.sleep(delay)
                ts = time.time() - cfg.capture_latency_s
                packets += 1
                self._count("video", received=1, bytes_in=packet.size)
                with self._policy_lock:
                    allowed = self._video_allowed
                if allowed:
                    self.video.publish(
                        CompressedVideo(bytes(packet), format="h265", frame_id=cfg.frame_id, ts=ts)
                    )
                    self._count("video", published=1)
                else:
                    self._count("video", dropped=1)
                if decode_here:
                    stamps[packet.pts] = ts
                    for frame in stream.decode(packet):
                        self._publish_decoded(
                            frame.to_ndarray(format="rgb24"), stamps.pop(frame.pts, ts)
                        )
            if decode_here and not self._stop_event.is_set():
                # Flush the decoder's lookahead so a replayed clip yields every frame.
                flush_ts = time.time() - cfg.capture_latency_s
                for frame in stream.decode(None):
                    self._publish_decoded(
                        frame.to_ndarray(format="rgb24"), stamps.pop(frame.pts, flush_ts)
                    )
        return packets

    def _publish_decoded(self, rgb: Any, ts: float) -> None:
        cfg = self.config
        now = time.monotonic()
        self._count("color_image", received=1)
        color_period = 1.0 / cfg.color_hz if cfg.color_hz > 0 else 0.0
        jpeg_period = self._jpeg_period()
        want_color = color_period > 0 and now - self._last_color_pub >= color_period
        want_jpeg = jpeg_period > 0 and now - self._last_jpeg_pub >= jpeg_period
        if not (want_color or want_jpeg):
            self._count("color_image", dropped=1)
            return
        image = Image(data=rgb, format=ImageFormat.RGB, frame_id=cfg.frame_id, ts=ts)
        if want_color:
            self._last_color_pub = now
            self.color_image.publish(image)
            self._count("color_image", published=1)
        if want_jpeg:
            self._last_jpeg_pub = now
            try:
                self.color_jpeg.publish(
                    CompressedImage.from_image(
                        image, quality=cfg.jpeg_quality, max_width=cfg.jpeg_max_width
                    )
                )
                self._count("color_jpeg", published=1)
            except (ValueError, OSError):
                self._count("color_jpeg", errors=1)
                logger.exception("jpeg encode failed")

    def _gst_loop(self) -> None:
        """Jetson path: NVDEC in a gst-launch subprocess, fixed-size BGRx frames on its stdout."""
        cfg = self.config
        while not self._stop_event.is_set():
            size = self._frame_size
            if size is None:
                self._stop_event.wait(0.2)
                continue
            width, height = size
            nbytes = width * height * 4
            cmd = gst_nv_pipeline(self._source, cfg.rtsp_latency_ms, width, height, cfg.color_hz)
            self._gst = subprocess.Popen(cmd, stdout=subprocess.PIPE)
            assert self._gst.stdout is not None
            while not self._stop_event.is_set():
                raw = self._gst.stdout.read(nbytes)
                if len(raw) < nbytes:
                    break
                ts = time.time() - cfg.capture_latency_s
                import numpy as np  # heavy-ish; only the gst path needs it here

                bgrx = np.frombuffer(raw, dtype=np.uint8).reshape(height, width, 4)
                self._publish_decoded(bgrx[:, :, 2::-1], ts)
            self._gst.wait(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
            self._stop_event.wait(_RECONNECT_WAIT_S)

    # Diagnostics

    def _count(self, stream: str, **inc: int) -> None:
        with self._stats_lock:
            st = self._stats.setdefault(stream, _StreamStat())
            for k, v in inc.items():
                setattr(st, k, getattr(st, k) + v)
            st.last_mono = time.monotonic()

    def _stats_snapshot(self) -> dict[str, _StreamStat]:
        with self._stats_lock:
            return {k: replace(v) for k, v in self._stats.items()}

    def _stats_report_loop(self) -> None:
        interval = self.config.sensor_stats_interval_s
        prev = self._stats_snapshot()
        prev_t = time.monotonic()
        while not self._stop_event.wait(interval):
            cur = self._stats_snapshot()
            now = time.monotonic()
            dt = now - prev_t
            rates = {
                name: f"rx={(s.received - prev.get(name, _StreamStat()).received) / dt:.1f}/s "
                f"pub={(s.published - prev.get(name, _StreamStat()).published) / dt:.1f}/s"
                for name, s in sorted(cur.items())
            }
            logger.info("RtspCamera stream rates", window_s=round(dt), **rates)
            prev, prev_t = cur, now

    @rpc
    def sensor_stats(self) -> dict[str, Any]:
        """Per-stream cumulative counters and last-message age."""
        now = time.monotonic()
        return {
            name: {
                "received": s.received,
                "published": s.published,
                "dropped": s.dropped,
                "errors": s.errors,
                "bytes_in": s.bytes_in,
                "age_s": (now - s.last_mono if s.last_mono else -1.0),
            }
            for name, s in self._stats_snapshot().items()
        }

    @rpc
    def codec_specs(self) -> dict[str, Any]:
        """The camera's own main and sub stream codec settings (needs sdk_enabled)."""
        if self._sdk is None:
            return {"sdk": "disabled"}
        out: dict[str, Any] = {}
        for name, stream in (("main", STREAM_MAIN), ("sub", STREAM_SUB)):
            spec = self._sdk.query_codec(stream)
            out[name] = None if spec is None else vars(spec)
        return out

    @rpc
    def set_codec(
        self, stream: int, codec: int, width: int, height: int, bitrate_kbps: int
    ) -> bool:
        """Ask the camera to change a stream's codec, size and bitrate (needs sdk_enabled)."""
        if self._sdk is None:
            return False
        return self._sdk.set_codec(CodecSpec(stream, codec, width, height, bitrate_kbps))
