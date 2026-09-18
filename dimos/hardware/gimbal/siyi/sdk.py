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

"""SIYI gimbal-camera Ethernet SDK: the UDP protocol on port 37260 (zoom, attitude, codec).

Ported from drone-autonomy ``common/gimbal.py:116-177`` (SiyiCamera, known good
2026-09-02) and extended with the codec-spec commands (0x20 request, 0x21 set) that let
the camera itself emit a lower-bitrate stream, so the aircraft never re-encodes for the
operator link. Packet building and parsing are pure; :class:`SiyiSdk` owns the socket.

Aim commands do not go through this SDK on the aircraft: PX4 is the gimbal manager and
the MAVLink path through Px4DroneConnection is the one writer.
"""

from __future__ import annotations

from dataclasses import dataclass
import socket
import struct
import time

A8_IP = "192.168.144.25"
A8_SDK_PORT = 37260

CMD_ATTITUDE = 0x0D
CMD_ZOOM_MULTIPLE = 0x18
CMD_ZOOM = 0x05
CMD_CODEC_GET = 0x20
CMD_CODEC_SET = 0x21

STREAM_MAIN = 0
STREAM_SUB = 1
CODEC_H264 = 1
CODEC_H265 = 2


def crc16(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def build_packet(cmd: int, payload: bytes = b"", seq: int = 0) -> bytes:
    pkt = bytearray(b"\x55\x66\x01") + struct.pack("<H", len(payload)) + struct.pack("<H", seq)
    pkt += bytes([cmd]) + payload
    return bytes(pkt + struct.pack("<H", crc16(bytes(pkt))))


def parse_packet(data: bytes) -> tuple[int, bytes] | None:
    """``(cmd, body)`` of a well-formed reply, else None (bad header, length or CRC)."""
    if len(data) < 10 or data[:2] != b"\x55\x66":
        return None
    n = int.from_bytes(data[3:5], "little")
    if len(data) < 8 + n + 2:
        return None
    if struct.unpack("<H", data[8 + n : 10 + n])[0] != crc16(data[: 8 + n]):
        return None
    return data[7], data[8 : 8 + n]


@dataclass(frozen=True)
class CodecSpec:
    """One stream's codec settings as the camera reports or accepts them."""

    stream: int  # STREAM_MAIN or STREAM_SUB
    codec: int  # CODEC_H264 or CODEC_H265
    width: int
    height: int
    bitrate_kbps: int

    def payload(self) -> bytes:
        return struct.pack(
            "<BBHHHB", self.stream, self.codec, self.width, self.height, self.bitrate_kbps, 0
        )

    @classmethod
    def from_body(cls, body: bytes) -> CodecSpec:
        stream, codec, w, h, kbps, _ = struct.unpack("<BBHHHB", body[:9])
        return cls(stream=stream, codec=codec, width=w, height=h, bitrate_kbps=kbps)


def parse_attitude(body: bytes) -> tuple[float, float, float]:
    """``(yaw, pitch, roll)`` degrees per SIYI 0x0D. Yaw has the opposite sign to MAVLink."""
    yaw, pitch, roll, _, _, _ = struct.unpack("<hhhhhh", body[:12])
    return yaw / 10.0, pitch / 10.0, roll / 10.0


def parse_zoom(body: bytes) -> float:
    return float(body[0]) + float(body[1]) / 10.0


class SiyiSdk:
    """Minimal SDK client. ``open()`` connects the UDP socket; every query is bounded."""

    def __init__(self, ip: str = A8_IP, port: int = A8_SDK_PORT, timeout_s: float = 0.15) -> None:
        self._addr = (ip, port)
        self._timeout_s = timeout_s
        self._sock: socket.socket | None = None
        self._seq = 0

    def open(self) -> None:
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.settimeout(self._timeout_s)
        self._sock.connect(self._addr)

    def close(self) -> None:
        if self._sock is not None:
            self._sock.close()
            self._sock = None

    def _query(
        self, cmd: int, min_len: int, payload: bytes = b"", deadline_s: float = 0.3
    ) -> bytes | None:
        sock = self._sock
        if sock is None:
            raise RuntimeError("SiyiSdk not open")
        seq, self._seq = self._seq, (self._seq + 1) & 0xFFFF
        sock.send(build_packet(cmd, payload, seq))
        end = time.monotonic() + deadline_s
        while time.monotonic() < end:
            try:
                data = sock.recv(1024)
            except TimeoutError:
                continue
            parsed = parse_packet(data)
            if parsed is None or parsed[0] != cmd:
                continue
            if len(parsed[1]) >= min_len:
                return parsed[1]
        return None

    def query_zoom(self) -> float | None:
        body = self._query(CMD_ZOOM_MULTIPLE, 2)
        return None if body is None else parse_zoom(body)

    def query_attitude(self) -> tuple[float, float, float] | None:
        body = self._query(CMD_ATTITUDE, 12)
        return None if body is None else parse_attitude(body)

    def query_codec(self, stream: int) -> CodecSpec | None:
        body = self._query(CMD_CODEC_GET, 9, bytes([stream]))
        return None if body is None else CodecSpec.from_body(body)

    def set_codec(self, spec: CodecSpec) -> bool:
        body = self._query(CMD_CODEC_SET, 1, spec.payload())
        return body is not None and body[0] == 1
