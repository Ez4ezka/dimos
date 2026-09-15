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

from __future__ import annotations

import struct

from dimos.hardware.gimbal.siyi.sdk import (
    CMD_ATTITUDE,
    CMD_CODEC_GET,
    CODEC_H265,
    STREAM_SUB,
    CodecSpec,
    build_packet,
    crc16,
    parse_attitude,
    parse_packet,
    parse_zoom,
)


def test_packet_roundtrip_and_crc() -> None:
    pkt = build_packet(CMD_ATTITUDE, seq=7)
    assert pkt[:3] == b"\x55\x66\x01"
    assert parse_packet(pkt) == (CMD_ATTITUDE, b"")
    corrupted = pkt[:-1] + bytes([pkt[-1] ^ 0xFF])
    assert parse_packet(corrupted) is None
    assert crc16(pkt[:-2]) == struct.unpack("<H", pkt[-2:])[0]


def test_attitude_and_zoom_parsing() -> None:
    body = struct.pack("<hhhhhh", -438, 200, 1800, 0, 0, 0)
    assert parse_attitude(body) == (-43.8, 20.0, 180.0)  # SIYI yaw sign is opposite MAVLink
    assert parse_zoom(bytes([2, 5])) == 2.5


def test_codec_spec_payload_roundtrip() -> None:
    spec = CodecSpec(stream=STREAM_SUB, codec=CODEC_H265, width=640, height=360, bitrate_kbps=400)
    pkt = build_packet(CMD_CODEC_GET, spec.payload())
    cmd, body = parse_packet(pkt) or (None, b"")
    assert cmd == CMD_CODEC_GET
    assert CodecSpec.from_body(body) == spec
