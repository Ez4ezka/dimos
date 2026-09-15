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

"""SiyiA8Gimbal on the synthetic attitude record. No A8, no MAVLink, no transports."""

from __future__ import annotations

from collections.abc import Iterator
import math
from typing import Any

import pytest

from dimos.hardware.gimbal.siyi.gimbal import SiyiA8Gimbal
from dimos.hardware.gimbal.siyi.replay import (
    A8_FOLLOW_FLAGS,
    load_attitude_record,
    synthetic_attitude_record,
)
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3

_FORBIDDEN_RPCS = {"arm", "set_mode", "send_gimbal_pitchyaw", "claim_gimbal_control"}


@pytest.fixture
def gimbal() -> Iterator[tuple[SiyiA8Gimbal, dict[str, list[Any]]]]:
    g = SiyiA8Gimbal(aim_enabled=True)
    published: dict[str, list[Any]] = {"tf": [], "camera_info": [], "gimbal_target": []}
    for name, sink in published.items():
        getattr(g, name).publish = sink.append
    yield g, published
    g.stop()


def test_no_mavlink_and_no_actuating_rpc(gimbal: tuple[SiyiA8Gimbal, dict[str, list[Any]]]) -> None:
    g, _ = gimbal
    assert not (set(g.rpcs) & _FORBIDDEN_RPCS)
    assert set(g.inputs) == {"gimbal_attitude", "target_los"}
    assert set(g.outputs) == {"tf", "camera_info", "gimbal_target"}


def test_tf_chain_follows_the_recorded_attitude(
    gimbal: tuple[SiyiA8Gimbal, dict[str, list[Any]]],
) -> None:
    g, published = gimbal
    record = synthetic_attitude_record()
    sample = record[15]
    g._on_attitude(sample.joint_state())
    assert g.publish_transforms()
    (msg,) = published["tf"]
    edges = {(t.frame_id, t.child_frame_id): t for t in msg.transforms}
    assert set(edges) == {
        ("base_link", "gimbal_base"),
        ("gimbal_base", "gimbal_link"),
        ("gimbal_link", "a8_optical"),
    }
    link = edges[("gimbal_base", "gimbal_link")]
    euler = link.rotation.to_euler()
    # MAVLink yaw clockwise positive and pitch up positive become FLU's opposite signs.
    assert math.degrees(euler.z) == pytest.approx(-sample.yaw_deg, abs=0.05)
    assert math.degrees(euler.y) == pytest.approx(-sample.pitch_deg, abs=0.05)
    optical = edges[("gimbal_link", "a8_optical")]
    assert (optical.rotation.x, optical.rotation.y, optical.rotation.z, optical.rotation.w) == (
        -0.5,
        0.5,
        -0.5,
        0.5,
    )
    assert g.state()["flags"] == "YAW_LOCK|YAW_IN_VEHICLE_FRAME"


def test_stale_attitude_publishes_nothing(
    gimbal: tuple[SiyiA8Gimbal, dict[str, list[Any]]],
) -> None:
    g, published = gimbal
    assert not g.publish_transforms()
    g._on_attitude(synthetic_attitude_record()[0].joint_state())
    g._attitude.rx_mono -= 5.0  # older than attitude_max_age_s
    assert not g.publish_transforms()
    assert published["tf"] == []


def test_camera_info_is_the_a8_main_stream_and_gated_by_zoom(
    gimbal: tuple[SiyiA8Gimbal, dict[str, list[Any]]],
) -> None:
    g, published = gimbal
    assert g.publish_camera_info()
    (info,) = published["camera_info"]
    assert (info.width, info.height, info.frame_id) == (1280, 720, "a8_optical")
    assert info.K[0] == pytest.approx(749.3) and info.K[2] == pytest.approx(640.0)
    g._zoom = 2.0
    assert not g.publish_camera_info()
    assert len(published["camera_info"]) == 1


def test_aim_from_line_of_sight_is_clamped_and_rate_limited(
    gimbal: tuple[SiyiA8Gimbal, dict[str, list[Any]]],
) -> None:
    g, published = gimbal
    # Target 150 deg to the left (FLU counter-clockwise) and 10 deg below the horizon.
    los = PoseStamped(
        ts=1.0,
        frame_id="base_link",
        position=Vector3(),
        orientation=Quaternion.from_euler(Vector3(0.0, math.radians(10.0), math.radians(150.0))),
    )
    g._on_target_los(los)
    (target,) = published["gimbal_target"]
    pitch, yaw = (math.degrees(p) for p in target.position)
    assert target.name == ["gimbal_pitch", "gimbal_yaw"]
    assert pitch == pytest.approx(-10.0, abs=0.05)  # nose-down LOS -> gimbal pitch down
    assert yaw == pytest.approx(-120.0, abs=0.05)  # left of the nose, clamped at the yaw limit
    g._on_target_los(los)  # inside the 10 Hz window: not sent again
    assert len(published["gimbal_target"]) == 1


def test_aim_disabled_publishes_no_target() -> None:
    g = SiyiA8Gimbal(aim_enabled=False)
    sent: list[Any] = []
    g.gimbal_target.publish = sent.append
    try:
        g._on_target_los(PoseStamped(ts=1.0, frame_id="base_link"))
        assert sent == []
        assert g.aim(-10.0, 20.0)  # the manual RPC still publishes a request
        assert len(sent) == 1
    finally:
        g.stop()


def test_record_loader_reads_a_bench_capture(tmp_path: Any) -> None:
    path = tmp_path / "gimbal_attitude.jsonl"
    path.write_text(
        '# recorded\n{"t": 1.0, "roll": 0.0, "pitch": -12.5, "yaw": 33.0, "flags": 48}\n'
    )
    (sample,) = load_attitude_record(path)
    assert (sample.pitch_deg, sample.yaw_deg, sample.flags) == (-12.5, 33.0, A8_FOLLOW_FLAGS)
