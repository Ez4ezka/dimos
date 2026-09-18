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

"""Recorded and synthetic ``gimbal_attitude`` streams for tests and gates without an A8.

The stream is what Px4DroneConnection publishes: a JointState with ``gimbal_roll``,
``gimbal_pitch``, ``gimbal_yaw`` in radians and the GIMBAL_DEVICE_FLAGS bits in
``effort``. A real capture is a JSONL file of ``{t, roll, pitch, yaw, flags}`` in degrees;
until one is recorded on the bench, :func:`synthetic_attitude_record` stands in.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path

from dimos.hardware.gimbal.siyi.frame import FLAG_YAW_IN_VEHICLE_FRAME, FLAG_YAW_LOCK
from dimos.msgs.sensor_msgs.JointState import JointState

GIMBAL_JOINTS = ("gimbal_roll", "gimbal_pitch", "gimbal_yaw")
A8_FOLLOW_FLAGS = FLAG_YAW_LOCK | FLAG_YAW_IN_VEHICLE_FRAME


@dataclass(frozen=True)
class AttitudeSample:
    t: float
    roll_deg: float
    pitch_deg: float
    yaw_deg: float
    flags: int = A8_FOLLOW_FLAGS
    failure_flags: int = 0

    def joint_state(self, frame_id: str = "gimbal_base") -> JointState:
        return JointState(
            ts=self.t,
            frame_id=frame_id,
            name=list(GIMBAL_JOINTS),
            position=[
                math.radians(self.roll_deg),
                math.radians(self.pitch_deg),
                math.radians(self.yaw_deg),
            ],
            velocity=[],
            effort=[float(self.flags), float(self.failure_flags), 0.0],
        )


def synthetic_attitude_record(
    *, t0: float = 1_700_000_000.0, hz: float = 10.0, n: int = 60
) -> list[AttitudeSample]:
    """Yaw sweep -60..60 deg with a pitch dip to -30 deg, the A8's 10 Hz, follow mode."""
    out = []
    for i in range(n):
        f = i / (n - 1)
        out.append(
            AttitudeSample(
                t=t0 + i / hz,
                roll_deg=0.0,
                pitch_deg=-30.0 * math.sin(math.pi * f),
                yaw_deg=-60.0 + 120.0 * f,
            )
        )
    return out


def load_attitude_record(path: Path) -> list[AttitudeSample]:
    """A bench capture: one JSON object per line, degrees, ``#`` lines ignored."""
    out = []
    for line in path.read_text().splitlines():
        if not line.strip() or line.startswith("#"):
            continue
        d = json.loads(line)
        out.append(
            AttitudeSample(
                t=float(d["t"]),
                roll_deg=float(d.get("roll", 0.0)),
                pitch_deg=float(d["pitch"]),
                yaw_deg=float(d["yaw"]),
                flags=int(d.get("flags", A8_FOLLOW_FLAGS)),
                failure_flags=int(d.get("failure_flags", 0)),
            )
        )
    return out
