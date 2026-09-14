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

"""Scripted target for SITL FOLLOW and YAW_TRACK tests: a line, a circle, and drop windows.

Ported from drone-autonomy ``sitl/fake_target.py`` (UDP JSON) into a module that
publishes the same three things the perception bridge will: ``target_state``
(Odometry in the ``odom`` frame), ``target_valid`` and ``target_los``.
"""

from __future__ import annotations

import math
import threading
import time
from typing import Any

from dimos_lcm.std_msgs import Bool  # type: ignore[import-untyped]
from pydantic import Field

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.robot.px4.mavlink.frames import ned_to_flu


class FakeTargetConfig(ModuleConfig):
    # Straight line: start (NED metres) and velocity (m/s).
    n: float = Field(default=15.0)
    e: float = Field(default=0.0)
    vn: float = Field(default=0.0)
    ve: float = Field(default=0.0)
    # Circle around the origin instead of the line (radius in metres, 0 = line).
    circle_radius_m: float = Field(default=0.0)
    circle_period_s: float = Field(default=60.0)
    # Simulated target loss window (seconds after start; 0 = never).
    drop_after_s: float = Field(default=0.0)
    drop_for_s: float = Field(default=5.0)
    # Faked gimbal yaw (body, clockwise positive, degrees) for YAW_TRACK tests.
    gimbal_yaw_deg: float = Field(default=0.0)
    rate_hz: float = Field(default=20.0)
    frame_id: str = Field(default="odom")


class FakeTarget(Module):
    config: FakeTargetConfig

    target_state: Out[Odometry]
    target_valid: Out[Bool]
    target_los: Out[PoseStamped]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, name="fake-target", daemon=True)
        self._thread.start()

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
            self._thread = None
        super().stop()

    def _run(self) -> None:
        cfg = self.config
        t0 = time.time()
        period = 1.0 / cfg.rate_hz
        while not self._stop_event.is_set():
            now = time.time()
            t = now - t0
            if cfg.circle_radius_m > 0:
                w = 2 * math.pi / cfg.circle_period_s
                r = cfg.circle_radius_m
                n, e = r * math.cos(w * t), r * math.sin(w * t)
                vn, ve = -r * w * math.sin(w * t), r * w * math.cos(w * t)
            else:
                n, e, vn, ve = cfg.n + cfg.vn * t, cfg.e + cfg.ve * t, cfg.vn, cfg.ve
            valid = not (
                cfg.drop_after_s > 0 and cfg.drop_after_s <= t < cfg.drop_after_s + cfg.drop_for_s
            )
            x, y, _ = ned_to_flu(n, e, 0.0)
            vx, vy, _ = ned_to_flu(vn, ve, 0.0)
            self.target_state.publish(
                Odometry(
                    ts=now,
                    frame_id=cfg.frame_id,
                    child_frame_id="target",
                    pose=Pose(Vector3(x, y, 0.0), Quaternion()),
                    twist=Twist(Vector3(vx, vy, 0.0), Vector3()),
                )
            )
            self.target_valid.publish(Bool(data=valid))
            if valid:
                yaw_flu = -math.radians(cfg.gimbal_yaw_deg)
                self.target_los.publish(
                    PoseStamped(
                        ts=now,
                        frame_id="base_link",
                        position=Vector3(),
                        orientation=Quaternion.from_euler(Vector3(0.0, 0.0, yaw_flu)),
                    )
                )
            self._stop_event.wait(period)
