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

"""Perception gate: the chain on a replayed clip beside the SITL connection and a fake A8.

Start ``make px4_sitl gz_x500`` in the PX4 tree, then::

    python dimos/robot/px4/tool_perception_gate.py

Replays a synthetic clip with the target parked at the image centre through
px4-sitl-perception with the blob detector, selects the first track, and asserts: a track
is confirmed within N frames, ``target_valid`` goes true, and the line-of-sight azimuth
agrees with the fake gimbal's yaw (plus the vehicle heading) within 2 degrees. The
aircraft on the ground gives no altitude, so the estimator uses a fixed 10 m AGL the way
the flown bench runs did (``--agl fixed``). PASS or FAIL with numbers.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import tempfile
import threading
import time

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.hardware.sensors.camera.rtsp.synthetic import write_synthetic_h265
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.vision_msgs.Detection2DArray import Detection2DArray
from dimos.robot.px4.blueprints.perception.px4_sitl_perception import px4_sitl_perception
from dimos.robot.px4.perception.tracker import MIN_HITS
from dimos.robot.px4.perception_bridge import PerceptionBridge
from dimos.robot.px4.sitl.fake_a8 import FakeA8

_CONFIRM_WITHIN_FRAMES = MIN_HITS + 5
_AZIMUTH_TOL_DEG = 2.0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=10.0)
    args = ap.parse_args()

    with tempfile.TemporaryDirectory() as d:
        clip = Path(d) / "synthetic_centered.mp4"
        write_synthetic_h265(clip, width=320, height=180, fps=25, seconds=2.0, centered=True)
        parsed = BlueprintConfigParser(px4_sitl_perception).parse(
            environ={},
            overrides={
                "g": {"viewer": "none"},
                "rtspcamera": {"url": str(clip), "color_hz": 25.0},
                "perceptionbridge": {
                    "detector": "blob",
                    "estimator": {"agl_source": "fixed", "fixed_agl_m": 10.0},
                },
            },
        )
        coordinator = ModuleCoordinator.build(px4_sitl_perception, parsed)
        ok = True
        lock = threading.Lock()
        frames_until_track = [-1]
        frames_seen = [0]
        latest_odom_yaw = [math.nan]
        los_yaws: list[float] = []

        def on_tracks(msg: Detection2DArray) -> None:
            with lock:
                frames_seen[0] += 1
                if msg.detections_length > 0 and frames_until_track[0] < 0:
                    frames_until_track[0] = frames_seen[0]

        def on_odom(msg: Odometry) -> None:
            with lock:
                latest_odom_yaw[0] = msg.orientation.to_euler().z

        def on_los(msg: PoseStamped) -> None:
            with lock:
                los_yaws.append(msg.yaw)

        try:
            unsubs = [
                coordinator.transports[("tracks", Detection2DArray)].subscribe(on_tracks),
                coordinator.transports[("odometry", Odometry)].subscribe(on_odom),
                coordinator.transports[("target_los", PoseStamped)].subscribe(on_los),
            ]
            bridge = coordinator.get_instance(PerceptionBridge)
            a8 = coordinator.get_instance(FakeA8)
            deadline = time.time() + args.seconds
            while time.time() < deadline:
                with lock:
                    if frames_until_track[0] > 0:
                        break
                time.sleep(0.2)
            with lock:
                first = frames_until_track[0]
            print(f"track confirmed after {first} frames (limit {_CONFIRM_WITHIN_FRAMES})")
            ok &= 0 < first <= _CONFIRM_WITHIN_FRAMES
            print("select:", bridge.select_track(1))
            time.sleep(2.0)
            status = bridge.status()
            for u in unsubs:
                u()
            with lock:
                yaws = list(los_yaws)
                veh_yaw_flu = latest_odom_yaw[0]
            print("target:", status["target"])
            print("los:", status["los"])
            ok &= bool(status["target"] and status["target"]["valid"])
            gimbal_yaw = a8.attitude()["yaw"]
            if yaws and not math.isnan(veh_yaw_flu):
                # LOS pose yaw is the gimbal body yaw as an FLU angle; azimuth = heading + gimbal yaw.
                azimuth = (-math.degrees(veh_yaw_flu) - math.degrees(yaws[-1])) % 360.0
                expected = (-math.degrees(veh_yaw_flu) + gimbal_yaw) % 360.0
                err = abs((azimuth - expected + 180.0) % 360.0 - 180.0)
                print(
                    f"azimuth from target_los {azimuth:.2f} deg vs gimbal+heading {expected:.2f} deg: err {err:.2f}"
                )
                ok &= err <= _AZIMUTH_TOL_DEG
                reported = status["los"]["azimuth_deg"] if status["los"] else None
                if reported is not None:
                    err2 = abs((reported - expected + 180.0) % 360.0 - 180.0)
                    print(f"azimuth reported by the bridge {reported:.2f} deg: err {err2:.2f}")
                    ok &= err2 <= _AZIMUTH_TOL_DEG
            else:
                print("no target_los received")
                ok = False
        finally:
            coordinator.stop()
    print("GATE", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
