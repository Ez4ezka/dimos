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

"""N2 gate: camera and gimbal beside the connection in SITL, with a replayed clip and a fake A8.

Start ``make px4_sitl gz_x500`` in the PX4 tree, then::

    python dimos/robot/px4/tool_bench_gate.py

Asserts that ``tf`` carries the gimbal chain, that frames are stamped within 50 ms of the
vehicle's odometry clock, and that an aim request changes the reported attitude (through
FakeA8 here; through PX4's gimbal manager on the aircraft). PASS or FAIL with numbers.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import statistics
import tempfile
import threading
import time

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.hardware.gimbal.siyi.gimbal import SiyiA8Gimbal
from dimos.hardware.sensors.camera.rtsp.synthetic import write_synthetic_h265
from dimos.msgs.foxglove_msgs.CompressedVideo import CompressedVideo
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.robot.px4.blueprints.bench.px4_sitl_bench import px4_sitl_bench
from dimos.robot.px4.sitl.fake_a8 import FakeA8

_STAMP_BOUND_MS = 50.0
# FakeTarget reports its line of sight this far right of the nose; the gimbal's aim path
# (target_los -> gimbal_target -> FakeA8 -> gimbal_attitude) must bring the A8 there.
_TARGET_YAW_DEG = 30.0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=12.0)
    args = ap.parse_args()

    with tempfile.TemporaryDirectory() as d:
        clip = Path(d) / "synthetic.mp4"
        write_synthetic_h265(clip, width=320, height=180, fps=25, seconds=2.0)
        parsed = BlueprintConfigParser(px4_sitl_bench).parse(
            environ={},
            overrides={
                "g": {"viewer": "none"},
                "rtspcamera": {"url": str(clip)},
                "faketarget": {"gimbal_yaw_deg": _TARGET_YAW_DEG},
            },
        )
        coordinator = ModuleCoordinator.build(px4_sitl_bench, parsed)
        ok = True
        lock = threading.Lock()
        edges: set[tuple[str, str]] = set()
        latest_odom_ts = [0.0]
        frame_lag_ms: list[float] = []

        def on_tf(msg: TFMessage) -> None:
            with lock:
                for t in msg.transforms:
                    edges.add((t.frame_id, t.child_frame_id))

        def on_odom(msg: Odometry) -> None:
            with lock:
                latest_odom_ts[0] = msg.ts

        def on_video(msg: CompressedVideo) -> None:
            with lock:
                if latest_odom_ts[0]:
                    frame_lag_ms.append((latest_odom_ts[0] - msg.ts) * 1e3)

        try:
            unsubs = [
                coordinator.transports[("tf", TFMessage)].subscribe(on_tf),
                coordinator.transports[("odometry", Odometry)].subscribe(on_odom),
                coordinator.transports[("video", CompressedVideo)].subscribe(on_video),
            ]
            time.sleep(args.seconds)
            gimbal = coordinator.get_instance(SiyiA8Gimbal)
            a8 = coordinator.get_instance(FakeA8)
            after = a8.attitude()
            state = gimbal.state()
            reported = state["attitude"]
            for u in unsubs:
                u()
            with lock:
                seen = set(edges)
                lags = list(frame_lag_ms)

            chain = {
                ("base_link", "gimbal_base"),
                ("gimbal_base", "gimbal_link"),
                ("gimbal_link", "a8_optical"),
            }
            print("tf edges:", sorted(seen))
            print("gimbal chain present:", chain <= seen)
            ok &= chain <= seen
            if lags:
                med = statistics.median(lags)
                print(
                    f"frame stamp vs vehicle odometry clock: median {med:.1f} ms, "
                    f"max {max(abs(v) for v in lags):.1f} ms over {len(lags)} frames"
                )
                ok &= abs(med) <= _STAMP_BOUND_MS
            else:
                print("no frames received")
                ok = False
            print(
                f"aim requests sent: {state['aim_sent']}  fake A8 at: {after}  "
                f"gimbal reports: {reported}  wanted yaw {_TARGET_YAW_DEG}"
            )
            ok &= state["aim_sent"] > 0
            ok &= abs(after["yaw"] - _TARGET_YAW_DEG) < 3.0
            ok &= reported is not None and abs(reported["yaw"] - _TARGET_YAW_DEG) < 3.0
            print("camera stats:", coordinator.get_instance("rtspcamera").sensor_stats())
        finally:
            coordinator.stop()
    print("GATE", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
