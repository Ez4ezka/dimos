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

"""SITL gate: the whole simulator twin (``px4-sitl``) checked end to end. PASS or FAIL with numbers.

Start ``make px4_sitl gz_x500`` in the PX4 tree, then::

    python dimos/robot/px4/tool_sitl_gate.py          # telemetry, camera, gimbal, perception, link
    python dimos/robot/px4/tool_sitl_gate.py --fly    # + takeoff, hover, land, scored by the tracker

Asserts: odometry at 25 Hz or better with the reader-side stamp lag within 50 ms; ``tf``
carries the gimbal chain; frames stamped within 50 ms of the vehicle's odometry clock; a
track confirmed within MIN_HITS + 5 frames, a valid target after selection, and the
line-of-sight azimuth within 2 deg of gimbal yaw plus heading; the aim path moved the fake
A8 to where the gimbal module reports it; the replayed link allows video. With ``--fly``:
HOVER within 60 s, IDLE after land, and the command tracker scores both ``ok``.
"""

from __future__ import annotations

import argparse
import math
import statistics
import threading
import time
from typing import Any

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.hardware.gimbal.siyi.gimbal import SiyiA8Gimbal
from dimos.msgs.foxglove_msgs.CompressedVideo import CompressedVideo
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.link_msgs.LinkPolicy import LinkPolicy
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.msgs.vision_msgs.Detection2DArray import Detection2DArray
from dimos.robot.px4.blueprints import px4_sitl
from dimos.robot.px4.command_tracker import CommandTracker
from dimos.robot.px4.connection import Px4DroneConnection
from dimos.robot.px4.link_monitor import LinkMonitor
from dimos.robot.px4.perception.bridge import PerceptionBridge
from dimos.robot.px4.perception.tracker import MIN_HITS
from dimos.robot.px4.sitl import FakeA8

_STAMP_BOUND_MS = 50.0
_CONFIRM_WITHIN_FRAMES = MIN_HITS + 5
_AZIMUTH_TOL_DEG = 2.0
_GIMBAL_TOL_DEG = 3.0
_CHAIN = {
    ("base_link", "gimbal_base"),
    ("gimbal_base", "gimbal_link"),
    ("gimbal_link", "a8_optical"),
}


def _fake_gcs(stop: threading.Event) -> None:
    """Stand in for QGroundControl on SITL's normal MAVLink instance (port 14550).

    PX4's arming check refuses without a ground station; in the field QGC is always
    connected. SITL only: system 255 is exactly what Px4DroneConnection must never be.
    """
    from pymavlink import mavutil

    gcs = mavutil.mavlink_connection("udpin:0.0.0.0:14550", source_system=255)
    while not stop.is_set():
        gcs.recv_match(blocking=True, timeout=0.2)
        gcs.mav.heartbeat_send(6, 8, 0, 0, 4)  # MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, ACTIVE
        stop.wait(0.8)
    gcs.close()


def _wait_state(drone: Any, wanted: set[str], timeout_s: float) -> str:
    deadline = time.time() + timeout_s
    state = ""
    while time.time() < deadline:
        state = drone.status()["state"]
        if state in wanted:
            return state
        time.sleep(0.25)
    return state


def _wrap(deg: float) -> float:
    return (deg + 180.0) % 360.0 - 180.0


class _Taps:
    """What the gate listens to while the twin runs."""

    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.odom_stamps: list[tuple[float, float]] = []
        self.latest_odom_ts = 0.0
        self.latest_odom_yaw_flu = math.nan
        self.edges: set[tuple[str, str]] = set()
        self.frame_lag_ms: list[float] = []
        self.track_frames = 0
        self.frames_until_track = -1
        self.los_yaws: list[float] = []
        self.policy: LinkPolicy | None = None

    def on_odom(self, msg: Odometry) -> None:
        with self.lock:
            self.odom_stamps.append((msg.ts, time.time()))
            self.latest_odom_ts = msg.ts
            self.latest_odom_yaw_flu = msg.orientation.to_euler().z

    def on_tf(self, msg: TFMessage) -> None:
        with self.lock:
            for t in msg.transforms:
                self.edges.add((t.frame_id, t.child_frame_id))

    def on_video(self, msg: CompressedVideo) -> None:
        with self.lock:
            if self.latest_odom_ts:
                self.frame_lag_ms.append((self.latest_odom_ts - msg.ts) * 1e3)

    def on_tracks(self, msg: Detection2DArray) -> None:
        with self.lock:
            self.track_frames += 1
            if msg.detections_length > 0 and self.frames_until_track < 0:
                self.frames_until_track = self.track_frames

    def on_los(self, msg: PoseStamped) -> None:
        with self.lock:
            self.los_yaws.append(msg.yaw)

    def on_policy(self, msg: LinkPolicy) -> None:
        with self.lock:
            self.policy = msg


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=15.0, help="passive listening window")
    ap.add_argument("--fly", action="store_true", help="sitl_enable + takeoff + hover + land")
    args = ap.parse_args()

    gcs_stop = threading.Event()
    gcs = threading.Thread(target=_fake_gcs, args=(gcs_stop,), name="fake-gcs", daemon=True)
    gcs.start()
    parsed = BlueprintConfigParser(px4_sitl).parse(environ={}, overrides={"g": {"viewer": "none"}})
    coordinator = ModuleCoordinator.build(px4_sitl, parsed)
    taps = _Taps()
    ok = True
    try:
        unsubs = [
            coordinator.transports[("odometry", Odometry)].subscribe(taps.on_odom),
            coordinator.transports[("tf", TFMessage)].subscribe(taps.on_tf),
            coordinator.transports[("video", CompressedVideo)].subscribe(taps.on_video),
            coordinator.transports[("tracks", Detection2DArray)].subscribe(taps.on_tracks),
            coordinator.transports[("target_los", PoseStamped)].subscribe(taps.on_los),
            coordinator.transports[("link_policy", LinkPolicy)].subscribe(taps.on_policy),
        ]
        drone = coordinator.get_instance(Px4DroneConnection)
        bridge = coordinator.get_instance(PerceptionBridge)
        gimbal = coordinator.get_instance(SiyiA8Gimbal)
        a8 = coordinator.get_instance(FakeA8)
        link = coordinator.get_instance(LinkMonitor)
        tracker = coordinator.get_instance(CommandTracker)

        time.sleep(args.seconds)
        # Select the synthetic target: from here the line of sight is valid and the gimbal
        # module aims at it (through FakeA8, which is already pointing there).
        print("select:", bridge.select_track(1))
        time.sleep(2.0)
        for u in unsubs:
            u()

        with taps.lock:
            samples = list(taps.odom_stamps)
            edges = set(taps.edges)
            lags = list(taps.frame_lag_ms)
            first_track = taps.frames_until_track
            los_yaws = list(taps.los_yaws)
            veh_yaw_flu = taps.latest_odom_yaw_flu
            policy = taps.policy

        # 1. Telemetry.
        stats = drone.sensor_stats()
        hz = len(samples) / args.seconds
        print(f"odometry: {len(samples)} msgs in {args.seconds:.0f}s = {hz:.1f} Hz")
        print(f"timebase: {stats['timebase']}")
        print(f"tick jitter ms: {stats['tick_jitter_ms']}")
        ok &= hz >= 25.0
        reader_lag = stats["timebase"]["stamp_lag_ms_p50"]
        ok &= reader_lag is not None and abs(reader_lag) <= _STAMP_BOUND_MS

        # 2. Camera and gimbal.
        print("gimbal chain in tf:", _CHAIN <= edges, sorted(edges))
        ok &= _CHAIN <= edges
        if lags:
            med = statistics.median(lags)
            print(f"frame stamp vs vehicle clock: median {med:.1f} ms over {len(lags)} frames")
            ok &= abs(med) <= _STAMP_BOUND_MS
        else:
            print("no video frames received")
            ok = False
        state = gimbal.state()
        reported, fake = state["attitude"], a8.attitude()
        print(f"aim requests: {state['aim_sent']}  fake A8: {fake}  gimbal reports: {reported}")
        ok &= state["aim_sent"] > 0 and reported is not None
        if reported is not None:
            ok &= abs(_wrap(reported["yaw"] - fake["yaw"])) < _GIMBAL_TOL_DEG
            ok &= abs(reported["pitch"] - fake["pitch"]) < _GIMBAL_TOL_DEG

        # 3. Perception.
        print(f"track confirmed after {first_track} frames (limit {_CONFIRM_WITHIN_FRAMES})")
        ok &= 0 < first_track <= _CONFIRM_WITHIN_FRAMES
        status = bridge.status()
        print("target:", status["target"])
        ok &= bool(status["target"] and status["target"]["valid"])
        if los_yaws and not math.isnan(veh_yaw_flu):
            # LOS pose yaw is the gimbal body yaw as an FLU angle; azimuth = heading + gimbal yaw.
            azimuth = (-math.degrees(veh_yaw_flu) - math.degrees(los_yaws[-1])) % 360.0
            expected = (-math.degrees(veh_yaw_flu) + fake["yaw"]) % 360.0
            err = abs(_wrap(azimuth - expected))
            print(
                f"azimuth from target_los {azimuth:.2f} vs gimbal+heading {expected:.2f}: err {err:.2f}"
            )
            ok &= err <= _AZIMUTH_TOL_DEG
        else:
            print("no target_los received")
            ok = False

        # 4. Link.
        print("link:", link.path(), "policy:", None if policy is None else policy.reason)
        ok &= policy is not None and policy.video_allowed

        # 5. Flight, scored by the tracker.
        if args.fly:
            print("sitl_enable:", drone.sitl_enable(True))
            print("takeoff:", drone.takeoff())
            st_name = _wait_state(drone, {"HOVER", "IDLE", "ABORT"}, timeout_s=60.0)
            st = drone.status()
            print(f"after takeoff: state={st_name} reason={st['reason']!r} alt_m={st.get('alt_m')}")
            ok &= st_name == "HOVER"
            time.sleep(5.0)
            print("land:", drone.land())
            st_name = _wait_state(drone, {"IDLE", "ABORT"}, timeout_s=60.0)
            print(f"after land: state={st_name} reason={drone.status()['reason']!r}")
            print(f"tick jitter ms (flight): {drone.sensor_stats()['tick_jitter_ms']}")
            ok &= st_name == "IDLE"
            time.sleep(1.0)  # the tracker closes `land` once the setpoint stream has ended
            verdicts = {tc["command"]: tc["verdict"] for tc in tracker.recent()}
            print("tracker verdicts:", verdicts)
            ok &= verdicts.get("takeoff") == "ok" and verdicts.get("land") == "ok"
        print("camera stats:", coordinator.get_instance("rtspcamera").sensor_stats())
    finally:
        coordinator.stop()
        gcs_stop.set()
        gcs.join(timeout=2.0)
    print("GATE", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
