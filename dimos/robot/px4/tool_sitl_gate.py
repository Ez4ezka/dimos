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

"""N0 gate against PX4 SITL: odometry rate, stamp offset to the laptop clock, tick jitter.

Start ``make px4_sitl gz_x500`` in the PX4 tree first, then::

    python dimos/robot/px4/tool_sitl_gate.py            # 20 s of telemetry, no flight
    python dimos/robot/px4/tool_sitl_gate.py --fly      # + sitl_enable, takeoff, hover, land

Prints the numbers the kickoff asks for before N1: odometry Hz, |stamp - wall| in ms,
timebase quality, tick jitter p50/p99/max.
"""

from __future__ import annotations

import argparse
import statistics
import threading
import time
from typing import Any

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.robot.px4.blueprints.basic.px4_sitl import px4_sitl
from dimos.robot.px4.connection import Px4DroneConnection


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


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--fly", action="store_true", help="sitl_enable + takeoff + hover + land")
    args = ap.parse_args()

    gcs_stop = threading.Event()
    gcs = threading.Thread(target=_fake_gcs, args=(gcs_stop,), name="fake-gcs", daemon=True)
    gcs.start()
    parsed = BlueprintConfigParser(px4_sitl).parse(environ={}, overrides={"g": {"viewer": "none"}})
    coordinator = ModuleCoordinator.build(px4_sitl, parsed)
    ok = True
    try:
        drone = coordinator.get_instance(Px4DroneConnection)

        stamps: list[tuple[float, float]] = []
        lock = threading.Lock()

        def on_odom(msg: Odometry) -> None:
            with lock:
                stamps.append((msg.ts, time.time()))

        unsub = coordinator.transports[("odometry", Odometry)].subscribe(on_odom)
        time.sleep(args.seconds)
        unsub()
        with lock:
            samples = list(stamps)

        hz = len(samples) / args.seconds
        offsets_ms = [(wall - ts) * 1e3 for ts, wall in samples]
        status = drone.status()
        stats = drone.sensor_stats()
        print(f"odometry: {len(samples)} msgs in {args.seconds:.0f}s = {hz:.1f} Hz")
        if offsets_ms:
            print(
                f"stamp lag vs laptop clock: median {statistics.median(offsets_ms):.1f} ms, "
                f"max {max(offsets_ms):.1f} ms"
            )
        print(f"timebase: {stats['timebase']}")
        print(f"tick jitter ms: {stats['tick_jitter_ms']}")
        print(f"state: {status['state']} px4_mode: {status['px4_mode']} armed: {status['armed']}")
        rates = {k: v["received"] / args.seconds for k, v in stats["messages"].items()}
        print("message rates Hz:", {k: round(v, 1) for k, v in sorted(rates.items())})
        ok &= hz >= 25.0
        # The kickoff bound (50 ms) is on the stamp itself, measured at the reader; the
        # end-to-end lag above adds the 33 Hz publish cadence and the zenoh hop.
        reader_lag = stats["timebase"]["stamp_lag_ms_p50"]
        ok &= reader_lag is not None and abs(reader_lag) <= 50.0

        if args.fly:
            print("sitl_enable:", drone.sitl_enable(True))
            print("takeoff:", drone.takeoff())
            state = _wait_state(drone, {"HOVER", "IDLE", "ABORT"}, timeout_s=60.0)
            st = drone.status()
            print(f"after takeoff: state={state} reason={st['reason']!r} alt_m={st.get('alt_m')}")
            ok &= state == "HOVER"
            time.sleep(5.0)
            print("land:", drone.land())
            state = _wait_state(drone, {"IDLE", "ABORT"}, timeout_s=60.0)
            st = drone.status()
            print(f"after land: state={state} reason={st['reason']!r}")
            print(f"tick jitter ms (flight): {drone.sensor_stats()['tick_jitter_ms']}")
            ok &= state == "IDLE"
    finally:
        coordinator.stop()
        gcs_stop.set()
        gcs.join(timeout=2.0)
    print("GATE", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
