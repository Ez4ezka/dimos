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

import math

import pytest

from dimos.msgs.px4_msgs.CommandEvent import CommandEvent
from dimos.msgs.px4_msgs.TrackedCommand import TrackedCommand
from dimos.msgs.px4_msgs.VehicleStatus import VehicleStatus


def test_vehicle_status_roundtrip() -> None:
    msg = VehicleStatus(
        armed=True,
        main_mode=6,
        sub_mode=0,
        mode="OFFBOARD",
        landed_state=2,
        battery_pct=73,
        voltage=15.8,
        gps_fix=6,
        gps_sats=27,
        gps_eph=0.6,
        rc_age_s=0.05,
        heartbeat_age_s=0.4,
        home_valid=True,
        home_lat=37.7749,
        home_lon=-122.4194,
        home_alt=12.5,
        timebase_quality="system_time",
        timebase_offset_s=1_800_000_000.25,
        writer="1/195",
        state="HOVER",
        estop_latched=False,
        tick_jitter_p99_ms=1.3,
        ts=1700000000.5,
    )
    back = VehicleStatus.lcm_decode(msg.lcm_encode())
    assert (back.armed, back.main_mode, back.mode, back.state) == (True, 6, "OFFBOARD", "HOVER")
    assert (back.landed_state, back.battery_pct, back.gps_fix, back.gps_sats) == (2, 73, 6, 27)
    assert back.voltage == pytest.approx(15.8, abs=1e-5)
    assert back.gps_eph == pytest.approx(0.6, abs=1e-6)
    assert (back.home_lat, back.home_lon) == (37.7749, -122.4194)
    assert back.timebase_offset_s == 1_800_000_000.25
    assert back.writer == "1/195" and back.timebase_quality == "system_time"
    assert back.tick_jitter_p99_ms == pytest.approx(1.3, abs=1e-6)
    assert back.ts == pytest.approx(1700000000.5, abs=1e-6)


def test_vehicle_status_unknowns_survive() -> None:
    back = VehicleStatus.lcm_decode(VehicleStatus(ts=1.0).lcm_encode())
    assert math.isnan(back.voltage) and math.isnan(back.rc_age_s)
    assert back.battery_pct == -1 and back.gps_fix == -1 and back.writer == ""


def test_command_event_roundtrip() -> None:
    ev = CommandEvent(
        request_id=7,
        command="set_guidance_mode",
        argument="TELEOP",
        source="rpc",
        verdict_ts=1700000000.75,
        accepted=False,
        rejection="estop_latched",
        state_before="IDLE",
        state_after="IDLE",
        ts=1700000000.5,
    )
    back = CommandEvent.lcm_decode(ev.lcm_encode())
    assert back.request_id == 7
    assert (back.command, back.argument, back.source) == ("set_guidance_mode", "TELEOP", "rpc")
    assert back.accepted is False and back.rejection == "estop_latched"
    assert (back.state_before, back.state_after) == ("IDLE", "IDLE")
    assert back.verdict_ts == 1700000000.75
    assert back.ts == pytest.approx(1700000000.5, abs=1e-6)


def test_fingerprints_differ_between_types() -> None:
    with pytest.raises(ValueError):
        CommandEvent.lcm_decode(VehicleStatus().lcm_encode())


def test_tracked_command_roundtrip() -> None:
    tc = TrackedCommand(
        event_id=3,
        command="cmd_vel",
        argument="1.00,0.00,0.00,0.00",
        source="cmd_vel",
        verdict="ok",
        request_ts=1700000000.0,
        verdict_ts=1700000000.01,
        commanded_ts=1700000000.05,
        moved_ts=1700000000.3,
        mux_ms=10.0,
        onboard_ms=40.0,
        response_ms=250.0,
        setpoint_count=40,
        setpoint_gap_max_ms=52.0,
        commanded_peak=1.0,
        measured_peak=0.9,
        clamp_ratio=1.0,
        duration_s=2.0,
        state_before="TELEOP",
        state_after="TELEOP",
        ts=1700000002.0,
    )
    back = TrackedCommand.lcm_decode(tc.lcm_encode())
    assert (back.event_id, back.command, back.verdict, back.rejection) == (3, "cmd_vel", "ok", "")
    assert (back.mux_ms, back.onboard_ms, back.response_ms) == pytest.approx((10.0, 40.0, 250.0))
    assert math.isnan(back.link_ms)
    assert back.setpoint_count == 40 and back.scoring_version == 1
    assert back.moved_ts == 1700000000.3
    assert "mux=10ms onboard=40ms response=250ms" in back.report_line()
