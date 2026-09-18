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

from dimos.msgs.sensor_msgs.BatteryState import (
    POWER_SUPPLY_STATUS_DISCHARGING,
    POWER_SUPPLY_TECHNOLOGY_LIPO,
    BatteryState,
)


def test_lcm_roundtrip() -> None:
    batt = BatteryState(
        voltage=15.8,
        current=-12.5,
        percentage=0.73,
        power_supply_status=POWER_SUPPLY_STATUS_DISCHARGING,
        power_supply_technology=POWER_SUPPLY_TECHNOLOGY_LIPO,
        present=True,
        cell_voltage=[3.95, 3.95, 3.9, 4.0],
        frame_id="battery",
        ts=1700000000.5,
    )
    back = BatteryState.lcm_decode(batt.lcm_encode())
    assert (back.voltage, back.current) == pytest.approx((15.8, -12.5))
    assert back.percentage == pytest.approx(0.73)
    assert back.power_supply_status == POWER_SUPPLY_STATUS_DISCHARGING
    assert back.present is True
    assert back.cell_voltage == pytest.approx([3.95, 3.95, 3.9, 4.0])
    assert math.isnan(back.temperature)
    assert back.frame_id == "battery"
    assert back.ts == pytest.approx(1700000000.5, abs=1e-6)
