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

"""Px4SkillContainer: the flight commands an agent can call, in words an operator would use.

Every skill is one ``Px4DroneConnection`` RPC plus a wait for the outcome, so the agent
hears where the drone ended up, not just that the command was accepted. The skills decide
nothing: the fence, the ceiling, the RC enable switch and the E-STOP latch all live in
``supervisor_core.py`` and refuse the same way whoever asks.
"""

from __future__ import annotations

from collections.abc import Callable
import math
import time
from typing import Any

from pydantic import Field

from dimos.agents.annotation import skill
from dimos.core.module import Module, ModuleConfig
from dimos.robot.px4.connection_spec import Px4DroneConnectionSpec
from dimos.robot.px4.supervisor_core import GOTO_ARRIVED, GUIDANCE_STATES, TAKEOFF_STATES

PX4_SYSTEM_PROMPT = """\
You fly a PX4 quadcopter for an operator who stands next to it with the RC transmitter.
Use the skills to do what the operator says, one flight command at a time, and report
each result in a sentence.

- Distances are metres. Directions are compass directions: south is a negative north_m,
  west is a negative east_m. Altitudes are above the takeoff point.
- "Take off to 2 meters" is takeoff(altitude_m=2). "Go 2 meters south at 3 m altitude"
  is go_to(north_m=-2, altitude_m=3). "Come back" is go_to(relative=False).
- A refused command names the reason (fence, ceiling, enable_switch_off, estop_latched,
  not_armed, wrong_state, invalid_argument). Tell the operator; never retry with
  different numbers on your own.
- The RC pilot can take over at any moment. If a skill reports that, stop and say so.
"""


# The supervisor calls a takeoff or a go-to done inside hover_tolerance_m and lets the
# hover setpoint close the rest; a skill reports once the vehicle sits this close to it.
_SETTLED_M = 0.15


class Px4SkillContainerConfig(ModuleConfig):
    # How long a skill waits for a command to end before it reports back. Every command
    # ends by itself well inside this (a go-to gives up after GotoConfig.timeout_s), and
    # it stays under the agent's 120 s tool-call timeout.
    wait_timeout_s: float = Field(default=90.0)
    # How long it then gives the hover to settle on its setpoint.
    settle_timeout_s: float = Field(default=8.0)
    poll_hz: float = Field(default=4.0)


def _where(status: dict[str, Any]) -> str:
    """The vehicle's place relative to the takeoff point, from a supervisor status."""
    if "alt_m" not in status:
        return f"State {status['state']}."
    return (
        f"State {status['state']}: {status['alt_m']:.1f} m up, {status['north_m']:+.1f} m north "
        f"and {status['east_m']:+.1f} m east of the takeoff point."
    )


def _refused(result: dict[str, Any]) -> str:
    return f"Refused: {result['rejection']} (state {result['state']})."


def _settled(status: dict[str, Any]) -> bool:
    """Whether the vehicle sits on the position setpoint it is holding."""
    sp, here = status.get("setpoint"), status.get("local")
    if not sp or not here or sp["kind"] != "pos":
        return True
    return math.dist((sp["n"], sp["e"], sp["d"]), (here["n"], here["e"], here["d"])) <= _SETTLED_M


class Px4SkillContainer(Module):
    """Agent-facing flight skills over ``Px4DroneConnection``."""

    config: Px4SkillContainerConfig

    _connection: Px4DroneConnectionSpec

    def _poll(self, done: Callable[[dict[str, Any]], bool], timeout_s: float) -> dict[str, Any]:
        """The supervisor status once ``done`` accepts it, or the last one when time is up."""
        deadline = time.monotonic() + timeout_s
        status = self._connection.status()
        while not done(status) and time.monotonic() < deadline:
            time.sleep(1.0 / self.config.poll_hz)
            status = self._connection.status()
        return status

    def _wait_while(self, busy: frozenset[str]) -> dict[str, Any]:
        return self._poll(lambda status: status["state"] not in busy, self.config.wait_timeout_s)

    def _where_settled(self) -> str:
        """Where the vehicle ends up once the hover has closed on its setpoint."""
        return _where(self._poll(_settled, self.config.settle_timeout_s))

    @skill
    def takeoff(self, altitude_m: float | None = None) -> str:
        """Take off and hover. Waits until the drone hovers or the takeoff fails.

        Args:
            altitude_m: Hover altitude above the ground in metres. Omit for the default.
        """
        result = self._connection.takeoff(altitude_m)
        if not result["accepted"]:
            return _refused(result)
        status = self._wait_while(TAKEOFF_STATES)
        if status["state"] in GUIDANCE_STATES:
            return f"Took off. {self._where_settled()}"
        return f"Takeoff failed: {status['reason']}. {_where(status)}"

    @skill
    def go_to(
        self,
        north_m: float = 0.0,
        east_m: float = 0.0,
        altitude_m: float | None = None,
        heading_deg: float | None = None,
        relative: bool = True,
    ) -> str:
        """Fly to a point at walking pace and hover there. Waits until the drone arrives.

        north_m and east_m are metres along the compass axes: south is a negative
        north_m, west a negative east_m. With relative=True they count from where the
        drone is now, with relative=False from the takeoff point. altitude_m is metres
        above the takeoff point and heading_deg a compass heading (0 north, 90 east);
        leave either out to keep the current one.

        Example calls:

            go_to(north_m=-2, altitude_m=3)   # 2 m south, at 3 m altitude
            go_to(altitude_m=5)               # climb to 5 m on the spot
            go_to(heading_deg=90)             # turn to face east
            go_to(relative=False)             # back over the takeoff point
        """
        result = self._connection.go_to(north_m, east_m, altitude_m, heading_deg, relative)
        if not result["accepted"]:
            return _refused(result)
        status = self._wait_while(frozenset({"GOTO"}))
        if status["reason"] == GOTO_ARRIVED:
            return f"Arrived. {self._where_settled()}"
        return f"Did not arrive: {status['reason']}. {_where(status)}"

    @skill
    def land(self) -> str:
        """Land where the drone is and disarm. Waits until it is on the ground."""
        result = self._connection.land()
        if not result["accepted"]:
            return _refused(result)
        status = self._wait_while(frozenset({"LANDING"}))
        return f"Landing ended: {status['reason']}. {_where(status)}"

    @skill
    def set_guidance_mode(self, mode: str) -> str:
        """Select what the drone does while it flies.

        Args:
            mode: HOVER holds position (also stops a go-to), TELEOP hands the drone to
                the operator's keyboard, YAW_TRACK and FOLLOW track the selected target.
        """
        result = self._connection.set_guidance_mode(mode)
        if not result["accepted"]:
            return _refused(result)
        return f"Guidance mode {mode.upper()}, state {result['state']}."

    @skill
    def flight_status(self) -> str:
        """Where the drone is and what the flight supervisor is doing."""
        status = self._connection.status()
        return (
            f"{_where(status)} Reason: {status['reason']}. Armed: {status['armed']}, "
            f"PX4 mode {status['px4_mode']}, battery {status['batt_pct']}%."
        )
