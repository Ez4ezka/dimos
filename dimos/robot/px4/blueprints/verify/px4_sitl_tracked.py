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

"""PX4 SITL plus the command tracker: every operator command scored with a verdict.

Adds CommandTracker to ``px4_sitl`` the way ``r1pro_nav`` adds modules to
``r1pro_coordinator``. The tracker binds to the connection's ``command_event``,
``offboard_setpoint``, ``odometry``, ``vehicle_status`` and ``supervisor_state`` by
exact name and type; only its own outputs need transports here.

Usage:
    dimos run px4-sitl-tracked    # then in `dimos shell`: command_tracker.recent()
"""

from __future__ import annotations

from dimos.core.coordination.blueprints import autoconnect
from dimos.msgs.px4_msgs.TrackedCommand import TrackedCommand
from dimos.msgs.std_msgs.Float32 import Float32
from dimos.msgs.std_msgs.String import String
from dimos.robot.px4.blueprints.basic.px4_basic import zenoh_transport
from dimos.robot.px4.blueprints.basic.px4_sitl import px4_sitl
from dimos.robot.px4.command_tracker import CommandTracker

px4_sitl_tracked = autoconnect(
    px4_sitl,
    CommandTracker.blueprint(),
).transports(
    {
        ("tracked_command", TrackedCommand): zenoh_transport("/tracked_command", TrackedCommand),
        ("command_report", String): zenoh_transport("/command_report", String),
        ("cmd_forward", Float32): zenoh_transport("/cmd_forward", Float32),
        ("meas_forward", Float32): zenoh_transport("/meas_forward", Float32),
    }
)
