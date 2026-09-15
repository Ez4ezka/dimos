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

"""The aircraft in the field: the bench stack plus the link monitor.

Adds LinkMonitor to ``px4_bench``; RtspCamera obeys its ``link_policy``. Advisory only,
nothing here is in the flight path.

Usage:
    dimos run px4-field            # on the Jetson, over 5G, props on with a pilot present
    dimos run px4-sitl-field       # the same policy loop against SITL with a replayed link
"""

from __future__ import annotations

from dimos.core.coordination.blueprints import autoconnect
from dimos.msgs.link_msgs.LinkStatus import LinkStatus
from dimos.robot.px4.blueprints.basic.px4_basic import zenoh_transport
from dimos.robot.px4.blueprints.bench.px4_bench import px4_bench
from dimos.robot.px4.blueprints.bench.px4_sitl_bench import px4_sitl_bench
from dimos.robot.px4.link_monitor import LinkMonitor

px4_field = autoconnect(
    px4_bench,
    LinkMonitor.blueprint(),
).transports({("link_status", LinkStatus): zenoh_transport("/link_status", LinkStatus)})

px4_sitl_field = autoconnect(
    px4_sitl_bench,
    LinkMonitor.blueprint(source="replay", replay_scenario="downtown_idle"),
).transports({("link_status", LinkStatus): zenoh_transport("/link_status", LinkStatus)})
