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

"""PX4 SITL stacks: the base against the simulator, and the same plus a scripted target.

Derived from ``px4_basic`` the way ``r1pro_teleop`` derives from ``r1pro_coordinator``.
The SITL endpoint and the fake enable switch are connection config, so ``px4_control()``
is called afresh with them; ``px4_sitl_follow`` then adds FakeTarget by module.

Usage:
    dimos run px4-sitl              # `make px4_sitl gz_x500` running in the PX4 tree
    dimos run px4-sitl-follow       # + a scripted target for FOLLOW and YAW_TRACK
"""

from __future__ import annotations

from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.px4.blueprints.basic.px4_basic import px4_control, px4_visualization
from dimos.robot.px4.config import SITL_MAV_URL
from dimos.robot.px4.sitl.fake_target import FakeTarget

# Same worker split as px4_basic: viewer encode in one interpreter, the connection alone
# in its dedicated worker.
px4_sitl = autoconnect(
    px4_visualization(),
    px4_control(mav_url=SITL_MAV_URL, sitl=True),
).global_config(n_workers=2)

px4_sitl_follow = autoconnect(
    px4_sitl,
    FakeTarget.blueprint(),
)
