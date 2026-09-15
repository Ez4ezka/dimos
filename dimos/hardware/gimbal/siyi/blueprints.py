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

"""Standalone SIYI A8 gimbal viewer: the frame chain and camera frustum in rerun.

``dimos run siyi-a8-gimbal-vis``   # beside a running px4-drone-connection (gimbal_attitude)

The module publishes only its own subtree; without the connection there is no
``gimbal_attitude`` and nothing is drawn. Pair it with ``fake-a8`` for a desk demo.
"""

from __future__ import annotations

from typing import Any

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.hardware.gimbal.siyi.gimbal import SiyiA8Gimbal
from dimos.visualization.vis_module import vis_module


def _rerun_blueprint() -> Any:
    import rerun.blueprint as rrb

    return rrb.Blueprint(rrb.Spatial3DView(origin="world", name="Gimbal chain"))


siyi_a8_gimbal_vis = autoconnect(
    vis_module(global_config.viewer, rerun_config={"blueprint": _rerun_blueprint}),
    SiyiA8Gimbal.blueprint(),
)
