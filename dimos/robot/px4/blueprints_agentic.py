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

"""The agentic PX4 blueprints: teleop plus an LLM agent that flies by the skills.

``px4-agentic``, ``px4-sitl-agentic``
    ``px4-teleop`` (or its simulator twin) with Px4SkillContainer, the MCP server and the
    agent: ``dimos agent-send "take off to 2 meters"``. Same composition as
    ``unitree_go2_agentic``.

Apart from ``blueprints.py`` because the agent needs the ``agents`` extra (langchain),
which the aircraft does not install for plain flight.
"""

from __future__ import annotations

from dimos.agents.mcp.mcp_client import McpClient
from dimos.agents.mcp.mcp_server import McpServer
from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.px4.blueprints import px4_sitl_teleop, px4_teleop
from dimos.robot.px4.skill_container import PX4_SYSTEM_PROMPT, Px4SkillContainer

_px4_agent = autoconnect(
    Px4SkillContainer.blueprint(),
    McpServer.blueprint(),
    McpClient.blueprint(system_prompt=PX4_SYSTEM_PROMPT),
)

px4_agentic = autoconnect(px4_teleop, _px4_agent)
px4_sitl_agentic = autoconnect(px4_sitl_teleop, _px4_agent)
