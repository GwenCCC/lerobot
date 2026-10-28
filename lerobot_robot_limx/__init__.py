# Copyright 2026 LimX Dynamics. All rights reserved.
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
"""LeRobot plugin for LimX Dynamics robots.

LeRobot imports this package automatically (any installed distribution named
``lerobot_robot_*``), which registers the robot types below:

- ``limx_tron1``    — TRON1 legged robot (point / sole / wheel foot)
- ``limx_tron2``    — TRON2 dual-arm
- ``limx_oli``      — Oli full-size humanoid (31 joints)
- ``limx_luna``     — Luna humanoid (27 joints)

Vendor SDKs are imported lazily at ``connect()``, so importing this package never
requires them.
"""

from .limx_luna import LimxLuna, LimxLunaConfig
from .limx_oli import LimxOli, LimxOliConfig
from .limx_tron1 import LimxTron1, LimxTron1Config
from .limx_tron2 import LimxTron2, LimxTron2Config

__version__ = "0.1.0"

__all__ = [
    "LimxLuna",
    "LimxLunaConfig",
    "LimxOli",
    "LimxOliConfig",
    "LimxTron1",
    "LimxTron1Config",
    "LimxTron2",
    "LimxTron2Config",
]
