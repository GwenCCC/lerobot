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
"""The plugin is found and dispatched the way LeRobot's CLI tools find it.

These run against whichever LeRobot is installed, so CI can catch a LeRobot
release that changes the plugin contract before users do.
"""

import importlib.metadata

import pytest

from lerobot.robots.config import RobotConfig
from lerobot.robots.utils import make_robot_from_config
from lerobot.utils.import_utils import register_third_party_plugins

ROBOT_TYPES = {
    "limx_tron1": ("LimxTron1Config", "LimxTron1", {}),
    "limx_tron2": ("LimxTron2Config", "LimxTron2", {"robot_ip": "10.0.0.1"}),
    "limx_oli": ("LimxOliConfig", "LimxOli", {}),
    "limx_luna": ("LimxLunaConfig", "LimxLuna", {"kp": [1.0] * 27, "kd": [0.1] * 27}),
}


def test_distribution_name_is_discoverable():
    # register_third_party_plugins() imports each matching distribution by its
    # metadata name, so that name must start with the prefix and be importable.
    name = importlib.metadata.distribution("lerobot_robot_limx").metadata["Name"]
    assert name.replace("-", "_").startswith("lerobot_robot_")
    assert name.replace("-", "_") == "lerobot_robot_limx"


@pytest.mark.parametrize("robot_type", sorted(ROBOT_TYPES))
def test_cli_discovery_and_dispatch(robot_type):
    register_third_party_plugins()
    config_name, robot_name, kwargs = ROBOT_TYPES[robot_type]

    config_cls = RobotConfig.get_choice_class(robot_type)
    assert config_cls.__name__ == config_name
    assert config_cls.__module__.startswith("lerobot_robot_limx.")

    robot = make_robot_from_config(config_cls(**kwargs))
    assert type(robot).__name__ == robot_name
    assert not robot.is_connected
