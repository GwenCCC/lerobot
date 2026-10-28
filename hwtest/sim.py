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
"""In-process stand-ins for the vendor SDKs, used by ``--sim``.

They are deliberately simple: the simulated joints follow the last command with a first-order lag,
and state is pushed from a background thread, like the real SDK callback.  They exist so the
scripts can be rehearsed and checked without a robot; they say nothing about real hardware.
"""

from __future__ import annotations

import sys
import threading
import time
import types

import numpy as np

# ------------------------------------------------------------- limxsdk fake


class _RobotCmd:
    def __init__(self):
        self.stamp = 0
        self.mode, self.q, self.dq, self.tau, self.Kp, self.Kd = [], [], [], [], [], []
        self.motor_names, self.parallel_solve_required = [], []


class _RobotState:
    def __init__(self, q):
        self.stamp = time.time_ns()
        self.q = list(q)
        self.dq = [0.0] * len(q)
        self.tau = [0.0] * len(q)
        self.motor_names = []


class _SimLimxRobot:
    """Minimal ``limxsdk.robot.Robot``: init, subscribeRobotState, publishRobotCmd, getMotorNames."""

    dim = 6

    def __init__(self, robot_type):
        self.robot_type = robot_type
        self._q = np.linspace(-0.2, 0.2, self.dim)
        self._target = None
        self._callback = None
        self._running = False

    def init(self, robot_ip: str) -> bool:
        self._running = True
        threading.Thread(target=self._state_loop, name="sim-state", daemon=True).start()
        return True

    def subscribeRobotState(self, callback):  # noqa: N802 - vendor method name
        self._callback = callback

    def publishRobotCmd(self, cmd):  # noqa: N802 - vendor method name
        self._target = np.asarray(cmd.q, dtype=float)

    def getMotorNames(self):  # noqa: N802 - vendor method name
        return [f"sim_motor_{i}" for i in range(self.dim)]

    def _state_loop(self):
        while self._running:
            if self._target is not None and len(self._target) == self.dim:
                self._q = self._q + 0.2 * (self._target - self._q)
            if self._callback is not None:
                self._callback(_RobotState(self._q))
            time.sleep(0.002)


def _install_limxsdk(dim: int) -> None:
    robot_cls = type("SimLimxRobot", (_SimLimxRobot,), {"dim": dim})
    top = types.ModuleType("limxsdk")
    top.datatypes = types.SimpleNamespace(RobotCmd=_RobotCmd)
    robot_mod = types.ModuleType("limxsdk.robot")
    robot_mod.Robot = robot_cls
    robot_mod.RobotType = types.SimpleNamespace(PointFoot="PointFoot", Humanoid="Humanoid")
    sys.modules["limxsdk"] = top
    sys.modules["limxsdk.robot"] = robot_mod


# -------------------------------------------------------------- tron2 fake


class _SimTron2Controller:
    """Minimal tron2-env motion controller: 18-value state, 16-value ServoJ commands."""

    def __init__(self):
        self._state = np.zeros(18)
        self._state[[7, 15]] = 0.5  # grippers
        self._target = None
        self._connected = False
        self._lock = threading.Lock()

    def start(self):
        self._connected = True
        threading.Thread(target=self._loop, name="sim-tron2", daemon=True).start()

    def is_connected(self):
        return self._connected

    def disconnect(self):
        self._connected = False

    def raise_if_faulted(self):
        pass

    def get_joint_states(self, timeout=None, max_age=None):
        with self._lock:
            return {"states": self._state.tolist()}

    def command_joints(self, servoj):
        self._target = np.asarray(servoj, dtype=float)

    def command_end_effector(self, left, right):
        with self._lock:
            self._state[7], self._state[15] = float(left[0]), float(right[0])

    def _loop(self):
        arm_head = [*range(0, 7), *range(8, 15), 16, 17]
        while self._connected:
            if self._target is not None:
                with self._lock:
                    self._state[arm_head] += 0.2 * (self._target - self._state[arm_head])
            time.sleep(0.003)


# ---------------------------------------------------------------- entrypoints


def install(robot: str) -> None:
    """Make the vendor import inside the driver resolve to a stand-in."""
    import importlib

    if robot == "tron2":
        return
    dim = {"tron1": 6, "oli": 31, "luna": 27}[robot]
    _install_limxsdk(dim)
    module = importlib.import_module(f"lerobot_robot_limx.limx_{robot}.limx_{robot}")
    module.require_sdk = lambda *a, **k: None


def wrap(robot: str, robot_cls, config):
    """Return a robot instance wired to the stand-in."""
    if robot == "tron1":
        # The joint count depends on the build, so size the stand-in from the config.
        _install_limxsdk(len(config.joint_names))
    if robot == "tron2":
        sim_cls = type("SimLimxTron2", (robot_cls,), {"_make_controller": lambda self: _SimTron2Controller()})
        return sim_cls(config)
    return robot_cls(config)
