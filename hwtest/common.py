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
"""Shared helpers for the hardware test scripts: robot construction, safety limits and logging.

Every script in this folder:

- builds the robot from the same command-line options (``--robot``, ``--ip``, gains, ...);
- refuses to command a target further than ``MAX_OFFSET`` from the pose it started at, or to move
  any joint more than ``MAX_STEP`` between two consecutive commands;
- asks for confirmation before the first command (skip with ``--yes``);
- disconnects on exit, including Ctrl+C;
- writes a timestamped log under ``hwtest_logs/`` and ends with a ``RESULT: PASS`` / ``RESULT: FAIL``
  line, whose path goes into the "实测记录 / 备注" column of the checklist.

``--sim`` replaces the vendor SDK with an in-process stand-in (see ``sim.py``), so every script can
be rehearsed without a robot.
"""

from __future__ import annotations

import argparse
import json
import logging
import math
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Any

#: Largest distance (rad) any commanded joint may be from the pose recorded at start.
MAX_OFFSET = 0.15
#: Largest change (rad) of any joint between two consecutive commands.
MAX_STEP = 0.02

ROBOTS = {
    # name: (robot type string, class name, config class name, default ip, default ROBOT_TYPE)
    "tron1": ("limx_tron1", "LimxTron1", "LimxTron1Config", "10.192.1.2", "PF_TRON1A"),
    "tron2": ("limx_tron2", "LimxTron2", "LimxTron2Config", "192.168.1.150", None),
    "oli": ("limx_oli", "LimxOli", "LimxOliConfig", "10.192.1.2", "HU_D04_01"),
    "luna": ("limx_luna", "LimxLuna", "LimxLunaConfig", "10.192.1.2", "HU_L04_01"),
}

#: Robots whose driver republishes from its own background thread (needed by order / stall tests).
PUBLISH_LOOP_ROBOTS = ("tron1", "oli", "luna")


class SafetyError(RuntimeError):
    """A command was refused by the script's own safety limits; nothing was sent."""


# ---------------------------------------------------------------- arguments


def base_parser(description: str) -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument("--robot", required=True, choices=sorted(ROBOTS), help="robot to test")
    parser.add_argument("--ip", help="robot address (defaults to the driver default)")
    parser.add_argument(
        "--robot-type",
        help="ROBOT_TYPE build string, e.g. PF_TRON1A / SF_TRON1A / WF_TRON1A / HU_D04_01 / HU_L04_01 "
        "(defaults: tron1 PF_TRON1A, oli HU_D04_01, luna HU_L04_01; ignored for tron2)",
    )
    parser.add_argument(
        "--gains-file",
        help='JSON file {"kp": [...], "kd": [...]} in joint_names order. Required for luna; '
        "optional for tron1 / oli (driver defaults are used otherwise); ignored for tron2",
    )
    parser.add_argument(
        "--gain-scale",
        type=float,
        default=0.25,
        help="multiply kp and kd by this factor (default 0.25 = low gains for first runs; use 1.0 for full gains)",
    )
    parser.add_argument("--state-max-age", type=float, default=0.5, help="reject state older than this (s)")
    parser.add_argument("--log-dir", default="hwtest_logs", help="where to write the log file")
    parser.add_argument("--yes", action="store_true", help="do not ask for confirmation before moving")
    parser.add_argument("--sim", action="store_true", help="use the built-in SDK stand-in instead of a robot")
    return parser


# ------------------------------------------------------------------ logging


class TestLog:
    """Human-readable log on stdout and in a file, plus a JSONL event stream next to it."""

    def __init__(self, script: str, args: argparse.Namespace):
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_dir = Path(args.log_dir)
        log_dir.mkdir(parents=True, exist_ok=True)
        base = f"{script}_{args.robot}{'_sim' if args.sim else ''}_{stamp}"
        self.path = log_dir / f"{base}.log"
        self.events_path = log_dir / f"{base}.jsonl"
        self._events = self.events_path.open("w", encoding="utf-8")
        self.failures: list[str] = []

        self.logger = logging.getLogger(f"hwtest.{script}")
        self.logger.setLevel(logging.INFO)
        self.logger.handlers.clear()
        self.logger.propagate = False
        fmt = logging.Formatter("%(asctime)s %(levelname)s %(message)s")
        for handler in (logging.StreamHandler(sys.stdout), logging.FileHandler(self.path, encoding="utf-8")):
            handler.setFormatter(fmt)
            self.logger.addHandler(handler)
        # Driver warnings (e.g. "no usable motor names") go to the same file.
        driver_log = logging.getLogger("lerobot_robot_limx")
        driver_log.setLevel(logging.INFO)
        driver_log.addHandler(self.logger.handlers[1])

        self.info("script=%s args=%s", script, vars(args))

    def info(self, msg: str, *a: Any) -> None:
        self.logger.info(msg, *a)

    def warn(self, msg: str, *a: Any) -> None:
        self.logger.warning(msg, *a)

    def event(self, kind: str, **fields: Any) -> None:
        record = {"t": time.monotonic(), "wall": time.time(), "kind": kind, **fields}
        self._events.write(json.dumps(record, ensure_ascii=False, default=_json_default) + "\n")
        self._events.flush()

    def fail(self, msg: str, *a: Any) -> None:
        text = msg % a if a else msg
        self.failures.append(text)
        self.logger.error("CHECK FAILED: %s", text)

    def finish(self) -> int:
        self._events.close()
        if self.failures:
            self.logger.error("RESULT: FAIL (%d check(s) failed) - log: %s", len(self.failures), self.path)
            return 1
        self.logger.info("RESULT: PASS - log: %s", self.path)
        return 0


def _json_default(value: Any) -> Any:
    if hasattr(value, "tolist"):
        return value.tolist()
    return str(value)


# -------------------------------------------------------------------- robot


def make_robot(args: argparse.Namespace, log: TestLog):
    """Build (but do not connect) the robot described by the command line."""
    import lerobot_robot_limx as plugin

    if args.sim:
        from . import sim

        sim.install(args.robot)

    robot_type_name, cls_name, cfg_name, default_ip, default_build = ROBOTS[args.robot]
    cfg_cls = getattr(plugin, cfg_name)
    robot_cls = getattr(plugin, cls_name)

    kwargs: dict[str, Any] = {"robot_ip": args.ip or default_ip, "state_max_age": args.state_max_age}
    if args.robot != "tron2":
        kwargs["robot_type"] = args.robot_type or default_build

    if args.robot != "tron2":
        kp, kd = _gains(args, cfg_cls, kwargs)
        kwargs["kp"] = [k * args.gain_scale for k in kp]
        kwargs["kd"] = [d * args.gain_scale for d in kd]

    config = cfg_cls(**kwargs)
    robot = sim.wrap(args.robot, robot_cls, config) if args.sim else robot_cls(config)

    log.info("robot=%s ip=%s build=%s", robot_type_name, config.robot_ip, kwargs.get("robot_type"))
    if "kp" in kwargs:
        log.info("gain_scale=%s kp=%s kd=%s", args.gain_scale, config.kp, config.kd)
    return robot


def _gains(args: argparse.Namespace, cfg_cls, kwargs: dict[str, Any]) -> tuple[list[float], list[float]]:
    if args.gains_file:
        data = json.loads(Path(args.gains_file).read_text(encoding="utf-8"))
        return list(map(float, data["kp"])), list(map(float, data["kd"]))
    if args.robot == "luna":
        raise SystemExit("luna has no default gains: pass --gains-file with 27 kp and 27 kd values")
    defaults = cfg_cls(**kwargs)  # driver defaults for this build
    return list(defaults.kp), list(defaults.kd)


def joint_keys(robot) -> list[str]:
    return list(robot.action_features)


def confirm(args: argparse.Namespace, log: TestLog, what: str) -> None:
    log.info("ABOUT TO MOVE: %s", what)
    if args.yes or args.sim:
        return
    answer = (
        input(f"\n即将运动：{what}\n确认机器人已吊挂/周围无人、急停在手边，输入 yes 继续：").strip().lower()
    )
    if answer != "yes":
        log.info("aborted by operator")
        raise SystemExit(2)


class SafeCommander:
    """Wraps ``send_action`` with the script's offset and step limits."""

    def __init__(self, robot, log: TestLog, base: dict[str, float]):
        self.robot = robot
        self.log = log
        self.base = dict(base)
        self.last = dict(base)

    def send(self, target: dict[str, float]) -> dict[str, float]:
        for key, value in target.items():
            if key not in self.base:
                continue
            if not math.isfinite(value):
                raise SafetyError(f"{key}: non-finite target {value}")
            if abs(value - self.base[key]) > MAX_OFFSET + 1e-9:
                raise SafetyError(f"{key}: target {value:.4f} is more than {MAX_OFFSET} rad from start pose")
            if abs(value - self.last[key]) > MAX_STEP + 1e-9:
                raise SafetyError(f"{key}: step {value - self.last[key]:+.4f} exceeds {MAX_STEP} rad")
        sent = self.robot.send_action(target)
        self.last.update({k: v for k, v in target.items() if k in self.base})
        return sent

    def ramp_to(self, target: dict[str, float], rate_hz: float) -> None:
        """Move from the last command to ``target`` in steps no larger than ``MAX_STEP / 2``."""
        diffs = [abs(target[k] - self.last[k]) for k in self.base]
        steps = max(1, math.ceil(max(diffs) / (MAX_STEP / 2)))
        start = dict(self.last)
        for i in range(1, steps + 1):
            a = i / steps
            self.send({k: start[k] + a * (target[k] - start[k]) for k in self.base})
            time.sleep(1.0 / rate_hz)


def read_pose(robot) -> dict[str, float]:
    obs = robot.get_observation()
    return {k: float(obs[k]) for k in joint_keys(robot)}


def run(main, script: str, args: argparse.Namespace) -> None:
    """Run ``main(robot, log, args)`` with logging, safety errors and a guaranteed disconnect."""
    log = TestLog(script, args)
    robot = None
    try:
        robot = make_robot(args, log)
        main(robot, log, args)
    except SafetyError as exc:
        log.fail("safety limit refused a command: %s", exc)
    except KeyboardInterrupt:
        log.fail("interrupted by operator (Ctrl+C)")
    except SystemExit:
        raise
    except Exception as exc:  # noqa: BLE001 - every error must end up in the log
        log.logger.exception("unexpected error")
        log.fail("%s: %s", type(exc).__name__, exc)
    finally:
        if robot is not None and robot.is_connected:
            try:
                robot.disconnect()
                log.info("disconnected")
            except Exception:  # noqa: BLE001
                log.logger.exception("error while disconnecting")
    sys.exit(log.finish())


# ------------------------------------------------------- send instrumentation


class SendRecorder:
    """Wrap the driver's ``_send`` to log every SDK publish, and optionally block one of them.

    ``_send(handle, datatypes, target)`` is what both ``send_action`` and the background publish loop
    call, while holding the driver's publish lock, right before ``publishRobotCmd``.  Wrapping it
    records which thread published, to which SDK handle, which target, and when the call was entered
    and returned.  Blocking inside the wrapper holds the publish lock exactly as a stalled
    ``publishRobotCmd`` would.
    """

    def __init__(self, robot, log: TestLog):
        import threading

        self.robot = robot
        self.log = log
        self.records: list[dict] = []
        self._lock = threading.Lock()
        self._original = robot._send
        self._block_thread: int | None = None
        self.blocked_entered = threading.Event()
        self.release = threading.Event()
        self._handle_gen: dict[int, int] = {}
        robot._send = self._wrapped

    def generation(self, handle) -> int:
        with self._lock:
            return self._handle_gen.setdefault(id(handle), len(self._handle_gen) + 1)

    def block_next_publish_from(self, thread_ident: int) -> None:
        self.blocked_entered.clear()
        self.release.clear()
        self._block_thread = thread_ident

    def _wrapped(self, handle, datatypes, target):
        import threading

        thread = threading.current_thread()
        rec = {
            "enter": time.monotonic(),
            "thread": thread.name,
            "thread_id": thread.ident,
            "handle_gen": self.generation(handle),
            "target": [round(float(x), 6) for x in target],
            "blocked": False,
        }
        with self._lock:
            self.records.append(rec)
        if self._block_thread is not None and thread.ident == self._block_thread:
            self._block_thread = None
            rec["blocked"] = True
            self.log.info("publish from %s (connection %d) is now BLOCKED", thread.name, rec["handle_gen"])
            self.blocked_entered.set()
            if not self.release.wait(timeout=30.0):
                self.log.warn("blocked publish released by 30 s safety timeout")
            rec["released"] = time.monotonic()
        try:
            self._original(handle, datatypes, target)
        finally:
            rec["return"] = time.monotonic()
            self.log.event("publish", **rec)

    def snapshot(self) -> list[dict]:
        with self._lock:
            return [dict(r) for r in self.records]
