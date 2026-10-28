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
"""Continuous drive: move every joint through a small sine around its start pose for several minutes.

Checklist item "连续驱动 5 分钟".  The motion is a sine of ``--amplitude`` rad (default 0.03) around
the pose the robot is in when the script starts, faded in and out over 3 s, so the robot starts and
ends where it was.  Each cycle reads one observation and sends one action, like a LeRobot loop.

PASS when the whole duration runs without an exception.  The log also reports the loop rate, the
tracking error (target minus measured) per joint and the process CPU usage.

    python -m hwtest.drive --robot tron1 --robot-type PF_TRON1A --duration 300
"""

from __future__ import annotations

import math
import time

import numpy as np

from .common import MAX_OFFSET, SafeCommander, base_parser, confirm, joint_keys, read_pose, run

FADE_S = 3.0


def trajectory(
    keys: list[str],
    base: dict[str, float],
    t: float,
    duration: float,
    amplitude: float,
    period: float,
    selected: list[str],
) -> dict[str, float]:
    """Target pose at time ``t``: each selected joint gets a phase-shifted sine, faded in and out."""
    fade = min(1.0, t / FADE_S, max(0.0, duration - t) / FADE_S)
    target = dict(base)
    for i, key in enumerate(keys):
        if key in selected:
            phase = 2 * math.pi * i / max(1, len(keys))
            target[key] = base[key] + fade * amplitude * math.sin(2 * math.pi * t / period + phase)
    return target


def select_joints(keys: list[str], pattern: str | None) -> list[str]:
    if not pattern:
        return list(keys)
    parts = [p.strip() for p in pattern.split(",") if p.strip()]
    chosen = [k for k in keys if any(p in k for p in parts)]
    if not chosen:
        raise SystemExit(f"--joints {pattern!r} matches none of: {keys}")
    return chosen


def default_rate(robot_name: str) -> float:
    return 30.0 if robot_name == "tron2" else 50.0


def drive(robot, log, args, on_frame=None) -> dict:
    """Run one trajectory; ``on_frame(obs, sent)`` is called every cycle (used by record.py)."""
    keys = joint_keys(robot)
    selected = select_joints(keys, args.joints)
    base = read_pose(robot)
    log.info("start pose: %s", {k: round(v, 4) for k, v in base.items()})
    log.info("moving joints: %s", selected)
    commander = SafeCommander(robot, log, base)
    commander.send(base)  # hold the current pose first

    rate = args.rate or default_rate(args.robot)
    period_s = 1.0 / rate
    errors = dict.fromkeys(keys, 0.0)
    overruns = 0
    cycles = 0
    t0 = time.monotonic()
    cpu0 = time.process_time()
    next_tick = t0
    last_report = t0
    while True:
        t = time.monotonic() - t0
        if t >= args.duration:
            break
        obs = robot.get_observation()
        for k in keys:
            errors[k] = max(errors[k], abs(commander.last[k] - float(obs[k])))
        target = trajectory(keys, base, t, args.duration, args.amplitude, args.period, selected)
        sent = commander.send(target)
        if on_frame is not None:
            on_frame(obs, sent)
        cycles += 1
        if time.monotonic() - last_report >= 10.0:
            last_report = time.monotonic()
            log.info("t=%.0fs cycles=%d max tracking error so far=%.4f rad", t, cycles, max(errors.values()))
        next_tick += period_s
        sleep = next_tick - time.monotonic()
        if sleep > 0:
            time.sleep(sleep)
        else:
            overruns += 1
            next_tick = time.monotonic()

    commander.ramp_to(base, rate)
    elapsed = time.monotonic() - t0
    cpu = (time.process_time() - cpu0) / elapsed * 100
    stats = {
        "cycles": cycles,
        "rate_hz": cycles / elapsed,
        "overruns": overruns,
        "cpu_percent_of_one_core": cpu,
        "max_tracking_error_rad": {k: round(v, 4) for k, v in errors.items()},
    }
    log.event("drive_stats", **stats)
    log.info(
        "loop: %d cycles, %.1f Hz (target %.0f), %d overruns, CPU %.0f%% of one core",
        cycles,
        stats["rate_hz"],
        rate,
        overruns,
        cpu,
    )
    worst = max(errors, key=errors.get)
    log.info("max tracking error: %.4f rad (%s); per joint in the .jsonl file", errors[worst], worst)
    return stats


def main(robot, log, args):
    if args.amplitude > MAX_OFFSET:
        raise SystemExit(f"--amplitude must be <= {MAX_OFFSET}")
    robot.connect()
    log.info("connected")
    confirm(args, log, f"所选关节在当前姿态附近做 ±{args.amplitude} rad 正弦运动，持续 {args.duration:.0f} s")
    stats = drive(robot, log, args)
    if stats["cycles"] == 0:
        log.fail("no cycle completed")
    if stats["rate_hz"] < 0.8 * (args.rate or default_rate(args.robot)):
        log.warn("loop rate below 80%% of target; check CPU / network")


def add_drive_args(parser):
    parser.add_argument("--duration", type=float, default=300.0, help="seconds (default 300)")
    parser.add_argument("--amplitude", type=float, default=0.03, help="sine amplitude in rad (default 0.03)")
    parser.add_argument("--period", type=float, default=4.0, help="sine period in s (default 4)")
    parser.add_argument("--rate", type=float, help="command rate in Hz (default 50; tron2 30)")
    parser.add_argument(
        "--joints", help="comma-separated substrings to select joints, e.g. knee,hip (default all)"
    )


if __name__ == "__main__":
    p = base_parser(__doc__.splitlines()[0])
    add_drive_args(p)
    a = p.parse_args()
    np.set_printoptions(precision=4)
    run(main, "drive", a)
