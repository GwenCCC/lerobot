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
"""Command order: three targets sent back to back must reach the SDK in order, with no stale repeat.

Checklist item "指令顺序" (TRON1 / Oli / Luna; TRON2 has no publish loop in the driver).  Each round
sends targets A, B, C from the main thread with random gaps of 0-3 ms (about one loop period), while
the driver's background loop keeps republishing.  Every SDK publish is logged with its thread and
target.  A round passes when the main thread's publishes arrive as A, B, C and every loop publish
carries the newest target the main thread had already published - never an older one (e.g. A after
B, the [1, 2, 1] pattern reported on the PR).  A, B and C differ only in one joint, by 0.01 / 0.02 /
0.03 rad.  The race is timing-dependent, so many rounds are run.

    python -m hwtest.order_check --robot tron1 --robot-type PF_TRON1A --rounds 100
"""

from __future__ import annotations

import random
import time

from .common import (
    PUBLISH_LOOP_ROBOTS,
    SafeCommander,
    SendRecorder,
    base_parser,
    confirm,
    joint_keys,
    read_pose,
    run,
)

DELTAS = (0.01, 0.02, 0.03)


def main(robot, log, args):
    if args.robot not in PUBLISH_LOOP_ROBOTS:
        raise SystemExit(
            "order_check applies to tron1 / oli / luna only (tron2 has no publish loop in the driver)"
        )
    robot.connect()
    log.info("connected")
    keys = joint_keys(robot)
    joint = args.joint or keys[0]
    if joint not in keys:
        raise SystemExit(f"--joint must be one of {keys}")
    col = keys.index(joint)
    base = read_pose(robot)
    commander = SafeCommander(robot, log, base)
    commander.send(base)
    time.sleep(0.5)

    recorder = SendRecorder(robot, log)
    confirm(args, log, f"{joint} 在当前位置上快速依次 +0.01/+0.02/+0.03 rad 再回位，共 {args.rounds} 轮")
    import threading

    main_thread = threading.current_thread().ident
    for rnd in range(args.rounds):
        values = [base[joint] + d for d in DELTAS]
        mark = len(recorder.snapshot())
        for v in values:
            time.sleep(random.uniform(0.0, 0.003))
            commander.send({**commander.last, joint: v})
        time.sleep(args.settle)
        records = recorder.snapshot()[mark:]
        check_round(log, rnd, records, values, col, main_thread)
        commander.ramp_to(base, 50.0)
        time.sleep(0.2)

    total = len(recorder.snapshot())
    loop_publishes = sum(1 for r in recorder.snapshot() if r["thread_id"] != main_thread)
    log.info("%d publishes logged (%d from the background loop)", total, loop_publishes)
    if loop_publishes == 0:
        log.fail("the background loop never published; the check did not exercise it")


def check_round(log, rnd, records, values, col, main_thread):
    def which(rec):
        v = rec["target"][col]
        for name, ref in zip("ABC", values, strict=True):
            if abs(v - ref) < 1e-6:
                return name
        return "?"

    mains = [r for r in records if r["thread_id"] == main_thread]
    main_seq = [which(r) for r in mains]
    if main_seq != ["A", "B", "C"]:
        log.fail("round %d: main-thread publishes were %s, expected A, B, C", rnd, main_seq)
        return
    stale = []
    loop_count = 0
    for r in records:
        if r["thread_id"] == main_thread:
            continue
        done_before = [m for m in mains if m["return"] < r["enter"]]
        if not done_before:
            continue  # published before A was sent
        loop_count += 1
        expected = which(done_before[-1])
        got = which(r)
        if got != expected:
            stale.append(f"{got} after {expected}")
    if stale:
        log.fail("round %d: loop published an older target: %s", rnd, stale[:5])
    elif rnd % 10 == 0 or rnd == 0:
        log.info("round %d: OK (%d loop publishes checked)", rnd, loop_count)
    return not stale


if __name__ == "__main__":
    p = base_parser(__doc__.splitlines()[0])
    p.add_argument("--joint", help="joint key to move, e.g. knee_L_Joint.pos (default: first joint)")
    p.add_argument("--rounds", type=int, default=100)
    p.add_argument(
        "--settle", type=float, default=0.1, help="seconds to watch the loop after C (default 0.1)"
    )
    run(main, "order_check", p.parse_args())
