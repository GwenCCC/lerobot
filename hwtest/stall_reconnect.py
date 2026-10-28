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
"""Reconnect after a stalled publish: the old publisher must not send again once it is released.

Checklist item "发送卡住后重连" - the case requested by the reviewer on huggingface/lerobot#4803:

1. connect (connection 1) and hold the current pose;
2. block the next publish of connection 1's background thread, inside the driver's send, so the
   publish lock is held just as a stalled ``publishRobotCmd`` would hold it;
3. ``disconnect()`` - its 2 s join times out while the publish is blocked;
4. ``connect()`` again (connection 2);
5. release the blocked publish, then hold the pose through connection 2.

Every publish is logged with entry / return time, thread, connection (SDK handle) and target.

PASS when: the blocked call is the last publish connection 1's thread ever starts (the blocked call
completing is expected; starting another one is the failure), that thread has exited, and every
publish after the release comes from connection 2's thread with the intended target.

The robot receives NO command for several seconds while the publish is blocked.  Run it with the
robot restrained (suspended, no load), as the reviewer asked.

    python -m hwtest.stall_reconnect --robot tron1 --robot-type PF_TRON1A
"""

from __future__ import annotations

import time

from .common import PUBLISH_LOOP_ROBOTS, SafeCommander, SendRecorder, base_parser, confirm, read_pose, run


def main(robot, log, args):
    if args.robot not in PUBLISH_LOOP_ROBOTS:
        raise SystemExit("stall_reconnect applies to tron1 / oli / luna only (tron2 has no publish loop)")
    recorder = SendRecorder(robot, log)

    robot.connect()
    handle1 = robot._robot
    thread1 = robot._publisher
    gen1 = recorder.generation(handle1)
    log.info("connection %d up (publish thread %s)", gen1, thread1.name)
    base = read_pose(robot)
    commander = SafeCommander(robot, log, base)
    commander.send(base)
    time.sleep(0.5)

    confirm(
        args,
        log,
        f"阻塞一次发送约 {2 + args.hold:.0f} s（期间机器人收不到任何指令），断开并重连，然后保持当前姿态",
    )

    # 2. block the next publish of connection 1's loop thread
    recorder.block_next_publish_from(thread1.ident)
    if not recorder.blocked_entered.wait(timeout=2.0):
        log.fail("connection 1's loop did not publish within 2 s; nothing was blocked")
        recorder.release.set()
        return
    t_block = time.monotonic()
    log.event("blocked", thread=thread1.name, handle_gen=gen1)

    # 3. disconnect while the publish is blocked
    t0 = time.monotonic()
    robot.disconnect()
    join_s = time.monotonic() - t0
    log.info("disconnect() returned after %.2f s (thread 1 alive: %s)", join_s, thread1.is_alive())
    log.event("disconnected", join_s=join_s, thread1_alive=thread1.is_alive())
    if join_s < 1.5:
        log.warn("disconnect returned in %.2f s; expected about 2 s while the publish is blocked", join_s)

    # 4. reconnect
    robot.connect()
    handle2 = robot._robot
    thread2 = robot._publisher
    gen2 = recorder.generation(handle2)
    log.info("connection %d up (publish thread %s)", gen2, thread2.name)
    log.event("reconnected", handle_gen=gen2, thread=thread2.name)
    if gen2 == gen1:
        log.fail("reconnect reused the old SDK handle")
    time.sleep(args.hold)

    # 5. release, then hold the pose through connection 2
    t_release = time.monotonic()
    recorder.release.set()
    log.info("released the blocked publish after %.2f s", t_release - t_block)
    log.event("released")
    time.sleep(0.5)
    base2 = read_pose(robot)
    commander2 = SafeCommander(robot, log, base2)
    commander2.send(base2)
    time.sleep(args.observe)

    analyse(log, recorder.snapshot(), thread1, thread2, gen1, gen2, t_release, base2)
    commander2.send(base2)


def analyse(log, records, thread1, thread2, gen1, gen2, t_release, base2):
    blocked = [r for r in records if r["blocked"]]
    if len(blocked) != 1:
        log.fail("expected exactly one blocked publish, found %d", len(blocked))
        return
    b = blocked[0]
    log.info(
        "blocked publish: entered %.3f, returned %.3f (connection %d)",
        b["enter"],
        b["return"],
        b["handle_gen"],
    )

    old_after = [r for r in records if r["thread_id"] == thread1.ident and r["enter"] > b["enter"]]
    if old_after:
        log.fail(
            "connection 1's thread started %d publish(es) after the blocked one (first at %.3f)",
            len(old_after),
            old_after[0]["enter"],
        )
    else:
        log.info("connection 1's thread started no publish after the blocked one: OK")

    if thread1.is_alive():
        log.fail("connection 1's publish thread is still alive after the release")
    else:
        log.info("connection 1's publish thread has exited: OK")

    after = [r for r in records if r["enter"] >= t_release and not r["blocked"]]
    wrong_handle = [r for r in after if r["handle_gen"] != gen2]
    if wrong_handle:
        log.fail(
            "%d publish(es) after the release went to connection %s",
            len(wrong_handle),
            sorted({r["handle_gen"] for r in wrong_handle}),
        )
    from_new = [r for r in after if r["thread_id"] == thread2.ident]
    if not from_new:
        log.fail("connection 2's thread never published after the release")
    else:
        target = [round(v, 6) for v in base2.values()]
        bad = [
            r for r in from_new if max(abs(a - c) for a, c in zip(r["target"], target, strict=True)) > 1e-5
        ]
        if bad:
            log.fail("%d connection-2 loop publishes carried an unexpected target", len(bad))
        log.info(
            "%d publishes after the release, all to connection %d, %d from its loop thread",
            len(after),
            gen2,
            len(from_new),
        )


if __name__ == "__main__":
    p = base_parser(__doc__.splitlines()[0])
    p.add_argument(
        "--hold", type=float, default=1.0, help="seconds to keep the publish blocked after reconnecting"
    )
    p.add_argument("--observe", type=float, default=2.0, help="seconds to watch publishing after the release")
    run(main, "stall_reconnect", p.parse_args())
