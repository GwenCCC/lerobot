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
"""Check that the robot is close to an episode's first action before running ``lerobot-replay``.

``lerobot-replay`` sends the first recorded action immediately.  If the robot has drifted from the
pose it was recorded in, that first command is a jump.  This script only reads: it connects,
compares the current pose with the episode's first action and the episode's range of motion, and
reports PASS only when every joint is within ``--tolerance`` (default 0.05 rad) of the start.

    python -m hwtest.replay_precheck --robot tron1 --robot-type PF_TRON1A \
        --root ./datasets/limx_tron1_hwtest --episode 0
"""

from __future__ import annotations

from pathlib import Path

import numpy as np

from .common import MAX_OFFSET, base_parser, joint_keys, read_pose, run


def main(robot, log, args):
    from lerobot.datasets import LeRobotDataset

    root = Path(args.root)
    repo_id = args.repo_id or f"local/{root.name}"
    ds = LeRobotDataset(repo_id, root=root, episodes=[args.episode])
    names = [str(n) for n in ds.features["action"]["names"]]
    actions = np.stack([np.asarray(ds[i]["action"]) for i in range(len(ds))])
    log.info("episode %d: %d frames, fps %d", args.episode, len(actions), ds.fps)

    robot.connect()
    log.info("connected (read only: this script sends no command)")
    pose = read_pose(robot)
    keys = joint_keys(robot)
    key_of = {k.removesuffix(".pos"): k for k in keys}

    worst = 0.0
    for col, name in enumerate(names):
        key = key_of.get(name.removesuffix(".pos"))
        if key is None:
            log.fail("dataset joint %s is not a joint of this robot", name)
            continue
        diff = abs(actions[0, col] - pose[key])
        worst = max(worst, diff)
        if diff > args.tolerance:
            log.fail(
                "%s: current %.4f vs first action %.4f (diff %.4f rad > %.3f)",
                key,
                pose[key],
                actions[0, col],
                diff,
                args.tolerance,
            )
    span = float(np.max(np.abs(actions - actions[0])))
    log.info("largest start difference %.4f rad; episode range of motion %.4f rad", worst, span)
    if span > MAX_OFFSET + 0.05:
        log.fail("episode moves %.3f rad from its first frame; not a small test trajectory", span)


if __name__ == "__main__":
    p = base_parser(__doc__.splitlines()[0])
    p.add_argument("--root", required=True, help="local dataset folder written by hwtest.record")
    p.add_argument("--repo-id", help="dataset name (default local/<root folder name>)")
    p.add_argument("--episode", type=int, default=0)
    p.add_argument("--tolerance", type=float, default=0.05, help="max allowed start difference (rad)")
    run(main, "replay_precheck", p.parse_args())
