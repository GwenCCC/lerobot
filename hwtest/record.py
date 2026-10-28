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
"""Record a small LeRobot dataset by driving the robot through the drive.py trajectory.

Checklist item "数据录制（脚本）".  ``lerobot-record`` needs a teleoperator whose action keys match
the robot, and none exists for LimX robots yet, so this script records the same way: each frame
holds the observation and the action that ``send_action`` returned.  The dataset stays local (it is
never pushed to the Hugging Face Hub).

After writing, the dataset is reopened and checked:

- the action and observation features are exactly the robot's joint keys, with no extra entries;
- every episode has the expected number of frames;
- the recorded actions stay within the safety limits.

    python -m hwtest.record --robot tron1 --robot-type PF_TRON1A --root ./datasets/limx_tron1_hwtest

Then replay it with ``python -m hwtest.replay_precheck`` followed by ``lerobot-replay`` (see checklist).
"""

from __future__ import annotations

from pathlib import Path

import numpy as np

from .common import MAX_OFFSET, base_parser, confirm, joint_keys, run
from .drive import add_drive_args, default_rate, drive

TASK = "limx hardware test: small sine around the start pose"


def main(robot, log, args):
    from lerobot.datasets import LeRobotDataset
    from lerobot.utils.feature_utils import build_dataset_frame, hw_to_dataset_features

    root = Path(args.root)
    if root.exists():
        raise SystemExit(f"{root} already exists; pick a new --root (LeRobot will not overwrite a dataset)")
    repo_id = args.repo_id or f"local/{root.name}"
    fps = int(args.rate or default_rate(args.robot))
    args.rate = fps

    robot.connect()
    log.info("connected")
    features = {
        **hw_to_dataset_features(robot.action_features, "action", use_video=False),
        **hw_to_dataset_features(robot.observation_features, "observation", use_video=False),
    }
    dataset = LeRobotDataset.create(
        repo_id, fps, features, root=root, robot_type=robot.name, use_videos=False
    )
    log.info("dataset %s at %s, fps=%d", repo_id, root, fps)

    confirm(args, log, f"录制 {args.episodes} 段，每段 {args.duration:.0f} s，±{args.amplitude} rad 正弦运动")
    for episode in range(args.episodes):
        log.info("episode %d / %d", episode + 1, args.episodes)

        def on_frame(obs, sent):
            frame = {
                **build_dataset_frame(dataset.features, obs, prefix="observation"),
                **build_dataset_frame(dataset.features, sent, prefix="action"),
                "task": TASK,
            }
            dataset.add_frame(frame)

        drive(robot, log, args, on_frame=on_frame)
        dataset.save_episode()
    dataset.finalize()
    robot.disconnect()
    log.info("recording done; verifying the dataset")
    verify(robot, log, repo_id, root, args)


def verify(robot, log, repo_id, root, args):
    from lerobot.datasets import LeRobotDataset

    ds = LeRobotDataset(repo_id, root=root)
    keys = joint_keys(robot)
    names = {name.removesuffix(".pos") for name in keys}

    for feature in ("action", "observation.state"):
        if feature not in ds.features:
            log.fail("dataset has no %s feature (features: %s)", feature, list(ds.features))
            continue
        recorded = ds.features[feature]["names"]
        recorded = {str(n).removesuffix(".pos") for n in recorded}
        if recorded != names:
            log.fail("%s names %s differ from the robot joints %s", feature, sorted(recorded), sorted(names))
        else:
            log.info("%s: %d joints, names match the robot", feature, len(recorded))
    extra = [f for f in ds.features if f.startswith(("action.", "observation.")) and f != "observation.state"]
    if extra:
        log.fail("unexpected extra features: %s", extra)

    if ds.meta.total_episodes != args.episodes:
        log.fail("expected %d episodes, found %d", args.episodes, ds.meta.total_episodes)
    else:
        log.info("episodes: %d, frames: %d", ds.meta.total_episodes, ds.meta.total_frames)

    actions = np.stack([np.asarray(ds[i]["action"]) for i in range(len(ds))])
    states = np.stack([np.asarray(ds[i]["observation.state"]) for i in range(len(ds))])
    spread = float(np.max(np.abs(actions - states[0])))
    log.info("largest |action - first observation| in the dataset: %.4f rad", spread)
    if spread > MAX_OFFSET + 0.05:
        log.fail("recorded actions move %.3f rad from the start pose; expected a small trajectory", spread)
    log.info(
        "replay with:  lerobot-replay --robot.type=%s ... --dataset.repo_id=%s --dataset.root=%s "
        "--dataset.episode=0",
        robot.name,
        repo_id,
        root.resolve(),
    )


if __name__ == "__main__":
    p = base_parser(__doc__.splitlines()[0])
    add_drive_args(p)
    p.set_defaults(duration=20.0)
    p.add_argument("--root", required=True, help="new local folder for the dataset")
    p.add_argument("--repo-id", help="dataset name (default local/<root folder name>)")
    p.add_argument("--episodes", type=int, default=2, help="number of episodes (default 2)")
    run(main, "record", p.parse_args())
