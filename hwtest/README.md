# Hardware test scripts

Scripts for the hardware test checklist. They are not part of the installed package: run them from
a checkout of this repository, from its root folder, with `python -m hwtest.<script>`.

```bash
git clone -b limx-plugin https://github.com/GwenCCC/lerobot.git lerobot_robot_limx
cd lerobot_robot_limx
pip install -e ".[tron1]"          # or [tron2], [oli], [luna]
python -m hwtest.drive --help
```

| Script | Checklist item | Robots |
| --- | --- | --- |
| `hwtest.drive` | 连续驱动 5 分钟 | all |
| `hwtest.record` | 数据录制（脚本） | all |
| `hwtest.replay_precheck` | run before `lerobot-replay` | all |
| `hwtest.order_check` | 指令顺序 | tron1, oli, luna |
| `hwtest.stall_reconnect` | 发送卡住后重连 (huggingface/lerobot#4803) | tron1, oli, luna |

## Common options

| Option | Meaning |
| --- | --- |
| `--robot` | `tron1`, `tron2`, `oli` or `luna` |
| `--ip` | robot address (default: the driver default) |
| `--robot-type` | build string: `PF_TRON1A` / `SF_TRON1A` / `WF_TRON1A` (tron1, default PF), `HU_D04_01` (oli), `HU_L04_01` (luna) |
| `--gains-file` | JSON `{"kp": [...], "kd": [...]}` in joint order. **Required for luna** |
| `--gain-scale` | multiplies kp and kd; default `0.25` (low gains). Use `1.0` for full gains |
| `--state-max-age` | default `0.5` s |
| `--yes` | skip the confirmation prompt |
| `--sim` | run against a built-in stand-in for the SDK, without a robot (rehearsal only) |

## Safety built into every script

- No command may be more than 0.15 rad from the pose the script started in, and no joint may move
  more than 0.02 rad between two consecutive commands; a violating command is refused, not sent.
- The script prints what it is about to do and waits for `yes` before the first motion.
- The robot is disconnected on exit, including Ctrl+C.
- `stall_reconnect` leaves the robot without commands for about 3 s by design: run it suspended.

## Results

Every run writes `hwtest_logs/<script>_<robot>_<time>.log` (readable) and `.jsonl` (every event, for
analysis) and ends with `RESULT: PASS` or `RESULT: FAIL`. Put the log path in the checklist's
"实测记录 / 备注" column. For `stall_reconnect`, attach the `.log` when replying on PR #4803.

Rehearse a script with `--sim` before running it on a robot, e.g.
`python -m hwtest.stall_reconnect --robot tron1 --sim`.
