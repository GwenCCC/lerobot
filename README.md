# lerobot_robot_limx

[LeRobot](https://github.com/huggingface/lerobot) plugin for [LimX Dynamics](https://www.limxdynamics.com/) robots.
Install it next to LeRobot and the robots below become available to every LeRobot tool
(`lerobot-record`, `lerobot-teleoperate`, `lerobot-replay`, `lerobot-calibrate`, `lerobot-rollout`)
with no change to LeRobot itself.

| `--robot.type` | Robot                                     | Joints | Vendor runtime | Extra   | Guide                                    |
| -------------- | ----------------------------------------- | ------ | -------------- | ------- | ---------------------------------------- |
| `limx_tron1`   | TRON1 point / sole / wheel foot           | 6 / 8  | `limxsdk`      | `tron1` | [docs/limx_tron1.md](docs/limx_tron1.md) |
| `limx_tron2`   | TRON2 dual-arm (2×7-DoF arms, 2-DoF head) | 16     | `tron2-env`    | `tron2` | [docs/limx_tron2.md](docs/limx_tron2.md) |
| `limx_oli`     | Oli full-size humanoid (`HU_D04`)         | 31     | `limxsdk`      | `oli`   | [docs/limx_oli.md](docs/limx_oli.md)     |
| `limx_luna`    | Luna humanoid (`HU_L04`)                  | 27     | `limxsdk`      | `luna`  | [docs/limx_luna.md](docs/limx_luna.md)   |

> [!NOTE]
> Oli was proposed upstream as `limx_humanoid` (huggingface/lerobot#4800). In this plugin its robot
> type is `limx_oli`, and the classes are `LimxOli` / `LimxOliConfig`.

## Install

Install the extra for your robot, so only the vendor runtime you need is pulled in. LeRobot itself,
with the dependencies `lerobot-record` and `lerobot-teleoperate` need (`lerobot[core-scripts]`), is
installed automatically:

```bash
pip install "lerobot_robot_limx[tron1]"      # or [tron2], [oli], [luna], [all]
```

From a checkout, for development:

```bash
pip install -e ".[tron1,tron2,test]"
```

`limxsdk` publishes no macOS wheel, so the `tron1`, `oli` and `luna` extras skip it on macOS;
connecting there fails with an install hint. Vendor runtimes are imported only on `connect()`, so
`--robot.type=...` is always available for `--help` and config parsing.

## Quick start

```bash
lerobot-record \
  --robot.type=limx_tron1 \
  --robot.robot_ip=10.192.1.2 \
  --robot.robot_type=PF_TRON1A \
  --robot.state_max_age=0.5 \
  ...
```

```python
from lerobot_robot_limx import LimxTron1, LimxTron1Config

robot = LimxTron1(LimxTron1Config(robot_ip="10.192.1.2", robot_type="PF_TRON1A"))
robot.connect()
print(robot.get_observation())
robot.disconnect()
```

Each robot's guide covers connecting to real hardware, the configuration fields, joint names, gains,
control mode, safety notes and troubleshooting.

## How LeRobot finds this plugin

LeRobot's CLI tools call `register_third_party_plugins()`, which imports every installed distribution
whose name starts with `lerobot_robot_`. Importing `lerobot_robot_limx` registers the four config
classes above, and LeRobot resolves the robot class from each config's name (`LimxTron1Config` →
`LimxTron1`). `tests/test_plugin_discovery.py` checks exactly this path against the installed LeRobot.

## Compatibility

| LeRobot | Status                                   |
| ------- | ---------------------------------------- |
| 0.6.1   | Tested                                   |
| main    | Tested in CI weekly (allowed to fail)    |

The upper bound in `pyproject.toml` is raised after CI passes on a new LeRobot minor release.

## Hardware validation

All four drivers are tested against stand-ins for the vendor SDKs (and against the real `limxsdk`
and `tron2-env` packages where installed), but **none has been validated on a physical robot yet**.
Validated combinations will be listed here:

| Robot | Build | Firmware | SDK version | Plugin commit | Result |
| ----- | ----- | -------- | ----------- | ------------- | ------ |
| —     |       |          |             |               |        |

## Safety

- Run legged and humanoid robots suspended (gantry or stand) the first time, with an emergency stop at
  hand.
- Set `state_max_age` (around `0.5` s) on hardware so stale state is rejected rather than fed to a policy.
- Default gains are the vendor's **standing** gains where noted; a policy should run with the same
  `kp`/`kd` it was recorded or trained with.
- The vendor control channels are plaintext and unauthenticated: keep them on a trusted local network.

## Development

```bash
pip install -e ".[tron1,tron2,test]" ruff==0.14.1
ruff check . && ruff format --check .
pytest -q
```

The lint rules match `huggingface/lerobot`, so code can move between the two unchanged.

## License

Apache-2.0. See [LICENSE](LICENSE).
