# LeRobot Policy Module

`LeRobotPolicyModule` runs trained LeRobot policies in a managed Python-native
subprocess. Its LeRobot, Transformers, Torch, and NumPy versions live in the
sibling `python/` project and do not change the main DimOS environment.

The host contract subscribes to:

- `color_image: Image`
- `coordinator_joint_state: JointState`

It publishes `joint_command: JointState` in the configured `joint_names` order.
The receiving coordinator and hardware stack must enforce joint limits and
other actuation safety constraints.

```python
from dimos.imitation.policy.lerobot.module import LeRobotPolicyModule

policy = LeRobotPolicyModule.blueprint(
    policy_path="outputs/pick/checkpoints/last/pretrained_model",
    task="pick up the object",
    joint_names=["arm/joint1", "arm/joint2", "arm/gripper"],
    fps=30.0,
    robot_type="my_robot",
)
```

The module exposes `start_rollout`, `stop_rollout`, and `rollout_status` RPCs.
It owns one configured checkpoint and loads it lazily on the first rollout.
The runtime rejects missing or stale observations, missing joints, non-finite
values, incompatible checkpoint features, and actions with the wrong dimension.

Run the OpenYAM rollout stack with a trained checkpoint:

```bash
uv run dimos run learning-rollout-quest-openyam \
  --LeRobotPolicyModule.policy-path \
    outputs/train/last/pretrained_model
```

Press Quest **A** to start or stop rollout. Hold the right controller grip to
teleoperate; teleoperation has higher coordinator priority and immediately
returns rollout to inactive. The policy must be started again after any
preemption. The checkpoint publishes all configured joints, including the
gripper, through the dedicated low-priority `policy_rollout` coordinator task.

Run isolated runtime checks with:

```bash
cd dimos/imitation/policy/lerobot/python
uv sync --locked --group tests
uv run --locked --group tests pytest
uv run --locked --group tests mypy
```
