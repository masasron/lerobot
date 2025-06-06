from lerobot.common.datasets.utils import build_dataset_frame, hw_to_dataset_features
from lerobot.common.policies.act.modeling_act import ACTPolicy
from lerobot.common.robots.roarm_m2_follower import RoArmM2Follower, RoArmM2FollowerConfig
from lerobot.common.utils.control_utils import predict_action
from lerobot.common.utils.utils import get_safe_torch_device

robot_config = RoArmM2FollowerConfig(port="/dev/ttyUSB1")
robot = RoArmM2Follower(robot_config)

robot.connect()

policy = ACTPolicy.from_pretrained("user/act_roarm")
policy.reset()

obs_features = hw_to_dataset_features(robot.observation_features, "observation")

print("Running inference")
while True:
    obs = robot.get_observation()
    observation_frame = build_dataset_frame(obs_features, obs, prefix="observation")
    action_values = predict_action(
        observation_frame,
        policy,
        get_safe_torch_device(policy.config.device),
        policy.config.use_amp,
    )
    action = {name: action_values[i].item() for i, name in enumerate(robot.action_features)}
    robot.send_action(action)

