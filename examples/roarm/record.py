import time

from lerobot.common.datasets.lerobot_dataset import LeRobotDataset
from lerobot.common.datasets.utils import hw_to_dataset_features
from lerobot.common.robots.roarm_m2_follower import RoArmM2Follower, RoArmM2FollowerConfig
from lerobot.common.teleoperators.roarm_m2_leader import RoArmM2Leader, RoArmM2LeaderConfig

NB_STEPS = 250

robot_config = RoArmM2FollowerConfig(port="/dev/ttyUSB1")
teleop_config = RoArmM2LeaderConfig(port="/dev/ttyUSB0")

robot = RoArmM2Follower(robot_config)
teleop = RoArmM2Leader(teleop_config)

action_features = hw_to_dataset_features(robot.action_features, "action")
obs_features = hw_to_dataset_features(robot.observation_features, "observation")
dataset_features = {**action_features, **obs_features}

dataset = LeRobotDataset.create(
    repo_id="user/roarm" + str(int(time.time())),
    fps=10,
    features=dataset_features,
    robot_type=robot.name,
)

teleop.connect()
robot.connect()

if not teleop.is_connected or not robot.is_connected:
    exit()

print("Starting RoArm teleoperation recording")
for _ in range(NB_STEPS):
    action = teleop.get_action()
    sent = robot.send_action(action)
    obs = robot.get_observation()
    dataset.add_frame({**sent, **obs}, "RoArm dataset example")

print("Disconnecting devices")
teleop.disconnect()
robot.disconnect()

print("Uploading dataset to the hub")
dataset.save_episode()
dataset.push_to_hub()

