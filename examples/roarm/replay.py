import time

from lerobot.common.datasets.lerobot_dataset import LeRobotDataset
from lerobot.common.robots.roarm_m2_follower import RoArmM2Follower, RoArmM2FollowerConfig
from lerobot.common.utils.robot_utils import busy_wait

robot_config = RoArmM2FollowerConfig(port="/dev/ttyUSB1")
robot = RoArmM2Follower(robot_config)

dataset = LeRobotDataset("user/roarm_example", episodes=[0])

robot.connect()

print("Replaying episode…")
for action_array in dataset.hf_dataset["action"]:
    t0 = time.perf_counter()
    action = {name: float(action_array[i]) for i, name in enumerate(dataset.features["action"]["names"])}
    robot.send_action(action)
    busy_wait(max(1.0 / dataset.fps - (time.perf_counter() - t0), 0.0))

print("Disconnecting robot")
robot.disconnect()

