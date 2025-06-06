from lerobot.common.robots.roarm_m2_follower import RoArmM2Follower, RoArmM2FollowerConfig
from lerobot.common.teleoperators.roarm_m2_leader import RoArmM2Leader, RoArmM2LeaderConfig

robot_config = RoArmM2FollowerConfig(port="/dev/ttyUSB1")
teleop_config = RoArmM2LeaderConfig(port="/dev/ttyUSB0")

robot = RoArmM2Follower(robot_config)
teleop = RoArmM2Leader(teleop_config)

robot.connect()
teleop.connect()

try:
    while True:
        action = teleop.get_action()
        robot.send_action(action)
except KeyboardInterrupt:
    pass
finally:
    teleop.disconnect()
    robot.disconnect()
