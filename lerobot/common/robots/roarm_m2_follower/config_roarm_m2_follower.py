from dataclasses import dataclass, field

from lerobot.common.cameras import CameraConfig
from ..config import RobotConfig


@RobotConfig.register_subclass("roarm_m2_follower")
@dataclass
class RoArmM2FollowerConfig(RobotConfig):
    port: str
    cameras: dict[str, CameraConfig] = field(default_factory=dict)
