from dataclasses import dataclass

from ..config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("roarm_m2_leader")
@dataclass
class RoArmM2LeaderConfig(TeleoperatorConfig):
    port: str
