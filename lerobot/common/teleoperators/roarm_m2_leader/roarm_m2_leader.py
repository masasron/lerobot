import json
import logging
import time
from typing import Any

import serial

from lerobot.common.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from ..teleoperator import Teleoperator
from .config_roarm_m2_leader import RoArmM2LeaderConfig

logger = logging.getLogger(__name__)


class RoArmM2Leader(Teleoperator):
    config_class = RoArmM2LeaderConfig
    name = "roarm_m2_leader"

    def __init__(self, config: RoArmM2LeaderConfig):
        super().__init__(config)
        self.config = config
        self.ser: serial.Serial | None = None
        self.motor_names = ["base", "shoulder", "elbow", "hand"]

    @property
    def action_features(self) -> dict[str, type]:
        return {f"{m}.pos": float for m in self.motor_names}

    @property
    def feedback_features(self) -> dict[str, type]:
        return {}

    @property
    def is_connected(self) -> bool:
        return self.ser is not None and self.ser.is_open

    def connect(self, calibrate: bool = True) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")
        self.ser = serial.Serial(self.config.port, 115200, timeout=1.0, dsrdtr=None)
        self.ser.setRTS(False)
        self.ser.setDTR(False)
        logger.info(f"{self} connected.")

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass

    def _send_cmd(self, payload: dict) -> None:
        assert self.ser is not None
        self.ser.write((json.dumps(payload) + "\n").encode("utf-8"))
        self.ser.flush()

    def _read_feedback(self, timeout: float) -> dict | None:
        assert self.ser is not None
        end = time.time() + timeout
        while time.time() < end:
            line = self.ser.readline().decode(errors="ignore").strip()
            if not line:
                continue
            try:
                data = json.loads(line)
            except json.JSONDecodeError:
                continue
            if data.get("T") == 1051:
                return data
        return None

    def get_action(self) -> dict[str, float]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        self._send_cmd({"T": 105})
        fb = self._read_feedback(timeout=0.1) or {}
        angles = [float(fb.get(k[0], 0.0)) for k in self.motor_names]
        return {f"{m}.pos": a for m, a in zip(self.motor_names, angles)}

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        pass

    def disconnect(self) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        assert self.ser is not None
        self.ser.close()
        logger.info(f"{self} disconnected.")
