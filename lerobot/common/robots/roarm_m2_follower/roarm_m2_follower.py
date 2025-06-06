import json
import logging
import time
from functools import cached_property
from typing import Any

import serial

from lerobot.common.cameras.utils import make_cameras_from_configs
from lerobot.common.constants import OBS_STATE
from lerobot.common.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError

from ..robot import Robot
from .config_roarm_m2_follower import RoArmM2FollowerConfig

logger = logging.getLogger(__name__)


class RoArmM2Follower(Robot):
    config_class = RoArmM2FollowerConfig
    name = "roarm_m2_follower"

    def __init__(self, config: RoArmM2FollowerConfig):
        super().__init__(config)
        self.config = config
        self.ser: serial.Serial | None = None
        self.cameras = make_cameras_from_configs(config.cameras)
        self.motor_names = ["base", "shoulder", "elbow", "hand"]

    @property
    def _motors_ft(self) -> dict[str, type]:
        return {f"{m}.pos": float for m in self.motor_names}

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        return {
            cam: (
                self.config.cameras[cam].height,
                self.config.cameras[cam].width,
                3,
            )
            for cam in self.cameras
        }

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        return {**self._motors_ft, **self._cameras_ft}

    @cached_property
    def action_features(self) -> dict[str, type]:
        return self._motors_ft

    @property
    def is_connected(self) -> bool:
        return self.ser is not None and self.ser.is_open and all(cam.is_connected for cam in self.cameras.values())

    def connect(self, calibrate: bool = True) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")
        self.ser = serial.Serial(self.config.port, 115200, timeout=1.0, dsrdtr=None)
        self.ser.setRTS(False)
        self.ser.setDTR(False)
        for cam in self.cameras.values():
            cam.connect()
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

    def get_observation(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        self._send_cmd({"T": 105})
        fb = self._read_feedback(timeout=0.1) or {}
        angles = [float(fb.get(k[0], 0.0)) for k in self.motor_names]
        obs = {f"{m}.pos": a for m, a in zip(self.motor_names, angles)}
        for cam_key, cam in self.cameras.items():
            obs[cam_key] = cam.async_read()
        return obs

    def send_action(self, action: dict[str, float]) -> dict[str, float]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        angles = {m: action.get(f"{m}.pos", 0.0) for m in self.motor_names}
        cmd = {
            "T": 102,
            "base": angles["base"],
            "shoulder": angles["shoulder"],
            "elbow": angles["elbow"],
            "hand": angles["hand"],
            "spd": 0.6,
            "acc": 10,
        }
        self._send_cmd(cmd)
        return {f"{m}.pos": angles[m] for m in self.motor_names}

    def disconnect(self) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        assert self.ser is not None
        self.ser.close()
        for cam in self.cameras.values():
            cam.disconnect()
        logger.info(f"{self} disconnected.")
