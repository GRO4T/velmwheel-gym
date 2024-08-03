import logging
from abc import ABC, abstractmethod
from typing import Optional

import gymnasium as gym
import numpy as np

from velmwheel_gym.constants import (
    COORDINATES_NORMALIZATION_FACTOR,
    GLOBAL_GUIDANCE_OBSERVATION_POINTS,
    LIDAR_DATA_SIZE,
    LIDAR_MAX_RANGE,
    NAVIGATION_DIFFICULTIES,
)
from velmwheel_gym.gazebo_env.start_position_and_goal_generator import (
    StartPositionAndGoalGenerator,
)
from velmwheel_gym.metrics import Metrics
from velmwheel_gym.types import Point

logger = logging.getLogger(__name__)


class VelmwheelBaseEnv(gym.Env):
    def __init__(self, **kwargs):
        logger.debug("VelmwheelBaseEnv.__init__ -> enter")
        super().__init__()

        self._render_mode = kwargs.get("render_mode", None)
        self._training_mode = kwargs.get("training_mode", False)
        self._difficulty = kwargs.get("difficulty", NAVIGATION_DIFFICULTIES[0])

        self.action_space = gym.spaces.Box(
            low=-1.0, high=1.0, shape=(3,), dtype=np.float64
        )
        self.observation_space = gym.spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(3 + 2 * GLOBAL_GUIDANCE_OBSERVATION_POINTS + LIDAR_DATA_SIZE,),
            dtype=np.float64,
        )

        self._steps = 0
        self._metrics = Metrics()
        self._start_position_and_goal_generator = StartPositionAndGoalGenerator()

    @property
    @abstractmethod
    def goal(self) -> Point:
        pass

    @property
    @abstractmethod
    def robot_position(self) -> Point:
        pass

    @property
    @abstractmethod
    def starting_position(self) -> Point:
        pass

    @property
    @abstractmethod
    def is_final_goal(self) -> bool:
        pass

    def _relative_to_robot(self, obs: np.array) -> np.array:
        robot_position = self.robot_position
        obs[1] -= robot_position[0]
        obs[2] -= robot_position[1]
        for i in range(3, 3 + 2 * GLOBAL_GUIDANCE_OBSERVATION_POINTS, 2):
            obs[i] -= robot_position[0]
            obs[i + 1] -= robot_position[1]
        return obs

    def _normalize_observation(self, obs: np.array) -> np.array:
        obs[0] /= np.pi
        obs[1] /= COORDINATES_NORMALIZATION_FACTOR
        obs[2] /= COORDINATES_NORMALIZATION_FACTOR
        for i in range(3, 3 + 2 * GLOBAL_GUIDANCE_OBSERVATION_POINTS, 2):
            obs[i] /= COORDINATES_NORMALIZATION_FACTOR
            obs[i + 1] /= COORDINATES_NORMALIZATION_FACTOR
        obs[-LIDAR_DATA_SIZE:] = [
            2 * min(scan, LIDAR_MAX_RANGE) / LIDAR_MAX_RANGE - 1
            for scan in obs[-LIDAR_DATA_SIZE:]
        ]
        return obs
