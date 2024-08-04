import logging
from abc import ABC, abstractmethod

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
from velmwheel_gym.global_guidance_path import GlobalGuidancePath, next_segment
from velmwheel_gym.metrics import Metrics
from velmwheel_gym.types import NavigationDifficulty, Point

logger = logging.getLogger(__name__)


class VelmwheelBaseEnv(gym.Env):
    def __init__(self, **kwargs):
        logger.debug("VelmwheelBaseEnv.__init__ -> enter")
        super().__init__()

        self._render_mode = kwargs.get("render_mode", None)
        self._training_mode = kwargs.get("training_mode", False)
        self._difficulty = kwargs.get("difficulty", NAVIGATION_DIFFICULTIES[0])
        self._global_path_segment_length = kwargs.get("global_path_segment_length", 5.0)
        self._render_freq = kwargs.get("render_freq", 50)
        self._env_name = kwargs.get("env_name", "VelmwheelBaseEnv")
        self._variant = kwargs.get("variant", "EasierFollowing")
        self._name = kwargs.get("name", "VelmwheelBaseEnv")

        self.action_space = gym.spaces.Box(
            low=-1.0, high=1.0, shape=(3,), dtype=np.float64
        )
        match self._variant:
            case "EasierFollowing":
                self.observation_space = gym.spaces.Box(
                    low=-1.0,
                    high=1.0,
                    shape=(
                        7 + 2 * GLOBAL_GUIDANCE_OBSERVATION_POINTS + LIDAR_DATA_SIZE,
                    ),
                    dtype=np.float64,
                )
            case "HarderFollowing":
                self.observation_space = gym.spaces.Box(
                    low=-1.0,
                    high=1.0,
                    shape=(
                        6 + 2 * GLOBAL_GUIDANCE_OBSERVATION_POINTS + LIDAR_DATA_SIZE,
                    ),
                    dtype=np.float64,
                )

        self._steps = 0
        self._metrics = Metrics()
        self._generate_next_goal = True
        self._vx = 0.0
        self._vy = 0.0

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

    @property
    @abstractmethod
    def _global_path(self) -> GlobalGuidancePath:
        pass

    @_global_path.setter
    @abstractmethod
    def _global_path(self, value: GlobalGuidancePath):
        pass

    @property
    @abstractmethod
    def _global_path_segment(self) -> GlobalGuidancePath:
        pass

    @_global_path_segment.setter
    @abstractmethod
    def _global_path_segment(self, value: GlobalGuidancePath):
        pass

    @property
    @abstractmethod
    def _start_position_and_goal_generator(self) -> StartPositionAndGoalGenerator:
        pass

    @abstractmethod
    def _update_difficulty(self, difficulty: NavigationDifficulty):
        pass

    def _relative_to_robot(self, obs: np.array) -> np.array:
        robot_position = self.robot_position
        obs[4] -= robot_position[0]
        obs[5] -= robot_position[1]
        global_path_start_idx = 7 if self._variant == "EasierFollowing" else 6
        for i in range(
            global_path_start_idx,
            global_path_start_idx + 2 * GLOBAL_GUIDANCE_OBSERVATION_POINTS,
            2,
        ):
            obs[i] -= robot_position[0]
            obs[i + 1] -= robot_position[1]
        return obs

    def _normalize_observation(self, obs: np.array) -> np.array:
        obs[1] /= np.pi
        obs[2] *= 2.0
        obs[3] *= 2.0
        obs[4] /= COORDINATES_NORMALIZATION_FACTOR
        obs[5] /= COORDINATES_NORMALIZATION_FACTOR
        global_path_start_idx = 7 if self._variant == "EasierFollowing" else 6
        for i in range(
            global_path_start_idx,
            global_path_start_idx + 2 * GLOBAL_GUIDANCE_OBSERVATION_POINTS,
            2,
        ):
            obs[i] /= COORDINATES_NORMALIZATION_FACTOR
            obs[i + 1] /= COORDINATES_NORMALIZATION_FACTOR
        obs[-LIDAR_DATA_SIZE:] = [
            2 * min(scan, LIDAR_MAX_RANGE) / LIDAR_MAX_RANGE - 1
            for scan in obs[-LIDAR_DATA_SIZE:]
        ]
        return obs

    def _get_current_level(self) -> int:
        return NAVIGATION_DIFFICULTIES.index(self._difficulty)

    def _get_max_level(self) -> int:
        return len(NAVIGATION_DIFFICULTIES) - 1

    def _get_current_target(self) -> Point:
        return (
            self._global_path_segment.points[0]
            if self._global_path_segment.points
            else self.goal
        )

    def _register_episode_terminated(self, success: bool):
        self._generate_next_goal = True
        if success:
            self._metrics.register_local_episode_result(1)
            if self.is_final_goal:
                self._metrics.register_global_episode_result(1)
                logger.debug("Successfully reached the final goal")
                if self._training_mode:
                    self._start_position_and_goal_generator.register_goal_reached()
            else:
                logger.debug("Successfully reached global path segment")
                (
                    self._global_path.points,
                    self._global_path_segment,
                ) = next_segment(
                    self._global_path.points,
                    self._global_path_segment.points,
                    Point(*self.robot_position),
                    self._difficulty,
                    self._global_path_segment_length,
                )
                self._generate_next_goal = False

    def _advance_to_next_level(self):
        logger.info(
            f"Global success rate ({self._metrics.global_success_rate}) > 0.7. Advancing to the next level."
        )
        self._metrics.advance_to_next_level()
        self._update_difficulty(NAVIGATION_DIFFICULTIES[self._get_current_level() + 1])
