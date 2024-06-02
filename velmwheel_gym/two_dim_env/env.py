""" Based on: https://github.com/EmanuelSamir/simple-2d-robot-lidar-env/blob/main/gym_robot2d/envs/robot2d_env.py """

import logging
import math

import gymnasium as gym
import numpy as np

from velmwheel_gym.constants import (
    COORDINATES_NORMALIZATION_FACTOR,
    GLOBAL_GUIDANCE_OBSERVATION_POINTS,
    LIDAR_DATA_SIZE,
)
from velmwheel_gym.global_guidance_path import get_n_points_evenly_spaced_on_path
from velmwheel_gym.reward import calculate_reward
from velmwheel_gym.types import Point

from .robot2d import Robot2D

ACTION_NORMALIZATION_FACTOR = 5.0


logger = logging.getLogger(__name__)


class Robot2dEnv(gym.Env):
    def __init__(
        self,
        dT=0.05,
        is_goal=True,
        **kwargs,
    ):
        super().__init__()

        self._point_reached_threshold = kwargs["point_reached_threshold"]
        self.robot = Robot2D(dT=dT, is_render=True, is_goal=is_goal)
        self.steps = 0

        self.robot_goal = np.array([0.0, 0.0])

        self.xr0 = 0
        self.yr0 = 0
        self.thr0 = 0

        self.action_space = gym.spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(2,),
            dtype=np.float64,
        )

        self.observation_space = gym.spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(4 + 2 * GLOBAL_GUIDANCE_OBSERVATION_POINTS + LIDAR_DATA_SIZE,),
            dtype=np.float64,
        )

    @property
    def max_episode_steps(self) -> int:
        return self._time_limit_max_episode_steps

    @property
    def goal(self) -> Point:
        return Point(*self.robot_goal)

    @property
    def robot_position(self) -> np.array:
        return np.array((self.robot.xr, self.robot.yr))

    @property
    def starting_position(self) -> Point:
        return Point(self.xr0, self.yr0)

    def _observe(self):
        self.robot.scanning()

        xls = self.robot.xls
        yls = self.robot.yls

        if xls.size == 0:
            xls = np.array([self.robot.max_range])
            yls = np.array([0.0])

        # convert LIDAR touches to ranges
        ranges = []
        for xl, yl in zip(xls, yls):
            ranges.append(math.dist(self.robot_position, (xl, yl)))
        # clamp and normalize LIDAR ranges
        ranges = [min(r, self.robot.max_range) / self.robot.max_range for r in ranges]

        obs = [
            self.robot_position[0],
            self.robot_position[1],
            self.goal.x,
            self.goal.y,
        ]

        obs.extend(
            get_n_points_evenly_spaced_on_path(
                self.robot._global_guidance_path, 10, [self.goal.x, self.goal.y]
            )
        )

        # normalize position and goal coordinates
        obs = [o / COORDINATES_NORMALIZATION_FACTOR for o in obs]

        obs.extend(ranges)

        return np.array(obs)

    def step(self, action):
        self.steps += 1
        # State Update
        vx = ACTION_NORMALIZATION_FACTOR * action[0]
        vy = ACTION_NORMALIZATION_FACTOR * action[1]
        w = 0.0

        self.robot.step(vx, vy, w)
        num_passed_points = self.robot._global_guidance_path.update(
            Point(*self.robot_position), self._point_reached_threshold
        )

        reward, terminated = calculate_reward(
            Point(*self.robot_position),
            self.goal,
            self.robot.is_crashed(),
            self._point_reached_threshold,
            num_passed_points,
            self.robot._global_guidance_path,
            self.max_episode_steps,
            self.steps,
        )

        goal_distance = math.dist(self.robot_position, self.robot_goal)

        obs = self._observe()

        logger.debug(
            f"{reward=} {self.robot_position=} {self.robot_goal=} {goal_distance=}"
        )

        return obs, reward, terminated, False, {}

    def reset(self, seed=None, options=None):  # Return to initial state
        self.robot.reset()

        self.steps = 0

        self.xr0 = self.robot.xr
        self.yr0 = self.robot.yr
        self.thr0 = self.robot.thr

        self.robot_goal = np.array([self.robot.xg, self.robot.yg])

        return self._observe(), {}

    def render(self, mode="human"):
        self.robot.render()

    def close(self):
        self.robot.close()
