""" Based on: https://github.com/EmanuelSamir/simple-2d-robot-lidar-env/blob/main/gym_robot2d/envs/robot2d_env.py """

import math
import logging

import gymnasium as gym
import numpy as np

from velmwheel_gym.constants import (
    COORDINATES_NORMALIZATION_FACTOR,
    GLOBAL_GUIDANCE_OBSERVATION_POINTS,
    LIDAR_DATA_SIZE,
)
from velmwheel_gym.types import Point

from .robot2d import Robot2D

ACTION_NORMALIZATION_FACTOR = 5.0


logger = logging.getLogger(__name__)


class Robot2dEnv(gym.Env):
    def __init__(
        self,
        dT=0.05,
        is_goal=True,
        eps_err=0.4,
        max_action_magnitude=5,
        **kwargs,
    ):
        super().__init__()

        # Initialize variables
        self.state = None
        self.viewer = None
        self.is_goal = is_goal
        self.dT = dT

        self.robot = Robot2D(dT=self.dT, is_render=True, is_goal=self.is_goal)
        self.eps_err = eps_err
        self.steps = 0

        self.max_action_magnitude = max_action_magnitude

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

        position_normalized = self.robot_position / COORDINATES_NORMALIZATION_FACTOR
        goal_normalized = self.robot_goal / COORDINATES_NORMALIZATION_FACTOR
        empty_global_guidace = np.zeros(
            2 * GLOBAL_GUIDANCE_OBSERVATION_POINTS
        )  # NOTE: At the moment global planner is not implemented

        return np.concatenate(
            (position_normalized, goal_normalized, empty_global_guidace, ranges)
        )

    def step(self, action):
        # State Update
        vx = ACTION_NORMALIZATION_FACTOR * action[0]
        vy = ACTION_NORMALIZATION_FACTOR * action[1]
        w = 0.0

        self.robot.step(vx, vy, w)

        terminated = False
        reward = 0.0

        goal_distance = math.dist(self.robot_position, self.robot_goal)

        if self.robot.is_crashed():
            reward = -1.0
            terminated = True
        else:
            if goal_distance < 1.0:
                reward = 1.0 - 0.1 * self.steps / self.max_episode_steps
                terminated = True
            else:
                goal_closeness_factor = (
                    1
                    - math.dist(self.robot_position, self.robot_goal)
                    / COORDINATES_NORMALIZATION_FACTOR
                )
                reward = goal_closeness_factor * 0.1 / self.max_episode_steps
                self.steps += 1

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
