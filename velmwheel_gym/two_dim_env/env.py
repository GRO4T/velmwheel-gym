""" Based on: https://github.com/EmanuelSamir/simple-2d-robot-lidar-env/blob/main/gym_robot2d/envs/robot2d_env.py """

import logging
import math
from collections import deque

import gymnasium as gym
import numpy as np

from velmwheel_gym.constants import (
    COORDINATES_NORMALIZATION_FACTOR,
    GLOBAL_GUIDANCE_OBSERVATION_POINTS,
    LIDAR_DATA_SIZE,
    NAVIGATION_DIFFICULTIES,
    STATS_BUFFER_SIZE,
)
from velmwheel_gym.global_guidance_path import get_n_points_evenly_spaced_on_path
from velmwheel_gym.reward import calculate_reward
from velmwheel_gym.types import NavigationDifficulty, Point

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

        self._difficulty: NavigationDifficulty = kwargs["difficulty"]
        self.robot = Robot2D(
            dT=dT, is_render=True, is_goal=is_goal, difficulty=self._difficulty
        )
        self.steps = 0
        self._episode = 0
        self._total_reward = 0.0
        self._reward_buffer = deque(maxlen=STATS_BUFFER_SIZE)
        self._success_buffer = deque(maxlen=STATS_BUFFER_SIZE)

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

        goal_x_relative = self.goal.x - self.robot_position[0]
        goal_y_relative = self.goal.y - self.robot_position[1]

        obs = [
            self.robot_position[0],
            self.robot_position[1],
            goal_x_relative,
            goal_y_relative,
        ]

        obs.extend(
            get_n_points_evenly_spaced_on_path(
                self.robot._global_guidance_path,
                10,
                [goal_x_relative, goal_y_relative],
                Point(*self.robot_position),
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
            Point(*self.robot_position)
        )

        reward, terminated = calculate_reward(
            Point(*self.robot_position),
            self.goal,
            self.robot.is_crashed(),
            self._difficulty,
            num_passed_points,
            self.robot._global_guidance_path,
            self.max_episode_steps,
            self.steps,
        )

        obs = self._observe()

        self._total_reward += reward

        if terminated:
            self._reward_buffer.append(self._total_reward)
            self._success_buffer.append(1 if reward > 0 else 0)
            mean_reward = (
                np.mean(self._reward_buffer)
                if len(self._reward_buffer) == STATS_BUFFER_SIZE
                else "N/A"
            )
            success_rate = (
                np.mean(self._success_buffer)
                if len(self._success_buffer) == STATS_BUFFER_SIZE
                else "N/A"
            )
            idx = NAVIGATION_DIFFICULTIES.index(self._difficulty)
            logger.debug(
                f"episode={self._episode} level={idx} reward={self._total_reward} mean_reward={mean_reward} success_rate={success_rate}"
            )
            if (
                idx < len(NAVIGATION_DIFFICULTIES) - 1
                and len(self._reward_buffer) == STATS_BUFFER_SIZE
                and mean_reward > 0.7
            ):
                logger.info(
                    "Mean reward ({mean_reward}) > 0.7. Advancing to the next level."
                )
                self._reward_buffer.clear()
                self._success_buffer.clear()
                self._difficulty = NAVIGATION_DIFFICULTIES[idx + 1]
                self.robot._difficulty = self._difficulty
                self.robot._global_guidance_path._difficulty = self._difficulty

        return obs, reward, terminated, False, {}

    def reset(self, seed=None, options=None):  # Return to initial state
        self.robot.reset()
        self._episode += 1

        self.steps = 0
        self._total_reward = 0.0

        self.xr0 = self.robot.xr
        self.yr0 = self.robot.yr
        self.thr0 = self.robot.thr

        self.robot_goal = np.array([self.robot.xg, self.robot.yg])

        return self._observe(), {}

    def render(self, mode="human"):
        self.robot.render()

    def close(self):
        self.robot.close()
