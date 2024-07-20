""" Based on: https://github.com/EmanuelSamir/simple-2d-robot-lidar-env/blob/main/gym_robot2d/envs/robot2d_env.py """

import logging
import math
from collections import deque

import gymnasium as gym
import numpy as np

import wandb
from velmwheel_gym.constants import (
    COORDINATES_NORMALIZATION_FACTOR,
    GLOBAL_GUIDANCE_OBSERVATION_POINTS,
    LIDAR_DATA_SIZE,
    NAVIGATION_DIFFICULTIES,
    STATS_BUFFER_SIZE,
)
from velmwheel_gym.global_guidance_path import (
    get_n_points_evenly_spaced_on_path,
    next_segment,
)
from velmwheel_gym.reward import calculate_reward
from velmwheel_gym.types import NavigationDifficulty, Point

from .robot2d import Robot2D

ACTION_NORMALIZATION_FACTOR = 5.0


logger = logging.getLogger(__name__)


class Robot2dEnv(gym.Env):
    def __init__(
        self,
        dT=0.05,
        **kwargs,
    ):
        super().__init__()

        self._training_mode = kwargs["training_mode"]
        self._difficulty: NavigationDifficulty = kwargs["difficulty"]
        self.robot = Robot2D(dT=dT, is_render=True, difficulty=self._difficulty)
        self._render_mode = kwargs.get("render_mode", None)
        self.steps = 0
        self._episode = 0
        self._total_reward = 0.0
        self._reward_buffer = deque(maxlen=STATS_BUFFER_SIZE)
        self._mean_reward = -2.0
        self._local_success_buffer = deque(maxlen=STATS_BUFFER_SIZE)
        self._local_success_rate = -2.0
        self._global_success_buffer = deque(maxlen=STATS_BUFFER_SIZE)
        self._global_success_rate = -2.0
        self._generate_next_goal = True

        self.xr0 = 0
        self.yr0 = 0
        self.thr0 = 0

        self.action_space = gym.spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(3,),
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
        goal_x = (
            self.robot.global_path_segment.points[-1].x
            if self.robot.global_path_segment.points
            else self.robot.xg
        )
        goal_y = (
            self.robot.global_path_segment.points[-1].y
            if self.robot.global_path_segment.points
            else self.robot.yg
        )
        return Point(goal_x, goal_y)

    @property
    def robot_position(self) -> np.array:
        return np.array((self.robot.xr, self.robot.yr))

    @property
    def starting_position(self) -> Point:
        return Point(self.xr0, self.yr0)

    @property
    def is_final_goal(self) -> bool:
        return not self.robot.global_path.points

    def _observe(self, alpha: float):
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
            goal_x_relative,
            goal_y_relative,
        ]

        obs.extend(
            get_n_points_evenly_spaced_on_path(
                self.robot.global_path_segment.points,
                10,
                [goal_x_relative, goal_y_relative],
                Point(*self.robot_position),
            )
        )

        # normalize position and goal coordinates
        obs = [o / COORDINATES_NORMALIZATION_FACTOR for o in obs]

        obs.extend(ranges)

        return np.array([1.0 if self.is_final_goal else 0.0] + [alpha / np.pi] + obs)

    def step(self, action):
        self.steps += 1

        vx = ACTION_NORMALIZATION_FACTOR * action[0]
        vy = ACTION_NORMALIZATION_FACTOR * action[1]
        w = ACTION_NORMALIZATION_FACTOR * action[2]
        self.robot.step(vx, vy, w)

        num_passed_points = self.robot.global_path_segment.update(
            Point(*self.robot_position)
        )

        target = (
            self.robot.global_path_segment.points[0]
            if self.robot.global_path_segment.points
            else self.goal
        )
        alpha = _angle_between(self.robot_position, target, self.robot.thr)

        success, reward, terminated = calculate_reward(
            self.is_final_goal,
            Point(*self.robot_position),
            alpha,
            self.goal,
            self.robot.is_crashed(),
            self._difficulty,
            num_passed_points,
            self.robot.global_path_segment,
            self.max_episode_steps,
            self.steps,
        )

        obs = self._observe(alpha)

        self._total_reward += reward

        if terminated:
            self._generate_next_goal = True
            if success:
                if self.is_final_goal:
                    self._global_success_buffer.append(1)
                    logger.debug("Successfully reached the final goal")
                    self.robot._start_position_and_goal_generator.register_goal_reached()
                else:
                    logger.debug("Successfully reached global path segment")
                    (
                        self.robot.global_path.points,
                        self.robot.global_path_segment,
                    ) = next_segment(
                        self.robot.global_path.points,
                        self.robot.global_path_segment.points,
                        Point(*self.robot_position),
                        self._difficulty,
                    )
                    self._generate_next_goal = False
            else:
                self._global_success_buffer.append(0)
            self._reward_buffer.append(self._total_reward)
            self._local_success_buffer.append(1 if success else 0)
            self._mean_reward = (
                np.mean(self._reward_buffer)
                if len(self._reward_buffer) == STATS_BUFFER_SIZE
                else -2.0
            )
            self._local_success_rate = (
                np.mean(self._local_success_buffer)
                if len(self._local_success_buffer) == STATS_BUFFER_SIZE
                else -2.0
            )
            self._global_success_rate = (
                np.mean(self._global_success_buffer)
                if len(self._global_success_buffer) == STATS_BUFFER_SIZE
                else -2.0
            )
            idx = NAVIGATION_DIFFICULTIES.index(self._difficulty)
            logger.debug(
                f"episode={self._episode} level={idx} reward={self._total_reward} mean_reward={self._mean_reward} success_rate={self._local_success_rate}"
            )
            if (
                idx < len(NAVIGATION_DIFFICULTIES) - 1
                and len(self._reward_buffer) == STATS_BUFFER_SIZE
                and self._global_success_rate > 0.7
            ):
                logger.info(
                    f"Mean reward ({self._mean_reward}) > 0.7. Advancing to the next level."
                )
                self._reward_buffer.clear()
                self._local_success_buffer.clear()
                self._global_success_buffer.clear()
                self._mean_reward = -2.0
                self._local_success_rate = -2.0
                self._global_success_rate = -2.0
                self._difficulty = NAVIGATION_DIFFICULTIES[idx + 1]
                self.robot._difficulty = self._difficulty
                self.robot.global_path._difficulty = self._difficulty
                self.robot.global_path_segment._difficulty = self._difficulty

        if self._episode % 1 == 0 and self._render_mode == "human":
            self.render()

        if self._training_mode:
            metrics = {
                "level": NAVIGATION_DIFFICULTIES.index(self._difficulty),
                "mean_reward": self._mean_reward,
                "local_success_rate": self._local_success_rate,
                "global_success_rate": self._global_success_rate,
            }
            if terminated:
                metrics["episode_reward"] = self._total_reward
            wandb.log(metrics)

        return obs, reward, terminated, False, {}

    def reset(self, seed=None, options=None):  # Return to initial state
        if self.steps >= self.max_episode_steps:
            if self.is_final_goal:
                logger.debug("Did not reach the final goal in time")
                self._generate_next_goal = True
            else:
                logger.debug("Did not reach global path segment in time")
                (
                    self.robot.global_path.points,
                    self.robot.global_path_segment,
                ) = next_segment(
                    self.robot.global_path.points,
                    self.robot.global_path_segment.points,
                    Point(*self.robot_position),
                    self._difficulty,
                )

        self.steps = 0
        self._total_reward = 0.0

        if self._generate_next_goal:
            self._generate_next_goal = False
            self.robot.reset()
            self._episode += 1

            self.xr0 = self.robot.xr
            self.yr0 = self.robot.yr
            self.thr0 = self.robot.thr

        target = (
            self.robot.global_path_segment.points[0]
            if self.robot.global_path_segment.points
            else self.goal
        )
        alpha = _angle_between(self.robot_position, target, self.robot.thr)

        return self._observe(alpha), {}

    def render(self, mode="human"):
        self.robot.render()

    def close(self):
        self.robot.close()


def _angle_between(robot_pos, goal_pos, theta):
    # Robot's position (x_r, y_r)
    x_r, y_r = robot_pos

    # Goal position (x_g, y_g)
    x_g, y_g = goal_pos

    # Robot's heading vector
    R = np.array([np.cos(theta), np.sin(theta)])

    # Goal direction vector
    G = np.array([x_g - x_r, y_g - y_r])

    # Normalize goal direction vector
    G = G / np.linalg.norm(G)

    # Calculate dot product
    dot_product = np.dot(R, G)

    # Calculate angle between the vectors
    alpha = np.arccos(dot_product)

    return alpha
