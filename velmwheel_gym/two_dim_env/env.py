""" Based on: https://github.com/EmanuelSamir/simple-2d-robot-lidar-env/blob/main/gym_robot2d/envs/robot2d_env.py """

import logging
import math

import numpy as np

from velmwheel_gym.base_env import VelmwheelBaseEnv
from velmwheel_gym.constants import BASE_STEP_TIME
from velmwheel_gym.gazebo_env.start_position_and_goal_generator import (
    StartPositionAndGoalGenerator,
)
from velmwheel_gym.global_guidance_path import (
    GlobalGuidancePath,
    get_n_points_evenly_spaced_on_path,
    next_segment,
)
from velmwheel_gym.reward import calculate_reward
from velmwheel_gym.types import NavigationDifficulty, Point
from velmwheel_gym.utils import angle_between_robot_and_goal

from .robot2d import Robot2D

logger = logging.getLogger(__name__)


class Velmwheel2DEnv(VelmwheelBaseEnv):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        self.robot = Robot2D(
            dT=BASE_STEP_TIME,
            is_render=True,
            difficulty=self._difficulty,
            name=f"{self._env_name}_{self._variant}",
            global_path_segment_length=self._global_path_segment_length,
        )

        self.xr0 = 0
        self.yr0 = 0

    @property
    def max_episode_steps(self) -> int:
        return self._time_limit_max_episode_steps

    @property
    def goal(self) -> Point:
        goal_x = (
            self._global_path_segment.points[-1].x
            if self._global_path_segment.points
            else self.robot.xg
        )
        goal_y = (
            self._global_path_segment.points[-1].y
            if self._global_path_segment.points
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
        return not self._global_path.points

    @property
    def _global_path(self) -> GlobalGuidancePath:
        return self.robot.global_path

    @_global_path.setter
    def _global_path(self, value):
        self.robot.global_path = value

    @property
    def _global_path_segment(self) -> GlobalGuidancePath:
        return self.robot.global_path_segment

    @_global_path_segment.setter
    def _global_path_segment(self, value):
        self.robot.global_path_segment = value

    @property
    def _start_position_and_goal_generator(self) -> StartPositionAndGoalGenerator:
        return self.robot._start_position_and_goal_generator

    def _update_difficulty(self, difficulty: NavigationDifficulty):
        self._difficulty = difficulty
        self.robot._difficulty = difficulty
        self._global_path._difficulty = difficulty
        self._global_path_segment._difficulty = difficulty

    def _observe(self):
        step_normalized = 2 * self._steps / self.max_episode_steps - 1
        obs = [
            step_normalized,
            self.robot.thr,
            self._vx,
            self._vy,
            self.goal.x,
            self.goal.y,
        ]

        if self._variant == "EasierFollowing":
            obs.append(1.0 if self.is_final_goal else -1.0)

        obs.extend(
            get_n_points_evenly_spaced_on_path(
                self._global_path_segment.points,
                10,
                [self.goal.x, self.goal.y],
            )
        )

        self.robot.scanning()
        # convert LIDAR touches to ranges
        ranges = []
        for xl, yl in zip(self.robot.xls, self.robot.yls):
            ranges.append(math.dist(self.robot_position, (xl, yl)))
        obs.extend(ranges)

        return self._normalize_observation(self._relative_to_robot(np.array(obs)))

    def step(self, action):
        self._steps += 1

        self._vx = 0.5 * action[0]
        self._vy = 0.5 * action[1]
        w = action[2]
        self.prev_robot_position = Point(*self.robot_position)
        self.robot.step(self._vx, self._vy, w)

        num_passed_points = self._global_path_segment.update(
            Point(*self.robot_position)
        )

        # TODO: verify this in Gazebo sim
        driving_vector = np.array([self._vx, self._vy])
        driving_vector /= np.linalg.norm(driving_vector)
        robot_heading_vector = np.array(
            [np.cos(self.robot.thr), np.sin(self.robot.thr)]
        )
        alpha = np.arccos(np.dot(robot_heading_vector, driving_vector))

        self.robot.scanning()
        # convert LIDAR touches to ranges
        ranges = []
        for xl, yl in zip(self.robot.xls, self.robot.yls):
            ranges.append(math.dist(self.robot_position, (xl, yl)))
        min_obstacle_dist = min(ranges)

        success, reward, terminated = calculate_reward(
            self.is_final_goal,
            self.prev_robot_position,
            Point(*self.robot_position),
            alpha,
            self.goal,
            self.robot.is_crashed(),
            self._difficulty,
            num_passed_points,
            self._global_path_segment,
            self.max_episode_steps,
            self._steps,
            min_obstacle_dist,
        )

        self._metrics.episode_reward += reward

        if terminated:
            self._register_episode_terminated(success)
            if (
                self._get_current_level() < self._get_max_level()
                and self._metrics.global_success_rate > 0.7
            ):
                self._advance_to_next_level()
                self.robot.first_render = True

        if (
            self._training_mode
            and self._metrics.global_episode % self._render_freq == 0
            and self._render_mode == "human"
        ):
            self.render()

        if self._training_mode:
            self._metrics.export(self._get_current_level(), terminated)

        obs = self._observe()

        info = {}
        if success and not self.is_final_goal:
            info["status"] = "segment_reached"
        elif self._steps == self.max_episode_steps:
            info["status"] = "max_steps_reached"

        return obs, reward, terminated, False, info

    def reset(self, seed=None, options=None):  # Return to initial state
        self._vx = 0.0
        self._vy = 0.0
        if self._steps >= self.max_episode_steps:
            self._generate_next_goal = True
            if self.is_final_goal:
                logger.debug("Did not reach the final goal in time")
            elif self._difficulty.extend_segment:
                self._generate_next_goal = False
                logger.debug("Did not reach global path segment in time")
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

        self._steps = 0
        self._metrics.register_local_episode_start()

        if self._generate_next_goal:
            self._metrics.register_global_episode_start()
            self._generate_next_goal = False
            self.robot.reset(options)

            self.xr0 = self.robot.xr
            self.yr0 = self.robot.yr

        return self._observe(), {}

    def render(self, mode="human"):
        self.robot.render()

    def close(self):
        self.robot.close()
