""" Based on: https://github.com/EmanuelSamir/simple-2d-robot-lidar-env/blob/main/gym_robot2d/envs/robot2d_env.py """

import logging
import math

import numpy as np

from velmwheel_gym.base_env import VelmwheelBaseEnv
from velmwheel_gym.constants import NAVIGATION_DIFFICULTIES
from velmwheel_gym.global_guidance_path import (
    get_n_points_evenly_spaced_on_path,
    next_segment,
)
from velmwheel_gym.reward import calculate_reward
from velmwheel_gym.types import Point
from velmwheel_gym.utils import angle_between_robot_and_goal

from .robot2d import Robot2D

ACTION_NORMALIZATION_FACTOR = 5.0


logger = logging.getLogger(__name__)


class Robot2dEnv(VelmwheelBaseEnv):
    def __init__(
        self,
        dT=0.050,
        **kwargs,
    ):
        super().__init__(**kwargs)

        self.robot = Robot2D(dT=dT, is_render=True, difficulty=self._difficulty)
        self._generate_next_goal = True

        self.xr0 = 0
        self.yr0 = 0
        self.thr0 = 0

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

    def _observe(self):
        obs = [self.robot.thr, self.goal.x, self.goal.y]

        obs.extend(
            get_n_points_evenly_spaced_on_path(
                self.robot.global_path_segment.points,
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

        vx = 0.5 * action[0]
        vy = 0.5 * action[1]
        w = action[2]
        self.prev_robot_position = Point(*self.robot_position)
        self.robot.step(vx, vy, w)

        num_passed_points = self.robot.global_path_segment.update(
            Point(*self.robot_position)
        )

        target = (
            self.robot.global_path_segment.points[0]
            if self.robot.global_path_segment.points
            else self.goal
        )
        alpha = angle_between_robot_and_goal(
            self.robot_position, target, self.robot.thr
        )

        success, reward, terminated = calculate_reward(
            self.is_final_goal,
            self.prev_robot_position,
            Point(*self.robot_position),
            alpha,
            self.goal,
            self.robot.is_crashed(),
            self._difficulty,
            num_passed_points,
            self.robot.global_path_segment,
            self.max_episode_steps,
            self._steps,
        )

        obs = self._observe()

        self._metrics.episode_reward += reward

        if terminated:
            self._generate_next_goal = True
            if success:
                self._metrics.register_local_episode_result(1)
                if self.is_final_goal:
                    self._metrics.register_global_episode_result(1)
                    logger.debug("Successfully reached the final goal")
                    if self._training_mode:
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
            idx = NAVIGATION_DIFFICULTIES.index(self._difficulty)
            if (
                idx < len(NAVIGATION_DIFFICULTIES) - 1
                and self._metrics.global_success_rate > 0.7
            ):
                logger.info(
                    f"Global success rate ({self._mean_reward}) > 0.7. Advancing to the next level."
                )
                self._metrics.advance_to_next_level()
                self._difficulty = NAVIGATION_DIFFICULTIES[idx + 1]
                self.robot._difficulty = self._difficulty
                self.robot.global_path._difficulty = self._difficulty
                self.robot.global_path_segment._difficulty = self._difficulty
                self.robot.first_render = True

        if (
            self._training_mode
            and self._metrics.global_episode % 10 == 0
            and self._render_mode == "human"
        ):
            self.render()

        if self._training_mode:
            self._metrics.export(self._difficulty, terminated)

        return obs, reward, terminated, False, {}

    def reset(self, seed=None, options=None):  # Return to initial state
        if self._steps >= self.max_episode_steps:
            self._generate_next_goal = True
            # if self.is_final_goal:
            #     logger.debug("Did not reach the final goal in time")
            #     self._generate_next_goal = True
            # else:
            #     logger.debug("Did not reach global path segment in time")
            #     (
            #         self.robot.global_path.points,
            #         self.robot.global_path_segment,
            #     ) = next_segment(
            #         self.robot.global_path.points,
            #         self.robot.global_path_segment.points,
            #         Point(*self.robot_position),
            #         self._difficulty,
            #     )

        self._steps = 0
        self._metrics.register_local_episode_start()

        if self._generate_next_goal:
            self._metrics.register_global_episode_start()
            self._generate_next_goal = False
            self.robot.reset(options)

            self.xr0 = self.robot.xr
            self.yr0 = self.robot.yr
            self.thr0 = self.robot.thr

        return self._observe(), {}

    def render(self, mode="human"):
        self.robot.render()

    def close(self):
        self.robot.close()
