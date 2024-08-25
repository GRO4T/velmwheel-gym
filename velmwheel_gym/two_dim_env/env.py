""" Based on: https://github.com/EmanuelSamir/simple-2d-robot-lidar-env/blob/main/gym_robot2d/envs/robot2d_env.py """

import logging
import math

import numpy as np

from velmwheel_gym.base_env import VelmwheelBaseEnv
from velmwheel_gym.constants import (
    BASE_STEP_TIME,
    MAX_LINEAR_VELOCITY,
    TARGET_LIDAR_RAY_COUNT,
)
from velmwheel_gym.gazebo_env.start_position_and_goal_generator import (
    StartPositionAndGoalGenerator,
)
from velmwheel_gym.global_guidance_path import (
    GlobalGuidancePath,
    get_n_points_evenly_spaced_on_path,
    next_segment,
)
from velmwheel_gym.reward import calculate_reward
from velmwheel_gym.two_dim_env.renderer import Env2DRenderer
from velmwheel_gym.types import NavigationDifficulty, Point
from velmwheel_gym.utils import angle_between_robot_and_goal

from .robot2d import Robot2D

logger = logging.getLogger(__name__)


class Velmwheel2DEnv(VelmwheelBaseEnv):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        self.robot = Robot2D(
            dT=BASE_STEP_TIME,
            difficulty=self._difficulty,
            global_path_segment_length=self._global_path_segment_length,
        )

        self.xr0 = 0
        self.yr0 = 0
        self._renderer = Env2DRenderer(
            window_title=f"{self._env_name}_{self._variant}", display_lidar=False
        )

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
        self._start_position_and_goal_generator._difficulty = difficulty
        self.robot.env._difficulty = difficulty
        from velmwheel_gym.two_dim_env.lidar_2d import Lidar2D

        self.robot._lidar = Lidar2D(
            self.robot, 360, self._difficulty.raw_lidar_ray_count, self.robot.max_range
        )

    def _observe(self):
        if "CNN" in self._variant:
            step_normalized = 2 * self._steps / self.max_episode_steps - 1
            final_goal_flag = 1.0 if self.is_final_goal else -1.0
            # map lidar points to image
            lidar_map = np.zeros((80, 80), dtype=np.uint8)
            for xl, yl in zip(self.robot.xls, self.robot.yls):
                xl_rel = xl - self.robot.xr
                yl_rel = yl - self.robot.yr
                if (
                    xl_rel <= 4.0
                    and xl_rel >= -4.0
                    and yl_rel <= 4.0
                    and yl_rel >= -4.0
                ):
                    xl_pixel = int((xl_rel + 4.0) * 10)
                    yl_pixel = 79 - int((yl_rel + 4.0) * 10)
                    lidar_map[yl_pixel, xl_pixel] = 255
            # save lidar map as bitmap
            from PIL import Image

            im = Image.fromarray(lidar_map, mode="L").convert("1")
            im.save("lidar_map.png")
            obs = {
                "scalar": np.array(
                    [
                        step_normalized,
                        self.robot.thr,
                        self.goal.x,
                        self.goal.y,
                        final_goal_flag,
                    ]
                ),
                "image": lidar_map,
            }
            return obs

        step_normalized = 2 * self._steps / self.max_episode_steps - 1

        obs = [
            step_normalized,
            self.robot.thr,
            self.goal.x,
            self.goal.y,
        ]

        if "EasierFollowing" in self._variant:
            obs.append(1.0 if self.is_final_goal else -1.0)

        if "NoGlobalGuidance" not in self._variant:
            obs.extend(
                get_n_points_evenly_spaced_on_path(
                    self._global_path_segment.points,
                    10,
                    [self.goal.x, self.goal.y],
                )
            )

        obs.extend(self._lidar_ranges)

        return self._normalize_observation(self._relative_to_robot(np.array(obs)))

    def step(self, action):
        self._total_steps += 1
        self._steps += 1

        t = min(1.0, self._total_steps / 5e3)
        target_vel = 0.5 * (1.0 - t) + MAX_LINEAR_VELOCITY * t
        target_rot = 1.0 * (1.0 - t) + 1.0 * t
        self._vx = target_vel * action[0]
        self._vy = target_vel * action[1]
        w = target_rot * action[2]
        self.prev_robot_position = Point(*self.robot_position)
        self.robot.step(self._vx, self._vy, w)

        num_passed_points = self._global_path_segment.update(
            Point(*self.robot_position)
        )

        alpha = angle_between_robot_and_goal(
            self.robot_position, self.goal, self.robot.thr
        )

        self._lidar_ranges = self._calculate_lidar_ranges()
        min_obstacle_dist = min(self._lidar_ranges)

        success, reward, terminated = calculate_reward(
            self._variant,
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
            w,
        )

        self._metrics.episode_reward += reward

        if terminated:
            self._register_episode_terminated(success)
            if (
                self._get_current_level() < self._get_max_level()
                and self._metrics.global_success_rate > 0.7
            ):
                self._advance_to_next_level()
                self._renderer.reset()
                self._lidar_ranges = self._calculate_lidar_ranges()

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

        self._lidar_ranges = self._calculate_lidar_ranges()

        return self._observe(), {}

    def render(self, mode="human"):
        self._renderer.render(
            Point(*self.robot_position),
            self.robot.thr,
            Point(self.robot.xg, self.robot.yg),
            self.robot.xls,
            self.robot.yls,
            self._global_path.points,
            self._global_path_segment.points,
            self.robot.env.static_walls,
            self.robot.env.dynamic_obstacles_x,
            self.robot.env.dynamic_obstacles_y,
        )

    def close(self):
        self.robot.close()
        self._renderer.close()

    def _calculate_lidar_ranges(self):
        self.robot.scanning()
        # convert LIDAR touches to ranges
        ranges = []
        for xl, yl in zip(self.robot.xls, self.robot.yls):
            ranges.append(math.dist(self.robot_position, (xl, yl)))
        factor = self._difficulty.raw_lidar_ray_count // TARGET_LIDAR_RAY_COUNT
        if factor > 1:
            return [min(ranges[i : i + factor]) for i in range(0, len(ranges), factor)]
        return ranges
