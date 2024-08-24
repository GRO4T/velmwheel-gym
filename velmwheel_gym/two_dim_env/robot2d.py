""" Based on: https://github.com/EmanuelSamir/simple-2d-robot-lidar/blob/main/robot2d/robot2d.py """

import copy
import logging
import os
import pickle
from typing import NamedTuple

import numpy as np

from velmwheel_gym.gazebo_env.start_position_and_goal_generator import (
    StartPositionAndGoalGenerator,
)
from velmwheel_gym.global_guidance_path import GlobalGuidancePath, next_segment
from velmwheel_gym.two_dim_env.lidar_2d import Lidar2D
from velmwheel_gym.two_dim_env.planner import AStarPlanner
from velmwheel_gym.types import NavigationDifficulty, Point
from velmwheel_gym.utils import interpolate_coordinates

from .utils import *

logger = logging.getLogger(__name__)


class Robot2D:
    def __init__(
        self,
        difficulty: NavigationDifficulty,
        env_max_size=9,
        env_min_size=-9,
        robot_radius=0.5,
        lidar_max_range=20.0,
        dT=0.01,
        global_path_segment_length=5.0,
    ):
        self._difficulty = difficulty
        # Environment
        self.env = Environment(env_min_size, env_max_size, self._difficulty)
        self.env_min_size = env_min_size
        self.env_max_size = env_max_size
        self._global_path_segment_length = global_path_segment_length

        # Initial State
        self.xr = 0
        self.yr = 0
        self.thr = 0
        self.rr = robot_radius

        # Goal parameters
        self._start_position_and_goal_generator = StartPositionAndGoalGenerator(
            self._difficulty
        )
        self.xg = 0
        self.yg = 0
        self.thg = 0
        self.rg = 0.1

        # Lidar parameters
        self.max_range = lidar_max_range
        self._lidar = Lidar2D(
            self, 360, self._difficulty.raw_lidar_ray_count, lidar_max_range
        )
        self.xls = []
        self.yls = []

        # Parameters for simulation
        self._time = 0.0
        self._delta = 0.001
        self.dT = dT

        self.global_path: GlobalGuidancePath = None
        self.global_path_segment: GlobalGuidancePath = None
        if os.path.exists("state/nav2_cache.pkl"):
            # with open("state/nav2_cache.pkl", "rb") as f:
            #     self._global_path_cache = pickle.load(f)
            # logger.debug("Loaded global path cache")
            self._global_path_cache: dict[tuple[Point, Point], GlobalGuidancePath] = {}
        else:
            self._global_path_cache: dict[tuple[Point, Point], GlobalGuidancePath] = {}

    def reset(self, options=None):
        if self._difficulty.dynamic_obstacles:
            self.env.get_random_obstacles(
                self.xr,
                self.yr,
                self.rr,
                self.xg,
                self.yg,
                self.rg,
                len(self._difficulty.dynamic_obstacles),
            )

        if options and "goal" in options and "starting_position" in options:
            self._start_position_and_goal_generator._starting_position = options[
                "starting_position"
            ]
            self._start_position_and_goal_generator._goal = options["goal"]
        else:
            while True:
                self._start_position_and_goal_generator.generate_next()
                self.xr = self._start_position_and_goal_generator.goal.x
                self.yr = self._start_position_and_goal_generator.goal.y
                self.scanning()
                if self.is_crashed(threshold=0.6):
                    continue
                self.xr = self._start_position_and_goal_generator.starting_position.x
                self.yr = self._start_position_and_goal_generator.starting_position.y
                self.thr = self._start_position_and_goal_generator.starting_rotation
                self.scanning()
                if not self.is_crashed(threshold=0.6):
                    break

        self.xg = self._start_position_and_goal_generator.goal.x
        self.yg = self._start_position_and_goal_generator.goal.y
        if getattr(self, "robot_goal", None):
            self.robot_goal.center = (self.xg, self.yg)
        self.thg = np.random.uniform(low=-np.pi, high=np.pi)

        self.xls = []
        self.yls = []

        if (
            Point(self.xr, self.yr),
            Point(self.xg, self.yg),
        ) in self._global_path_cache:
            logger.debug("Using cached global path")
            self.global_path = copy.deepcopy(
                self._global_path_cache[
                    (Point(self.xr, self.yr), Point(self.xg, self.yg))
                ]
            )
        else:
            logger.debug("Calculating new global path")
            a_star = AStarPlanner(
                self.env.o_static_grid_x, self.env.o_static_grid_y, 0.2, 0.5
            )
            px, py = a_star.planning(self.xr, self.yr, self.xg, self.yg)

            points = [Point(x, y) for x, y in zip(px, py)]
            points = points[
                ::-1
            ]  # This implementation of A* returns the path in reverse order
            self.global_path = GlobalGuidancePath(
                Point(self.xr, self.yr), points, self._difficulty
            )
            self._global_path_cache[
                (Point(self.xr, self.yr), Point(self.xg, self.yg))
            ] = copy.deepcopy(self.global_path)
            # with open("state/nav2_cache.pkl", "wb") as f:
            #     pickle.dump(self._global_path_cache, f)
        self.global_path.points, self.global_path_segment = next_segment(
            self.global_path.points,
            [],
            Point(self.xr, self.yr),
            self._difficulty,
            self._global_path_segment_length,
        )

    def step(self, vx, vy, w=0):
        self.xr = self.xr + self.dT * (np.cos(self.thr) * vx - np.sin(self.thr) * vy)
        self.yr = self.yr + self.dT * (np.sin(self.thr) * vx + np.cos(self.thr) * vy)
        self.thr = self.thr + self.dT * w
        self.thr = clip_angle(self.thr)

        if self._time > 1.0 or self._time < 0.0:
            self._delta = -self._delta
        self._time += self._delta
        if self._difficulty.dynamic_obstacle_motion:
            for i in range(len(self.env.dynamic_obstacles_x)):
                x, y = (
                    self.env.dynamic_obstacles_orig_x[i],
                    self.env.dynamic_obstacles_orig_y[i],
                )
                tx, ty = self.env.dynamic_obstacles_goals[i]
                new_x, new_y = interpolate_coordinates(x, y, tx, ty, self._time)
                self.env.dynamic_obstacles_x[i] = new_x
                self.env.dynamic_obstacles_y[i] = new_y

    def is_crashed(self, threshold=0.5):
        for xl, yl in zip(self.xls, self.yls):
            if np.linalg.norm([xl - self.xr, yl - self.yr]) < threshold:
                return True

        return False

    def scanning(self):
        self._lidar.scan()


class Environment:
    class Wall(NamedTuple):
        x: float
        y: float
        width: float
        height: float

    def __init__(
        self,
        env_min_size,
        env_max_size,
        difficulty: NavigationDifficulty,
    ):
        self.env_min_size = env_min_size
        self.env_max_size = env_max_size
        self._difficulty = difficulty

        self.static_walls: list[Environment.Wall] = [
            Environment.Wall(-6, -6, 8, 0.2),
            Environment.Wall(4, -6, 5, 0.2),
            Environment.Wall(-6, -6, 0.2, 4),
            Environment.Wall(-6, 0, 0.2, 6),
            Environment.Wall(-6, 6, 12, 0.2),
            Environment.Wall(6, -6, 0.2, 12),
            Environment.Wall(-9, -9, 18, 0.2),
            Environment.Wall(-9, -9, 0.2, 18),
            Environment.Wall(-9, 8.8, 18, 0.2),
            Environment.Wall(8.8, -9, 0.2, 18),
        ]

        self.o_static_grid_x = []
        self.o_static_grid_y = []

        for wall in self.static_walls:
            if wall.width > wall.height:
                x = wall.x
                while x < wall.x + wall.width:
                    self.o_static_grid_x.append(x)
                    self.o_static_grid_y.append(wall.y)
                    x += 0.2
            else:
                y = wall.y
                while y < wall.y + wall.height:
                    self.o_static_grid_x.append(wall.x)
                    self.o_static_grid_y.append(y)
                    y += 0.2

        self.dynamic_obstacles_orig_x = np.array([])
        self.dynamic_obstacles_orig_y = np.array([])
        self.dynamic_obstacles_x = np.array([])
        self.dynamic_obstacles_y = np.array([])
        self.dynamic_obstacles_goals = np.array([])
        self.dynamic_obstacles_radius = np.array([])

    def _random_point_without_robot_and_goal(self, pxr, pyr, rr, pxg, pyg, rg, r):
        cond = False
        while not cond:
            px = np.random.uniform(
                low=self.env_min_size + rr, high=self.env_max_size - rr
            )
            py = np.random.uniform(
                low=self.env_min_size + rr, high=self.env_max_size - rr
            )
            if (
                ((px < pxr + r + rr) and (px > pxr - r - rr))
                and ((py < pyr + r + rr) and (py > pyr - r - rr))
            ) or (
                ((px < pxg + r + rg) and (px > pxg - r - rg))
                and ((py < pyg + r + rg) and (py > pyg - r - rg))
            ):
                pass
            else:
                cond = True
        return px, py

    def get_random_obstacles(self, xr, yr, rr, xg=0, yg=0, rg=0, n=0, r=0.3):
        xcs = []
        ycs = []
        gcs = []
        rcs = n * [r]

        if n > 0:
            xcs = [p[0] for p in self._difficulty.dynamic_obstacles]
            ycs = [p[1] for p in self._difficulty.dynamic_obstacles]

        self.dynamic_obstacles_orig_x = np.array(xcs)
        self.dynamic_obstacles_orig_y = np.array(ycs)
        self.dynamic_obstacles_x = np.array(xcs)
        self.dynamic_obstacles_y = np.array(ycs)
        self.dynamic_obstacles_radius = np.array(rcs)
        self.dynamic_obstacles_goals = np.array(gcs)
