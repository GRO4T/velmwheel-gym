""" Based on: https://github.com/EmanuelSamir/simple-2d-robot-lidar/blob/main/robot2d/robot2d.py """

import copy
import logging
import os
import pickle
from typing import NamedTuple

import numpy as np
from matplotlib import pyplot as plt

from velmwheel_gym.constants import LIDAR_DATA_SIZE
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
        is_render=True,
        name="Robot2D",
        global_path_segment_length=5.0,
    ):
        self._difficulty = difficulty
        self._name = name
        # Environment
        self.env = Environment(env_min_size, env_max_size)
        self.env_min_size = env_min_size
        self.env_max_size = env_max_size
        self._global_path_segment_length = global_path_segment_length

        # Initial State
        self.xr = 0
        self.yr = 0
        self.thr = 0
        self.rr = robot_radius

        # Goal parameters
        self._start_position_and_goal_generator = StartPositionAndGoalGenerator()
        self.xg = 0
        self.yg = 0
        self.thg = 0
        self.rg = 0.1

        # Lidar parameters
        self.max_range = lidar_max_range
        self._lidar = Lidar2D(self, 360, LIDAR_DATA_SIZE, lidar_max_range)
        self.xls = []
        self.yls = []

        # Parameters for simulation
        self._time = 0.0
        self._delta = 0.001
        self.dT = dT
        self.is_render = is_render
        self.fig = None
        self.ax = None
        self.first_render = True

        self.global_path: GlobalGuidancePath = None
        self.global_path_segment: GlobalGuidancePath = None
        if os.path.exists("state/nav2_cache.pkl"):
            with open("state/nav2_cache.pkl", "rb") as f:
                self._global_path_cache = pickle.load(f)
            logger.debug("Loaded global path cache")
        else:
            self._global_path_cache: dict[tuple[Point, Point], GlobalGuidancePath] = {}

    def reset(self, options=None):
        if options and "goal" in options and "starting_position" in options:
            self._start_position_and_goal_generator._starting_position = options[
                "starting_position"
            ]
            self._start_position_and_goal_generator._goal = options["goal"]
        else:
            self._start_position_and_goal_generator.generate_next()
        self.xg = self._start_position_and_goal_generator.goal.x
        self.yg = self._start_position_and_goal_generator.goal.y
        if getattr(self, "robot_goal", None):
            self.robot_goal.center = (self.xg, self.yg)
        self.xr = self._start_position_and_goal_generator.starting_position.x
        self.yr = self._start_position_and_goal_generator.starting_position.y
        self.thr = 0.0
        self.thg = np.random.uniform(low=-np.pi, high=np.pi)

        self.xls = []
        self.yls = []

        if self._difficulty.dynamic_obstacle_count > 0:
            is_path_to_goal = False
            while not is_path_to_goal:
                self.env.get_random_obstacles(
                    self.xr,
                    self.yr,
                    self.rr,
                    self.xg,
                    self.yg,
                    self.rg,
                    self._difficulty.dynamic_obstacle_count,
                )
                o_dynamic_grid_x = copy.deepcopy(self.env.o_static_grid_x)
                o_dynamic_grid_y = copy.deepcopy(self.env.o_static_grid_y)

                for ox, oy in zip(
                    self.env.dynamic_obstacles_x, self.env.dynamic_obstacles_y
                ):
                    o_dynamic_grid_x.append(ox)
                    o_dynamic_grid_y.append(oy)
                    o_dynamic_grid_x.append(ox + 0.2)
                    o_dynamic_grid_y.append(oy)
                    o_dynamic_grid_x.append(ox - 0.2)
                    o_dynamic_grid_y.append(oy)
                    o_dynamic_grid_x.append(ox)
                    o_dynamic_grid_y.append(oy + 0.2)
                    o_dynamic_grid_x.append(ox)
                    o_dynamic_grid_y.append(oy - 0.2)

                a_star = AStarPlanner(o_dynamic_grid_x, o_dynamic_grid_y, 0.2, 0.98)
                px, py = a_star.planning(self.xr, self.yr, self.xg, self.yg)
                if len(px) > 1:
                    is_path_to_goal = True

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
                self.env.o_static_grid_x, self.env.o_static_grid_y, 0.2, 0.98
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
            with open("state/nav2_cache.pkl", "wb") as f:
                pickle.dump(self._global_path_cache, f)
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

    def is_crashed(self):
        if (self.xr > self.env_max_size - self.rr) or (
            self.xr < self.env_min_size + self.rr
        ):
            return True

        if (self.yr > self.env_max_size - self.rr) or (
            self.yr < self.env_min_size + self.rr
        ):
            return True

        for xl, yl in zip(self.xls, self.yls):
            if np.linalg.norm([xl - self.xr, yl - self.yr]) < 0.5:
                return True

        return False

    def scanning(self):
        return self._lidar.scan()

    def render(self):
        if self.is_render and self.first_render:
            plt.ion()
            self.fig, self.ax = plt.subplots(figsize=(10, 10))
            self.fig.canvas.manager.set_window_title(self._name)
            self.ax.set_xlim((-9, 9))
            self.ax.set_ylim((-9, 9))

            # Draw static walls
            self.static_walls = []
            for wall in self.env.static_walls:
                wall_patch = plt.Rectangle(
                    (wall.x, wall.y),
                    wall.width,
                    wall.height,
                    color="b",
                    fill=True,
                )
                self.ax.add_patch(wall_patch)
                self.static_walls.append(wall_patch)

            # Draw global guidance path
            px = [p.x for p in self.global_path.points]
            py = [p.y for p in self.global_path.points]
            (self.global_path_line,) = self.ax.plot(px, py, ".g")

            sx = [s.x for s in self.global_path_segment.points]
            sy = [s.y for s in self.global_path_segment.points]
            (self.global_path_segment_line,) = self.ax.plot(sx, sy, ".y")

            # Draw goal
            self.robot_goal = plt.Circle(
                (self.xg, self.yg), self.rg, color="g", fill=True, zorder=10
            )
            self.ax.add_patch(self.robot_goal)

            # Draw robot
            self.robot_circle = plt.Circle(
                (self.xr, self.yr), self.rr, color="r", fill=True, zorder=10
            )
            self.ax.add_patch(self.robot_circle)
            self.robot_triangle = plt.Polygon(self._get_robot_triangle(), zorder=20)
            self.ax.add_patch(self.robot_triangle)

            # Draw dynamic obstacles
            self.dynamic_obstacles = []
            for x, y, r in zip(
                self.env.dynamic_obstacles_x,
                self.env.dynamic_obstacles_y,
                self.env.dynamic_obstacles_radius,
            ):
                obstacle = plt.Circle((x, y), r, color="b", fill=True)
                self.ax.add_patch(obstacle)
                self.dynamic_obstacles.append(obstacle)

            # Draw lidar
            # self.lidar_scans = []
            # for xl, yl in zip(self.xls, self.yls):
            #     self.lidar_scans.append(
            #         self.ax.plot([self.xr, xl], [self.yr, yl], color="gray")
            #     )
            # self.lidar_points = self.ax.scatter(self.xls, self.yls, color="r")

            plt.pause(0.5)
            self.first_render = False

        if self.is_render:
            # Update robot position
            self.robot_circle.center = (self.xr, self.yr)
            self.robot_triangle.set_xy(self._get_robot_triangle())

            # Update dynamic obstacles
            for obstacle, (x, y) in zip(
                self.dynamic_obstacles,
                zip(self.env.dynamic_obstacles_x, self.env.dynamic_obstacles_y),
            ):
                obstacle.center = (x, y)

            # Update global guidance path
            px = [p.x for p in self.global_path.points]
            py = [p.y for p in self.global_path.points]
            self.global_path_line.set_data(px, py)

            sx = [s.x for s in self.global_path_segment.points]
            sy = [s.y for s in self.global_path_segment.points]
            self.global_path_segment_line.set_data(sx, sy)

            # Update lidar
            # for scan, (xl, yl) in zip(self.lidar_scans, zip(self.xls, self.yls)):
            #     scan[0].set_data([self.xr, xl], [self.yr, yl])
            # self.lidar_points.set_offsets(np.c_[self.xls, self.yls])

            self.fig.canvas.draw_idle()
            self.fig.canvas.flush_events()

    def _get_robot_triangle(self):
        return [
            [
                self.rr * np.cos(self.thr + np.deg2rad(140)) + self.xr,
                self.rr * np.sin(self.thr + np.deg2rad(140)) + self.yr,
            ],
            [
                self.rr * np.cos(self.thr) + self.xr,
                self.rr * np.sin(self.thr) + self.yr,
            ],
            [
                self.rr * np.cos(self.thr - np.deg2rad(140)) + self.xr,
                self.rr * np.sin(self.thr - np.deg2rad(140)) + self.yr,
            ],
        ]

    def close(self):
        plt.ioff()
        plt.close()
        self.first_render = True


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
    ):
        self.env_min_size = env_min_size
        self.env_max_size = env_max_size

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

        # for _ in range(n):
        #     px, py = self._random_point_without_robot_and_goal(
        #         xr, yr, rr, xg, yg, rg, r
        #     )
        #     xcs.append(px)
        #     ycs.append(py)
        #     px, py = self._random_point_without_robot_and_goal(
        #         xr, yr, rr, xg, yg, rg, r
        #     )
        #     gcs.append((px, py))
        from velmwheel_gym.constants import OBSTACLES_EASY

        if n == 5:
            xcs = [p[0] for p in OBSTACLES_EASY]
            ycs = [p[1] for p in OBSTACLES_EASY]

        self.dynamic_obstacles_orig_x = np.array(xcs)
        self.dynamic_obstacles_orig_y = np.array(ycs)
        self.dynamic_obstacles_x = np.array(xcs)
        self.dynamic_obstacles_y = np.array(ycs)
        self.dynamic_obstacles_radius = np.array(rcs)
        self.dynamic_obstacles_goals = np.array(gcs)
