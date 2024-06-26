""" Based on: https://github.com/EmanuelSamir/simple-2d-robot-lidar/blob/main/robot2d/robot2d.py """

import numpy as np
from matplotlib import pyplot as plt

from velmwheel_gym.constants import LIDAR_DATA_SIZE
from velmwheel_gym.global_guidance_path import GlobalGuidancePath
from velmwheel_gym.two_dim_env.lidar_2d import Lidar2D
from velmwheel_gym.types import NavigationDifficulty, Point

from .utils import *


class Robot2D:
    def __init__(
        self,
        difficulty: NavigationDifficulty,
        env_max_size=5,
        env_min_size=-5,
        robot_radius=0.4,
        lidar_max_range=20.0,
        dT=0.01,
        is_render=True,
        is_goal=True,
    ):
        self._difficulty = difficulty
        # Environment
        self.env = Environment(env_min_size, env_max_size)
        self.env_min_size = env_min_size
        self.env_max_size = env_max_size

        # Initial State
        self.xr = 0
        self.yr = 0
        self.thr = 0
        self.rr = robot_radius

        # Goal parameters
        self.is_goal = is_goal
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
        self.dT = dT
        self.is_render = is_render
        self.fig = None
        self.ax = None
        self.first_render = True

        self._global_guidance_path: GlobalGuidancePath = None

    def reset(self):
        self.xr = np.random.uniform(
            low=self.env_min_size + self.rr, high=self.env_max_size - self.rr
        )
        self.yr = np.random.uniform(
            low=self.env_min_size + self.rr, high=self.env_max_size - self.rr
        )

        if self.is_goal:
            self.xg, self.yg = self.env._random_point_without_robot(
                self.xr, self.yr, self.rr, self.rg
            )
            self.thg = np.random.uniform(low=-np.pi, high=np.pi)

        self.env.get_random_obstacles(
            self.xr,
            self.yr,
            self.rr,
            self.is_goal,
            self.xg,
            self.yg,
            self.rg,
            self._difficulty.dynamic_obstacle_count,
        )
        self.xls = []
        self.yls = []

        dx = self.xg - self.xr
        dy = self.yg - self.yr
        dx_step = dx / 160
        dy_step = dy / 160
        px = []
        py = []
        for i in range(160):
            px.append(self.xr + i * dx_step)
            py.append(self.yr + i * dy_step)

        points = [Point(x, y) for x, y in zip(px, py)]
        self._global_guidance_path = GlobalGuidancePath(
            Point(self.xr, self.yr), points, self._difficulty
        )

    def set_init_state(self, x0, y0, th0=0):
        self.xr = x0
        self.yr = y0
        self.thr = clip_angle(th0)

        if (self.xr > self.env_max_size - self.rr) or (
            self.xr < self.env_min_size + self.rr
        ):
            raise ValueError(
                "x value: {} is out of range {} and {}".format(
                    self.xr, self.env_min_size, self.env_max_size
                )
            )

        if (self.yr > self.env_max_size - self.rr) or (
            self.yr < self.env_min_size + self.rr
        ):
            raise ValueError(
                "y value: {} is out of range {} and {}".format(
                    self.yr, self.env_min_size, self.env_max_size
                )
            )

        if self.is_goal:
            self.xg, self.yg = self.env._random_point_without_robot(
                self.xr, self.yr, self.rr, self.rg
            )
            self.thg = np.random.uniform(low=-np.pi, high=np.pi)

        self.env.get_random_obstacles(
            self.xr,
            self.yr,
            self.rr,
            self.is_goal,
            self.xg,
            self.yg,
            self.rg,
            self._difficulty.dynamic_obstacle_count,
        )
        self.xls = []
        self.yls = []

    def set_random_goal(self):
        if self.is_goal:
            self.xg, self.yg = self.env._random_point_without_obstacles_and_robot(
                self.xr, self.yr, self.rr, self.rg
            )
            self.thg = np.random.uniform(low=-np.pi, high=np.pi)

    def step(self, vx, vy, w=0):
        self.xr = self.xr + self.dT * (np.cos(self.thr) * vx - np.sin(self.thr) * vy)
        self.yr = self.yr + self.dT * (np.sin(self.thr) * vx + np.cos(self.thr) * vy)
        self.thr = self.thr + self.dT * w
        self.thr = clip_angle(self.thr)

    def is_crashed(self):
        xcs = self.env.xcs
        ycs = self.env.ycs
        rcs = self.env.rcs

        if (self.xr > self.env_max_size - self.rr) or (
            self.xr < self.env_min_size + self.rr
        ):
            return True

        if (self.yr > self.env_max_size - self.rr) or (
            self.yr < self.env_min_size + self.rr
        ):
            return True

        for xc, yc, rc in zip(xcs, ycs, rcs):
            if (np.sqrt((xc - self.xr) ** 2 + (yc - self.yr) ** 2)) < rc + self.rr:
                return True
        return False

    def scanning(self):
        return self._lidar.scan()

    def render(self):
        # If render enabled,
        if self.is_render and self.first_render:
            plt.ion()
            self.fig, self.ax = plt.subplots(figsize=(10, 10))
            self.ax.set_xlim((-5, 5))
            self.ax.set_ylim((-5, 5))
            circle = plt.Circle((self.xr, self.yr), self.rr, color="r", fill=True)
            self.ax.add_patch(circle)
            plt.pause(0.5)
            self.first_render = False

        if self.is_render:
            xcs = self.env.xcs
            ycs = self.env.ycs
            rcs = self.env.rcs
            self.ax.clear()
            self.ax.set_xlim((-5, 5))
            self.ax.set_ylim((-5, 5))

            circle = plt.Circle(
                (self.xr, self.yr), self.rr, color="r", fill=True, zorder=10
            )
            self.ax.add_patch(circle)

            points = [
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
            triag = plt.Polygon(points, zorder=20)
            self.ax.add_patch(triag)

            if self.is_goal:
                circle = plt.Circle(
                    (self.xg, self.yg), self.rg, color="g", fill=True, zorder=10
                )
                self.ax.add_patch(circle)

            self.ax.scatter(self.xls, self.yls, color="r")
            for xc, yc, rc in zip(xcs, ycs, rcs):
                circle = plt.Circle((xc, yc), rc, color="b", fill=True)
                self.ax.add_patch(circle)
            for xl, yl in zip(self.xls, self.yls):
                self.ax.plot([self.xr, xl], [self.yr, yl], color="gray")

            px = [p.x for p in self._global_guidance_path.points]
            py = [p.y for p in self._global_guidance_path.points]
            plt.plot(px, py, ".g")

            plt.pause(0.02)
            self.fig.canvas.draw()

    def close(self):
        plt.ioff()
        plt.close()
        self.first_render = True


class Environment:
    def __init__(
        self,
        env_min_size,
        env_max_size,
        xcs=np.array([]),
        ycs=np.array([]),
        rcs=np.array([]),
    ):
        # env param
        self.env_min_size = env_min_size
        self.env_max_size = env_max_size

        # obstacles 3- x, y, radius
        self.xcs = xcs
        self.ycs = ycs
        self.rcs = rcs

    def _random_point_without_robot(self, pxr, pyr, rr, r):
        cond = False
        while not cond:
            px = np.random.uniform(
                low=self.env_min_size + rr, high=self.env_max_size - rr
            )
            py = np.random.uniform(
                low=self.env_min_size + rr, high=self.env_max_size - rr
            )
            if ((px < pxr + r + rr) and (px > pxr - r - rr)) and (
                (py < pyr + r + rr) and (py > pyr - r - rr)
            ):
                pass
            else:
                cond = True
        return px, py

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

    def _random_point_without_obstacles_and_robot(self, pxr, pyr, rr, r):
        if self.xcs.size == 0:
            print("No obstacles were found. Please load obstacles first.")
            return -1

        cond = False
        while not cond:
            cond = True
            px = np.random.uniform(
                low=self.env_min_size + r, high=self.env_max_size - r
            )
            py = np.random.uniform(
                low=self.env_min_size + r, high=self.env_max_size - r
            )

            if (np.linalg.norm([px - pxr, py - pyr])) < 4:
                cond = False
                continue

            for xc, yc, rc in zip(self.xcs, self.ycs, self.rcs):
                if (np.linalg.norm([px - xc, py - yc])) < rc + r:
                    cond = False
                    break

        return px, py

    def get_random_obstacles(
        self, xr, yr, rr, is_goal=False, xg=0, yg=0, rg=0, n=0, r=0.3
    ):
        xcs = []
        ycs = []
        rcs = n * [r]

        if is_goal:
            for _ in range(n):
                px, py = self._random_point_without_robot_and_goal(
                    xr, yr, rr, xg, yg, rg, r
                )
                xcs.append(px)
                ycs.append(py)
        else:
            for _ in range(n):
                px, py = self._random_point_without_robot(xr, yr, rr, r)
                xcs.append(px)
                ycs.append(py)

        self.xcs = np.array(xcs)
        self.ycs = np.array(ycs)
        self.rcs = np.array(rcs)
