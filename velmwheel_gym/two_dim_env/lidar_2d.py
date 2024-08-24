import numpy as np
from FastLine import Line

from .utils import *


class Lidar2D:
    def __init__(self, robot: object, fov: int, rays: int, max_range: float):
        self._robot = robot
        self._max_range = max_range
        self._alphas = np.linspace(-np.deg2rad(fov) / 2, np.deg2rad(fov) / 2, rays)

    def scan(self):
        xcs = self._robot.env.dynamic_obstacles_x
        ycs = self._robot.env.dynamic_obstacles_y
        rcs = self._robot.env.dynamic_obstacles_radius
        r = self._max_range
        num_rays = len(self._alphas)

        # Preallocate arrays
        self._robot.xls = np.zeros(num_rays)
        self._robot.yls = np.zeros(num_rays)

        # Vectorized calculation of ray end points
        cos_alphas = np.cos(self._alphas + self._robot.thr)
        sin_alphas = np.sin(self._alphas + self._robot.thr)
        self._robot.xls = self._robot.xr + r * cos_alphas
        self._robot.yls = self._robot.yr + r * sin_alphas

        wall_lines = []
        for i, wall in enumerate(self._robot.env.static_walls):
            wall_lines.append(
                Line(
                    p1=(wall.x, wall.y),
                    p2=(wall.x + wall.width, wall.y + wall.height),
                )
            )

        for i, (xl, yl, alpha) in enumerate(
            zip(self._robot.xls, self._robot.yls, self._alphas)
        ):
            l_ray = Line(p1=(self._robot.xr, self._robot.yr), p2=(xl, yl))

            # Check if the lidar rays touch any static obstacle
            for j, wall in enumerate(self._robot.env.static_walls):
                result = wall_lines[j].intersection(l_ray)
                if result:
                    if (
                        result[0] < wall.x
                        or result[0] > wall.x + wall.width
                        or result[1] < wall.y
                        or result[1] > wall.y + wall.height
                    ):
                        continue
                    cond = validate_point(
                        result[0] - self._robot.xr,
                        result[1] - self._robot.yr,
                        self._robot.xls[i] - self._robot.xr,
                        self._robot.yls[i] - self._robot.yr,
                        self._robot.thr + alpha,
                        self._max_range,
                    )
                    if cond:
                        self._robot.xls[i] = result[0]
                        self._robot.yls[i] = result[1]

            # Check if the lidar rays touch any dynamic obstacle
            for xc, yc, rc in zip(xcs, ycs, rcs):
                is_inter, result = obtain_intersection_points(
                    self._robot.xr, self._robot.yr, xl, yl, xc, yc, rc
                )
                if is_inter:
                    cond = validate_point(
                        result[0] - self._robot.xr,
                        result[1] - self._robot.yr,
                        self._robot.xls[i] - self._robot.xr,
                        self._robot.yls[i] - self._robot.yr,
                        self._robot.thr + alpha,
                        self._max_range,
                    )
                    if cond:
                        self._robot.xls[i] = result[0]
                        self._robot.yls[i] = result[1]

    def _clamp_to_environment_boundaries(self, x, y):
        top_bound = ((9, 9), (-9, 9))
        bottom_bound = ((-9, -9), (9, -9))
        left_bound = ((-9, 9), (-9, -9))
        right_bound = ((9, -9), (9, 9))

        # TODO: take closer intersection point
        if x > 9:
            if x > abs(y):
                return line_intersection(
                    right_bound, ((self._robot.xr, self._robot.yr), (x, y))
                )
            if y > 9:
                return line_intersection(
                    top_bound, ((self._robot.xr, self._robot.yr), (x, y))
                )
            return line_intersection(
                bottom_bound, ((self._robot.xr, self._robot.yr), (x, y))
            )
        if x < -9:
            if abs(x) > abs(y):
                return line_intersection(
                    left_bound, ((self._robot.xr, self._robot.yr), (x, y))
                )
            if y > 9:
                return line_intersection(
                    top_bound, ((self._robot.xr, self._robot.yr), (x, y))
                )
            return line_intersection(
                bottom_bound, ((self._robot.xr, self._robot.yr), (x, y))
            )
        if y > 9:
            return line_intersection(
                top_bound, ((self._robot.xr, self._robot.yr), (x, y))
            )
        if y < -9:
            return line_intersection(
                bottom_bound, ((self._robot.xr, self._robot.yr), (x, y))
            )
        return x, y
