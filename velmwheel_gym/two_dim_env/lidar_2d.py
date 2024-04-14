import numpy as np
from line_profiler import profile

from .utils import *


class Lidar2D:
    def __init__(self, robot: object, fov: int, rays: int, max_range: float):
        self._robot = robot
        self._max_range = max_range
        self._alphas = np.linspace(-np.deg2rad(fov) / 2, np.deg2rad(fov) / 2, rays)

    @profile
    def scan(self) -> list:
        xcs = self._robot.env.xcs
        ycs = self._robot.env.ycs
        rcs = self._robot.env.rcs

        r = self._max_range
        self._robot.xls = []
        self._robot.yls = []
        touches = []
        for alpha in self._alphas:
            self._robot.xls.append(self._robot.xr + r * np.cos(alpha + self._robot.thr))
            self._robot.yls.append(self._robot.yr + r * np.sin(alpha + self._robot.thr))

        for i, (xl, yl, alpha) in enumerate(
            zip(self._robot.xls, self._robot.yls, self._alphas)
        ):
            touch = False
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
                        touch = True
                        self._robot.xls[i] = result[0]
                        self._robot.yls[i] = result[1]
            touches.append(touch)

        for i, (x, y) in enumerate(zip(self._robot.xls, self._robot.yls)):
            if x >= 5 or x <= -5 or y >= 5 or y <= -5:
                touches[i] = True

        xls = [max(min(x, 5.0), -5) for x in self._robot.xls]
        self._robot.xls = np.array(xls)
        yls = [max(min(x, 5.0), -5) for x in self._robot.yls]
        self._robot.yls = np.array(yls)
        return touches
