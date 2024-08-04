from typing import Optional

import matplotlib.pyplot as plt
import numpy as np

from velmwheel_gym.types import Point


class DebugRenderer:
    def __init__(self, window_title: str):
        self._is_first_render = True
        self._fig = None
        self._ax = None
        self._window_title = window_title

    def render(
        self,
        robot: Point,
        goal: Point,
        lidar_point_cloud: Optional[np.array],
        global_path: list[Point],
        global_path_segment: list[Point],
    ):
        if lidar_point_cloud is None:
            return

        if self._is_first_render:
            self._first_render(
                robot, goal, lidar_point_cloud, global_path, global_path_segment
            )
        else:
            self._fast_render(
                robot, goal, lidar_point_cloud, global_path, global_path_segment
            )

    def close(self):
        plt.close()

    def _first_render(
        self,
        robot: Point,
        goal: Point,
        lidar_point_cloud: Optional[np.array],
        global_path: list[Point],
        global_path_segment: list[Point],
    ):
        plt.ion()
        self._fig, self._ax = plt.subplots(figsize=(10, 10))
        self._ax.set_xlim((-9, 9))
        self._ax.set_ylim((-9, 9))
        self._ax.invert_yaxis()
        self._ax.invert_xaxis()

        # Draw global guidance path
        px = [p.x for p in global_path]
        py = [p.y for p in global_path]
        (self._global_path_line,) = self._ax.plot(px, py, ".g")

        sx = [s.x for s in global_path_segment]
        sy = [s.y for s in global_path_segment]
        (self._global_path_segment_line,) = self._ax.plot(sx, sy, ".y")

        # Draw robot
        self._robot_rect = plt.Rectangle(
            (robot.x, robot.y),
            1.0,
            1.0,
            color="orange",
            fill=True,
        )
        self._ax.add_patch(self._robot_rect)

        # Draw goal
        self._robot_goal = plt.Circle(
            (goal.x, goal.y), 0.25, color="g", fill=True, zorder=10
        )
        self._ax.add_patch(self._robot_goal)

        # Draw LiDAR point cloud
        X = [point[0] + robot.x for point in lidar_point_cloud]
        Y = [point[1] + robot.y for point in lidar_point_cloud]
        (self._point_cloud_points,) = self._ax.plot(X, Y, "o", markersize=1, color="r")
        plt.pause(0.5)
        self._is_first_render = False

    def _fast_render(
        self,
        robot: Point,
        goal: Point,
        lidar_point_cloud: Optional[np.array],
        global_path: list[Point],
        global_path_segment: list[Point],
    ):
        self._robot_rect.set_xy((robot.x, robot.y))
        self._robot_goal.center = (goal.x, goal.y)
        # Update global guidance path
        px = [p.x for p in global_path]
        py = [p.y for p in global_path]
        self._global_path_line.set_data(px, py)
        sx = [s.x for s in global_path_segment]
        sy = [s.y for s in global_path_segment]
        self._global_path_segment_line.set_data(sx, sy)
        # Update LiDAR point cloud
        X = [point[0] + robot.x for point in lidar_point_cloud]
        Y = [point[1] + robot.y for point in lidar_point_cloud]
        self._point_cloud_points.set_data(X, Y)

        self._fig.canvas.draw_idle()
        self._fig.canvas.flush_events()
