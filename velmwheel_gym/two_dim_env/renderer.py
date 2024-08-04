import matplotlib.pyplot as plt
import numpy as np

from velmwheel_gym.two_dim_env.robot2d import Environment
from velmwheel_gym.types import Point


class Env2DRenderer:
    def __init__(self, window_title: str, display_lidar: bool = False):
        self._is_first_render = True
        self._fig = None
        self._ax = None
        self._window_title = window_title
        self._display_lidar = display_lidar

    def reset(self):
        self._is_first_render = True

    def close(self):
        plt.ioff()
        plt.close()

    def render(
        self,
        robot_position: Point,
        robot_orientation: float,
        goal: Point,
        xls: np.array,
        yls: np.array,
        global_path: list[Point],
        global_path_segment: list[Point],
        static_walls: list[Environment.Wall],
        dynamic_obstacles_x: np.array,
        dynamic_obstacles_y: np.array,
    ):
        if self._is_first_render:
            self._first_render(
                robot_position,
                robot_orientation,
                goal,
                xls,
                yls,
                global_path,
                global_path_segment,
                static_walls,
                dynamic_obstacles_x,
                dynamic_obstacles_y,
            )
        else:
            self._fast_render(
                robot_position,
                robot_orientation,
                goal,
                xls,
                yls,
                global_path,
                global_path_segment,
                dynamic_obstacles_x,
                dynamic_obstacles_y,
            )

    def _first_render(
        self,
        robot_position: Point,
        robot_orientation: float,
        goal: Point,
        xls: np.array,
        yls: np.array,
        global_path: list[Point],
        global_path_segment: list[Point],
        static_walls: list[Environment.Wall],
        dynamic_obstacles_x: np.array,
        dynamic_obstacles_y: np.array,
    ):
        plt.ion()
        self._fig, self._ax = plt.subplots(figsize=(10, 10))
        self._fig.canvas.manager.set_window_title(self._window_title)
        self._ax.set_xlim((-9, 9))
        self._ax.set_ylim((-9, 9))

        # Draw static walls
        self._static_wall_patches = []
        for wall in static_walls:
            wall_patch = plt.Rectangle(
                (wall.x, wall.y),
                wall.width,
                wall.height,
                color="b",
                fill=True,
            )
            self._ax.add_patch(wall_patch)
            self._static_wall_patches.append(wall_patch)

        # TODO: use 'zip'
        # Draw global guidance path
        px = [p.x for p in global_path]
        py = [p.y for p in global_path]
        (self._global_path_line,) = self._ax.plot(px, py, ".g")

        sx = [s.x for s in global_path_segment]
        sy = [s.y for s in global_path_segment]
        (self._global_path_segment_line,) = self._ax.plot(sx, sy, ".y")

        # Draw goal
        self._robot_goal = plt.Circle(
            (goal.x, goal.y), 0.1, color="g", fill=True, zorder=10
        )
        self._ax.add_patch(self._robot_goal)

        # Draw robot
        self._robot_circle = plt.Circle(
            (robot_position.x, robot_position.y),
            0.5,
            color="orange",
            fill=True,
            zorder=10,
        )
        self._ax.add_patch(self._robot_circle)
        self._robot_triangle = plt.Polygon(
            self._get_robot_triangle(robot_position, robot_orientation), zorder=20
        )
        self._ax.add_patch(self._robot_triangle)

        # Draw dynamic obstacles
        self._dynamic_obstacle_patches = []
        for x, y in zip(
            dynamic_obstacles_x,
            dynamic_obstacles_y,
        ):
            obstacle = plt.Circle((x, y), 0.25, color="b", fill=True)
            self._ax.add_patch(obstacle)
            self._dynamic_obstacle_patches.append(obstacle)

        # Draw lidar
        if self._display_lidar:
            self._lidar_scans = []
            for xl, yl in zip(xls, yls):
                self._lidar_scans.append(
                    self._ax.plot(
                        [robot_position.x, xl], [robot_position.y, yl], color="gray"
                    )
                )
            self._lidar_points = self._ax.scatter(xls, yls, color="r")

        plt.pause(0.5)
        self._is_first_render = False

    def _fast_render(
        self,
        robot_position: Point,
        robot_orientation: float,
        goal: Point,
        xls: np.array,
        yls: np.array,
        global_path: list[Point],
        global_path_segment: list[Point],
        dynamic_obstacles_x: np.array,
        dynamic_obstacles_y: np.array,
    ):
        # Update robot position
        self._robot_circle.center = (robot_position.x, robot_position.y)
        self._robot_triangle.set_xy(
            self._get_robot_triangle(robot_position, robot_orientation)
        )

        # Update goal position
        self._robot_goal.center = (goal.x, goal.y)

        # Update dynamic obstacles
        for obstacle, (x, y) in zip(
            self._dynamic_obstacle_patches,
            zip(dynamic_obstacles_x, dynamic_obstacles_y),
        ):
            obstacle.center = (x, y)

        # Update global guidance path
        px = [p.x for p in global_path]
        py = [p.y for p in global_path]
        self._global_path_line.set_data(px, py)

        sx = [s.x for s in global_path_segment]
        sy = [s.y for s in global_path_segment]
        self._global_path_segment_line.set_data(sx, sy)

        # Update lidar
        if self._display_lidar:
            for scan, (xl, yl) in zip(self._lidar_scans, zip(xls, yls)):
                scan[0].set_data([robot_position.x, xl], [robot_position.y, yl])
            self._lidar_points.set_offsets(np.c_[xls, yls])

        self._fig.canvas.draw_idle()
        self._fig.canvas.flush_events()

    def _get_robot_triangle(self, robot_position: Point, robot_orientation: float):
        robot_radius = 0.5
        return [
            [
                robot_radius * np.cos(robot_orientation + np.deg2rad(140))
                + robot_position.x,
                robot_radius * np.sin(robot_orientation + np.deg2rad(140))
                + robot_position.y,
            ],
            [
                robot_radius * np.cos(robot_orientation) + robot_position.x,
                robot_radius * np.sin(robot_orientation) + robot_position.y,
            ],
            [
                robot_radius * np.cos(robot_orientation - np.deg2rad(140))
                + robot_position.x,
                robot_radius * np.sin(robot_orientation - np.deg2rad(140))
                + robot_position.y,
            ],
        ]
