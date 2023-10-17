""" TODO(kolasdam): write docstring. """
import logging
import math
import random
from collections import namedtuple

import gym
import numpy as np
import rclpy
from std_srvs.srv import Empty
from visualization_msgs.msg import Marker, MarkerArray

from velmwheel_gym.constants import DEFAULT_QOS_PROFILE
from velmwheel_gym.robot import VelmwheelRobot
from velmwheel_gym.types import Point

logger = logging.getLogger(__name__)


class VelmwheelEnv(gym.Env):
    def __init__(self):
        super().__init__()

        rclpy.init()
        self._node = rclpy.create_node(self.__class__.__name__)

        self._reset_service = self._node.create_client(Empty, "/reset_world")

        self._goal_marker_pub = self._node.create_publisher(
            MarkerArray,
            "/velmwheel/markers_map/visualization",
            qos_profile=DEFAULT_QOS_PROFILE,
        )

        self._robot = VelmwheelRobot()

        self.action_space = gym.spaces.Discrete(5)
        self.observation_space = gym.spaces.Box(
            low=-100.0, high=100.0, shape=(4,), dtype=np.float64
        )

        self._goal: Point = None
        self._min_goal_dist: float = 0

    @property
    def goal(self) -> Point:
        """Robot's navigation goal."""
        return self._goal

    @goal.setter
    def goal(self, point: Point):
        self._goal = point

    @property
    def min_goal_dist(self) -> float:
        """Minimum distance to goal to reach it"""
        return self._min_goal_dist

    @min_goal_dist.setter
    def min_goal_dist(self, dist: float):
        self._min_goal_dist = dist

    def step(self, action):
        self._robot.move(action)
        self._robot.update()

        obs = self._observe()

        dist_to_goal = self._calculate_distance_to_goal(obs)

        reward = self._calculate_reward(dist_to_goal)
        done = self._robot.is_collide or dist_to_goal < self._min_goal_dist
        info = {}

        return obs, reward, done, info

    def reset(self):
        while not self._reset_service.wait_for_service(timeout_sec=1.0):
            print("Waiting for Velmwheel's Gazebo sim...")
            logger.info("/reset_world service not available, waiting again...")

        reset_future = self._reset_service.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self._node, reset_future)

        self._robot.reset()
        self._robot.update()

        self._set_new_goal()
        self._publish_goal_marker()

        return self._observe()

    def close(self):
        logger.info("Closing " + self.__class__.__name__ + " environment.")
        self._move(0)
        self._node.destroy_node()
        rclpy.shutdown()

    def _observe(self) -> np.array:
        position = self._robot.position
        return np.array([position.x, position.y, self._goal.x, self._goal.y])

    def _calculate_distance_to_goal(self, obs: np.array) -> float:
        pos_x, pos_y, *_ = obs
        return math.dist(self._goal, (pos_x, pos_y))

    def _calculate_reward(self, dist_to_goal: float) -> float:
        if self._robot.is_collide:
            return -1.0

        if dist_to_goal < self._min_goal_dist:
            return 1.0

        return 0.0

    def _set_new_goal(self):
        r = 3.0
        self._goal = Point(x=random.uniform(-r, r), y=random.uniform(-r, r))
        logger.info(f"{self._goal=}")

    def _publish_goal_marker(self):
        ma = MarkerArray()
        m = Marker()

        m.header.frame_id = "world"
        m.ns = "velmwheel"
        m.id = 0
        m.type = Marker.SPHERE
        m.scale.x = 1.0
        m.scale.y = 1.0
        m.scale.z = 1.0
        m.pose.position.x = self._goal.x
        m.pose.position.y = self._goal.y
        m.pose.position.z = 0.0
        m.pose.orientation.x = 0.0
        m.pose.orientation.y = 0.0
        m.pose.orientation.z = 0.0
        m.pose.orientation.w = 1.0
        m.action = Marker.ADD
        m.color.r = 0.0
        m.color.g = 1.0
        m.color.b = 0.0
        m.color.a = 1.0

        ma.markers = [m]

        self._goal_marker_pub.publish(ma)
