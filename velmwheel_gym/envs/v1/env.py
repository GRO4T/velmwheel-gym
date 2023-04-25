""" TODO(kolasdam): write docstring. """
import logging
import math

import numpy as np

import gym
import rclpy
from std_srvs.srv import Empty

from velmwheel_gym.envs.v1.robot import VelmwheelRobot


logger = logging.getLogger(__name__)


class VelmwheelEnvV1(gym.Env):
    def __init__(self):
        super().__init__()
        # Initialize and configure ROS2 node
        rclpy.init()
        self._node = rclpy.create_node(self.__class__.__name__)
        self._reset_service = self._node.create_client(Empty, "/reset_world")

        self._robot = VelmwheelRobot()

        self.action_space = gym.spaces.Discrete(5)
        self.observation_space = gym.spaces.Box(
            low=-100.0, high=100.0, shape=(2,), dtype=np.float64
        )

        self.goal = [3, 3]
        self.min_goal_dist = 1.0

    def step(self, action):
        self._robot.move(action)
        self._robot.update()

        obs = self._observe()

        dist_to_goal = math.dist(self.goal, obs)

        reward = self._calculate_reward(dist_to_goal)
        done = self._robot.is_collide() or dist_to_goal < self.min_goal_dist
        info = {}

        return obs, reward, done, info

    def reset(self):
        while not self._reset_service.wait_for_service(timeout_sec=1.0):
            logger.info("/reset_world service not available, waiting again...")

        reset_future = self._reset_service.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self._node, reset_future)

        self._robot.reset()
        self._robot.update()

        return self._observe()

    def close(self):
        logger.info("Closing " + self.__class__.__name__ + " environment.")
        self._move(0)
        self._node.destroy_node()
        rclpy.shutdown()

    def _observe(self) -> np.array:
        position_as_point = self._robot.get_position()
        position_as_list = [position_as_point.x, position_as_point.y]
        return np.array(position_as_list)

    def _calculate_reward(self, dist_to_goal: float) -> float:
        if self._robot.is_collide():
            logger.debug("Collision!")
            return -100.0

        if dist_to_goal < self.min_goal_dist:
            logger.debug("Goal achieved!")
            return 200.0

        return -0.001
