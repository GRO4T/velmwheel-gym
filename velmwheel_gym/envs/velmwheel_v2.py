import logging
import sys
import os

import numpy as np

import gym
import rclpy
from std_srvs.srv import Empty

from velmwheel_gym.robot import VelmwheelRobot
from velmwheel_gym.reward import VelmwheelReward

# from velmwheel_gym_msgs.msg import ContactState
from velmwheel_gazebo_msgs.msg import ContactState

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)
handler = logging.StreamHandler(sys.stdout)
handler.setLevel(logging.INFO)
formatter = logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s")
handler.setFormatter(formatter)
logger.addHandler(handler)


class VelmwheelEnvV2(gym.Env):
    def __init__(self):
        super().__init__()
        # Initialize and configure ROS2 node
        rclpy.init()
        self._node = rclpy.create_node(self.__class__.__name__)
        self._reset_service = self._node.create_client(Empty, "/reset_world")

        self._robot = VelmwheelRobot()
        self._reward = VelmwheelReward(self._robot)

        self.action_space = gym.spaces.Discrete(5)
        self.observation_space = gym.spaces.Box(
            low=-100.0, high=100.0, shape=(2,), dtype=np.float64
        )

    def step(self, action):
        self._robot.move(action)
        self._robot.update()

        pos = self._robot.get_position()
        x = [pos.x, pos.y]
        obs = x
        reward = self._reward.calculate()
        done = self._robot.is_collide() or self._reward.is_done()
        info = {}

        return obs, reward, done, info

    def reset(self):
        while not self._reset_service.wait_for_service(timeout_sec=1.0):
            logger.info("/reset_world service not available, waiting again...")

        reset_future = self._reset_service.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self._node, reset_future)

        self._robot.reset()
        self._robot.update()

        pos = self._robot.get_position()
        x = [pos.x, pos.y]
        return x

    def close(self):
        logger.info("Closing " + self.__class__.__name__ + " environment.")
        self._move(0)
        self._node.destroy_node()
        rclpy.shutdown()
