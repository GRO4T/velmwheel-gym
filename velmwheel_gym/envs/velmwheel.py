import logging
import enum
import sys
import time

import numpy as np

import gym
import rclpy
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Empty

# from velmwheel_gym_msgs.msg import ContactState
from velmwheel_gazebo_msgs.msg import ContactState

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)
handler = logging.StreamHandler(sys.stdout)
handler.setLevel(logging.INFO)
formatter = logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s")
handler.setFormatter(formatter)
logger.addHandler(handler)


class VelmwheelEnv(gym.Env):
    def __init__(self):
        # Initialize and configure ROS2 node
        rclpy.init()
        self._node = rclpy.create_node(self.__class__.__name__)
        self._reset_service = self._node.create_client(Empty, "/reset_world")
        self._movement_pub = self._node.create_publisher(
            Twist,
            "/velmwheel/base/velocity_setpoint",
            qos_profile=10,
        )
        self._observation_sub = self._node.create_subscription(
            PointCloud2,
            "/velmwheel/lidars/cloud/combined",
            self._observation_callback,
            qos_profile=qos_profile_sensor_data,
        )
        self._collision_sub = self._node.create_subscription(
            ContactState,
            "/velmwheel/contacts",
            self._collision_callback,
            qos_profile=qos_profile_system_default,
        )

        # Class variables
        self._observation_msg = None
        self._is_collided = False

        self.action_space = gym.spaces.Discrete(5)
        self.observation_space = gym.spaces.Box(
            low=0, high=255, shape=(34560,), dtype=np.uint8
        )

        self._action_to_direction = {
            0: [0.0, 0.0],
            1: [1.0, 0.0],
            2: [-1.0, 0.0],
            3: [0.0, 1.0],
            4: [0.0, -1.0],
        }

    def step(self, action):
        print(action)
        self._move(action)
        time.sleep(1)

        obs = self._observe()
        reward = self._calculate_reward()
        done = self._is_collided
        info = {}

        return obs, reward, done, info

    def reset(self):
        while not self._reset_service.wait_for_service(timeout_sec=1.0):
            logger.info("/reset_world service not available, waiting again...")

        reset_future = self._reset_service.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self._node, reset_future)

        self._is_collided = False

        return self._observe()

    def close(self):
        logger.info("Closing " + self.__class__.__name__ + " environment.")
        self._move(VelmwheelEnv.Actions.STOP)
        self._node.destroy_node()
        rclpy.shutdown()

    def _move(self, action):
        motion_cmd = Twist()

        direction = self._action_to_direction[action]
        motion_cmd.linear.x = direction[0]
        motion_cmd.linear.y = direction[1]

        # NOTE(damiankolaska) - temporary
        self._is_moved = direction[0] != 0 or direction[1] != 0

        self._movement_pub.publish(motion_cmd)
        logger.debug("Publishing movement commmand: %s" % str(motion_cmd))

    def _observe(self) -> any:
        rclpy.spin_once(self._node)
        while self._observation_msg is None:
            rclpy.spin_once(self._node)
        obs = self._observation_msg.data
        obs = np.array(obs)
        return obs

    def _calculate_reward(self) -> float:
        print(f"{self._is_collided=}")
        if self._is_collided:
            return -100.0
        elif self._is_moved:
            return 1.0
        else:
            return 0.0

    def _observation_callback(self, message):
        self._observation_msg = message
        logger.debug("Received observation")

    def _collision_callback(self, message):
        self._is_collided = True
        logger.debug(f"Received collision msg: {message}")
