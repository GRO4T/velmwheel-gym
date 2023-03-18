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
    """
    TODO
    """

    class Actions(enum.Enum):
        STOP = 0
        FORWARD = 1
        BACKWARD = 2
        LEFT = 3
        RIGHT = 4

    def __init__(self):
        # Initialize and configure ROS2 node
        rclpy.init()
        self._node = rclpy.create_node(self.__class__.__name__)
        self._movement_pub = self._node.create_publisher(
            Twist,
            "/velmwheel/base/velocity_setpoint",
            qos_profile=10,
        )
        self._reset_sim = self._node.create_client(Empty, "/reset_world")
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
        self._action_time = None

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

    def step(self, action: Actions):
        print(action)
        self._move(action)
        time.sleep(1)
        self._action_time = rclpy.clock.Clock().now().nanoseconds

        obs = self._observe()
        reward = 1
        done = False
        info = {}

        return obs, reward, done, info

    def reset(self):
        print("reset")
        while not self._reset_sim.wait_for_service(timeout_sec=1.0):
            logger.info("/reset_simulation service not available, waiting again...")

        reset_future = self._reset_sim.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self._node, reset_future)

        self._action_time = rclpy.clock.Clock().now().nanoseconds

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

        self._movement_pub.publish(motion_cmd)
        logger.debug("Publishing movement commmand: %s" % str(motion_cmd))

    def _observe(self) -> any:
        rclpy.spin_once(self._node)
        while self._observation_msg is None:
            rclpy.spin_once(self._node)
        obs = self._observation_msg.data
        obs = np.array(obs)
        return obs

    def _observation_callback(self, message):
        self._observation_msg = message
        logger.debug("Received observation")

    def _collision_callback(self, message):
        self._collision_msg = message
        logger.debug(f"Received collision msg: {message}")
