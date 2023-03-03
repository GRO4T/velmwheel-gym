import logging
import enum
import sys

import gym
import rclpy
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Empty
from gazebo_msgs.msg import ContactState

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)
handler = logging.StreamHandler(sys.stdout)
handler.setLevel(logging.DEBUG)
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
        self._reset_sim = self._node.create_client(Empty, "/reset_simulation")
        self._observation_sub = self._node.create_subscription(
            PointCloud2,
            "/velmwheel/lidars/cloud/combined",
            self._observation_callback,
            qos_profile=qos_profile_sensor_data,
        )
        self._collision_sub = self._node.create_subscription(
            ContactState,
        )
        # Class variables
        self._observation_msg = None
        self._action_time = None

    def step(self, action: Actions):
        self._move(action)
        self._action_time = rclpy.clock.Clock().now().nanoseconds
        self._observe()

        obs = self._observation_msg.data
        reward = 1
        done = False
        info = {}

        return obs, reward, done, info

    def reset(self):
        while not self._reset_sim.wait_for_service(timeout_sec=1.0):
            logger.info("/reset_simulation service not available, waiting again...")

        reset_future = self._reset_sim.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self._node, reset_future)

        self._action_time = rclpy.clock.Clock().now().nanoseconds

        self._observe()
        obs = self._observation_msg.data

        return obs

    def close(self):
        logger.info("Closing " + self.__class__.__name__ + " environment.")
        self._move(VelmwheelEnv.Actions.STOP)
        self._node.destroy_node()
        rclpy.shutdown()

    def _move(self, action: Actions):
        motion_cmd = Twist()

        if action == VelmwheelEnv.Actions.FORWARD:
            motion_cmd.linear.x = 1.0
        elif action == VelmwheelEnv.Actions.BACKWARD:
            motion_cmd.linear.x = -1.0
        elif action == VelmwheelEnv.Actions.LEFT:
            motion_cmd.linear.y = 1.0
        elif action == VelmwheelEnv.Actions.RIGHT:
            motion_cmd.linear.y = -1.0

        self._movement_pub.publish(motion_cmd)
        logger.debug("Publishing movement commmand: %s" % str(motion_cmd))

    def _observe(self) -> any:
        rclpy.spin_once(self._node)
        # while self._observation_msg is None or self._is_observation_prior_to_action():
        #     rclpy.spin_once(self._node)

    # This does not work probably because NTP server is not running
    # def _is_observation_prior_to_action(self) -> bool:
    # observation_time = int(
    #     str(self._observation_msg.header.stamp.sec)
    #     + (str(self._observation_msg.header.stamp.nanosec))
    # )
    # logger.debug(f"observation_time_secs={self._observation_msg.header.stamp.sec}")
    # logger.debug(
    #     f"observation_time_nsecs={self._observation_msg.header.stamp.nanosec}"
    # )
    # logger.debug(f"observation={self._observation_msg.__dir__}")
    # logger.debug(f"{observation_time=}")
    # logger.debug(f"{self._action_time=}")
    # return observation_time < self._action_time

    def _observation_callback(self, message):
        self._observation_msg = message
        logger.debug("Received observation")
