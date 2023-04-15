import sys
import time
import logging
import rclpy
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist, PoseStamped
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from velmwheel_gazebo_msgs.msg import ContactState


logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)
handler = logging.FileHandler("./logs/velmwheel/default.log")
handler.setLevel(logging.DEBUG)
formatter = logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s")
handler.setFormatter(formatter)
logger.addHandler(handler)


ACTION_TO_DIRECTION = {
    0: [0.0, 0.0],
    1: [1.0, 0.0],
    2: [-1.0, 0.0],
    3: [0.0, 1.0],
    4: [0.0, -1.0],
}


class VelmwheelRobot:
    def __init__(self):
        # Initialize and configure ROS2 node
        self._node = rclpy.create_node(self.__class__.__name__)

        self._lidar_data = None
        self._is_collide = False
        self._position = None

        self._movement_pub = self._node.create_publisher(
            Twist,
            "/velmwheel/base/velocity_setpoint",
            qos_profile=10,
        )

        self._lidar_sub = self._node.create_subscription(
            PointCloud2,
            "/velmwheel/lidars/cloud/combined",
            self._lidar_callback,
            qos_profile=qos_profile_sensor_data,
        )

        self._collision_sub = self._node.create_subscription(
            ContactState,
            "/velmwheel/contacts",
            self._collision_callback,
            qos_profile=qos_profile_system_default,
        )

        self._position_sub = self._node.create_subscription(
            PoseStamped,
            # "/velmwheel/odom/filtered/pose",
            "/velmwheel/sim/pose",
            self._position_callback,
            qos_profile=qos_profile_sensor_data,
        )

    def reset(self):
        self.move(0)
        self._is_collide = False
        self._lidar_data = None
        self._position = None

    def update(self):
        rclpy.spin_once(self._node)
        while self._lidar_data is None or self._position is None:
            rclpy.spin_once(self._node)

    def move(self, action):
        motion_cmd = Twist()

        direction = ACTION_TO_DIRECTION[action]
        motion_cmd.linear.x = direction[0]
        motion_cmd.linear.y = direction[1]

        self._movement_pub.publish(motion_cmd)
        # logger.debug("Publishing movement commmand: %s" % str(motion_cmd))
        time.sleep(0.16 / 6)

    def get_lidar_data(self) -> any:
        return self._lidar_data

    def get_position(self):
        return self._position

    def is_collide(self) -> bool:
        return self._is_collide

    def _lidar_callback(self, message):
        self._lidar_data = message.data

    def _collision_callback(self, message):
        self._is_collide = True
        # logger.debug(f"Received collision msg: {message}")

    def _position_callback(self, message: PoseStamped):
        if self._position is None:
            logger.debug(f"Position after reset: {message.pose.position}")
        self._position = message.pose.position
        # logger.debug(f"Received robot position: {self._position}")