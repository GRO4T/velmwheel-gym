import logging
import time

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from velmwheel_gazebo_msgs.msg import ContactState

from velmwheel_gym.constants import ACTION_TO_DIRECTION
from velmwheel_gym.types import Point

logger = logging.getLogger(__name__)


class VelmwheelRobot:
    def __init__(self):
        # Initialize and configure ROS2 node
        self._node = rclpy.create_node(self.__class__.__name__)

        self._is_collide = False
        self._position: Point = None

        self._movement_pub = self._node.create_publisher(
            Twist,
            "/velmwheel/base/velocity_setpoint",
            qos_profile=qos_profile_system_default,
        )

        self._nav_initial_position_pub = self._node.create_publisher(
            PoseWithCovarianceStamped,
            "/initialpose",
            qos_profile=qos_profile_system_default,
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

    @property
    def position(self) -> Point:
        """Current robot's position"""
        return self._position

    @property
    def is_collide(self) -> bool:
        """Flag signalling whether robot's is in collision with an obstacle."""
        return self._is_collide

    def reset(self):
        """Resets robot state."""
        self.move(0)
        self._is_collide = False
        self._position = None

    def update(self):
        """Spins ROS2 node to obtain current measurements."""
        rclpy.spin_once(self._node)
        while self._position is None:
            rclpy.spin_once(self._node)

    def move(self, action):
        """Sends movement controls to Velmwheel robot through ROS2 interface."""
        motion_cmd = Twist()

        direction = ACTION_TO_DIRECTION[action]
        motion_cmd.linear.x = direction[0]
        motion_cmd.linear.y = direction[1]

        self._movement_pub.publish(motion_cmd)
        time.sleep(0.16 / 6)

    def _collision_callback(self, _):
        self._is_collide = True
        # logger.debug(f"Received collision msg: {message}")

    def _position_callback(self, message: PoseStamped):
        if self._position is None:
            logger.debug(f"Position after reset: {message.pose.position}")
        p = message.pose.position
        self._position = Point(x=p.x, y=p.y)
        self._publish_nav_initial_pose()

    def _publish_nav_initial_pose(self):
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.frame_id = "map"
        initial_pose.pose.pose.position.x = self._position.x
        initial_pose.pose.pose.position.y = self._position.y
        self._nav_initial_position_pub.publish(initial_pose)
