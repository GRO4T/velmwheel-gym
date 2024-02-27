import logging
import time

import rclpy
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped, Twist
from rclpy.qos import qos_profile_system_default
from scan_tools_msgs.srv import SetPose
from velmwheel_gazebo_msgs.msg import ContactState

from velmwheel_gym.constants import ACTION_TO_DIRECTION
from velmwheel_gym.types import Point
from velmwheel_gym.utils import call_service, create_ros_service_client

logger = logging.getLogger(__name__)

ROBOT_MOVEMENT_TOPIC = "/velmwheel/base/velocity_setpoint"
ROBOT_POSE_SIMULATION_TOPIC = "/velmwheel/sim/pose"
ROBOT_COLLISION_TOPIC = "/velmwheel/contacts"
ODOM_LASER_POSE_TOPIC = "/velmwheel/odom/laser/pose"
LASER_SCAN_MATCHER_SET_POSE_TOPIC = "/velmwheel/laser_scan_matcher/set_pose"
NAVIGATION_INITIAL_POSE_TOPIC = "/initialpose"
ENCODERS_SET_POSE_TOPIC = "/velmwheel/odom/encoders/set_pose"


class VelmwheelRobot:
    def __init__(self):
        self._is_collide = False
        self._position: Point = None
        self._simulation_time = 0

        # ROS related stuff
        self._node = rclpy.create_node(self.__class__.__name__)
        # services
        self._laser_scan_matcher_set_pose_srv = create_ros_service_client(
            self._node, SetPose, LASER_SCAN_MATCHER_SET_POSE_TOPIC
        )
        # publishers
        self._movement_pub = self._node.create_publisher(
            Twist,
            ROBOT_MOVEMENT_TOPIC,
            qos_profile=qos_profile_system_default,
        )
        self._nav_initial_position_pub = self._node.create_publisher(
            PoseWithCovarianceStamped,
            NAVIGATION_INITIAL_POSE_TOPIC,
            qos_profile=qos_profile_system_default,
        )
        self._encoders_set_pose_pub = self._node.create_publisher(
            Pose,
            ENCODERS_SET_POSE_TOPIC,
            qos_profile=qos_profile_system_default,
        )
        # subscribers
        self._collision_sub = self._node.create_subscription(
            ContactState,
            ROBOT_COLLISION_TOPIC,
            self._collision_callback,
            qos_profile=qos_profile_system_default,
        )
        self._odom_laser_pose_sub = self._node.create_subscription(
            PoseStamped,
            ODOM_LASER_POSE_TOPIC,
            self._odom_laser_pose_callback,
            qos_profile=qos_profile_system_default,
        )
        self._position_sub = self._node.create_subscription(
            PoseStamped,
            ROBOT_POSE_SIMULATION_TOPIC,
            self._position_callback,
            qos_profile=qos_profile_system_default,
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
        """Resets robot's state."""
        self.move(0)
        self._is_collide = False
        self._position = None
        self._reset_laser_scan_matcher_pose()
        self._reset_encoders_pose()

    def update(self):
        """Updates robot's state measurements."""
        rclpy.spin_once(self._node)
        while self._position is None:
            rclpy.spin_once(self._node)

    def move(self, action):
        """Sends movement controls to the robot."""
        motion_cmd = Twist()

        direction = ACTION_TO_DIRECTION[action]
        motion_cmd.linear.x = direction[0]
        motion_cmd.linear.y = direction[1]

        self._movement_pub.publish(motion_cmd)
        time.sleep(0.16 / 6)

    def _collision_callback(self, _):
        self._is_collide = True

    def _odom_laser_pose_callback(self, message: PoseStamped):
        self._simulation_time = message.header.stamp.sec
        logger.debug(f"Simulation time: {self._simulation_time}")
        logger.debug(f"Laser odom pose: {message.pose}")

    def _position_callback(self, message: PoseStamped):
        if self._position is None:
            logger.debug(f"Position after reset: {message.pose.position}")
        p = message.pose.position
        self._position = Point(x=p.x, y=p.y)

    def _publish_nav_initial_pose(self):
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.frame_id = "map"
        initial_pose.pose.pose.position.x = self._position.x
        initial_pose.pose.pose.position.y = self._position.y
        self._nav_initial_position_pub.publish(initial_pose)

    def _reset_laser_scan_matcher_pose(self):
        """Resets pose estimation of laser-based odometry."""
        logger.debug("Reset laser scan matcher pose")
        req = self._laser_scan_matcher_set_pose_srv.request
        req.pose.header.frame_id = "odom"
        req.pose.header.stamp.sec = self._simulation_time + 1
        req.pose.pose.position.x = 0.0
        req.pose.pose.position.y = 0.0
        req.pose.pose.position.z = 0.0
        req.pose.pose.orientation.x = 0.0
        req.pose.pose.orientation.y = 0.0
        req.pose.pose.orientation.z = 0.0
        req.pose.pose.orientation.w = 1.0
        call_service(self._laser_scan_matcher_set_pose_srv)

    def _reset_encoders_pose(self):
        logger.debug("Reset encoders pose")
        pose = Pose()
        pose.position.x = 0.0
        pose.position.y = 0.0
        pose.position.z = 0.0
        self._encoders_set_pose_pub.publish(pose)
