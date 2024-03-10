import logging

import rclpy
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped, Twist
from rclpy.qos import qos_profile_system_default
from scan_tools_msgs.srv import SetPose
from velmwheel_gazebo_msgs.msg import ContactState

from velmwheel_gym.constants import BASE_STEP_TIME
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
SET_ENTITY_STATE_TOPIC = "/set_entity_state"


class VelmwheelRobot:
    def __init__(self):
        logger.debug("Creating VelmwheelRobot")

        self._is_collide = False
        self._position: Point = None
        self._position_tstamp_sec: int = None
        self._position_tstamp_nanosec: int = None
        self._simulation_time: int = None
        self._real_time_factor: float = 1.0

        self._ros_init()
        self.update()

        logger.debug("VelmwheelRobot created")

    def _ros_init(self):
        self._node = rclpy.create_node(self.__class__.__name__)
        # services
        self._laser_scan_matcher_set_pose_srv = create_ros_service_client(
            self._node, SetPose, LASER_SCAN_MATCHER_SET_POSE_TOPIC
        )
        self._set_entity_pose_srv = create_ros_service_client(
            self._node, SetEntityState, SET_ENTITY_STATE_TOPIC
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
    def position_tstamp(self) -> str:
        """Timestamp of the last received position."""
        return f"{self._position_tstamp_sec}.{self._position_tstamp_nanosec}"

    @property
    def is_collide(self) -> bool:
        """Flag signalling whether robot's is in collision with an obstacle."""
        return self._is_collide

    @property
    def real_time_factor(self) -> float:
        """Real time factor for the simulation."""
        return self._real_time_factor

    @real_time_factor.setter
    def real_time_factor(self, factor: float):
        self._real_time_factor = factor

    def reset(self, starting_position: Point):
        """Resets robot's state."""
        self.move([0.0, 0.0])
        self._is_collide = False
        self._set_entity_pose(starting_position)
        self._set_laser_scan_matcher_pose(starting_position)
        self._set_encoders_pose(starting_position)
        self._position = None

    def update(self):
        """Updates robot's state measurements."""
        # NOTE: 50% of step time (adjusted for real time factor)
        # can be spent waiting for environment measurements
        rclpy.spin_once(
            self._node, timeout_sec=0.5 * BASE_STEP_TIME / self.real_time_factor
        )
        while self._position is None or self._simulation_time is None:
            rclpy.spin_once(self._node)

    def move(self, action):
        """Sends movement controls to the robot."""
        motion_cmd = Twist()

        motion_cmd.linear.x = action[0]
        motion_cmd.linear.y = action[1]

        self._movement_pub.publish(motion_cmd)

    def _collision_callback(self, _):
        self._is_collide = True

    def _odom_laser_pose_callback(self, message: PoseStamped):
        self._simulation_time = message.header.stamp.sec

    def _position_callback(self, message: PoseStamped):
        if self._position is None:
            logger.debug(f"Position after reset: {message.pose.position}")
        p = message.pose.position
        self._position = Point(x=p.x, y=p.y)
        self._position_tstamp_sec = message.header.stamp.sec
        self._position_tstamp_nanosec = message.header.stamp.nanosec

    def _publish_navigation_initial_pose(self):
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.frame_id = "map"
        initial_pose.pose.pose.position.x = self._position.x
        initial_pose.pose.pose.position.y = self._position.y
        self._nav_initial_position_pub.publish(initial_pose)

    def _set_entity_pose(self, position: Point):
        """Sets robot's position in the Gazebo simulation."""
        req = self._set_entity_pose_srv.request
        req.state.name = "velmwheel"
        req.state.pose.position.x = position.x
        req.state.pose.position.y = position.y
        req.state.pose.position.z = 0.0
        req.state.pose.orientation.x = 0.0
        req.state.pose.orientation.y = 0.0
        req.state.pose.orientation.z = 0.0
        req.state.pose.orientation.w = 1.0
        call_service(self._set_entity_pose_srv)

    def _set_laser_scan_matcher_pose(self, position: Point):
        """Sets pose estimation of laser-based odometry."""
        logger.debug("Reset laser scan matcher pose")
        req = self._laser_scan_matcher_set_pose_srv.request
        req.pose.header.frame_id = "odom"
        req.pose.header.stamp.sec = self._simulation_time + 1
        req.pose.pose.position.x = position.x
        req.pose.pose.position.y = position.y
        req.pose.pose.position.z = 0.0
        req.pose.pose.orientation.x = 0.0
        req.pose.pose.orientation.y = 0.0
        req.pose.pose.orientation.z = 0.0
        req.pose.pose.orientation.w = 1.0
        call_service(self._laser_scan_matcher_set_pose_srv)

    def _set_encoders_pose(self, position: Point):
        """Sets pose estimation of wheel encoders."""
        logger.debug("Reset encoders pose")
        pose = Pose()
        pose.position.x = position.x
        pose.position.y = position.y
        pose.position.z = 0.0
        self._encoders_set_pose_pub.publish(pose)
