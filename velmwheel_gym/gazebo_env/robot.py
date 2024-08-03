import copy
import logging
import math
import time
from typing import Optional

import numpy as np
import rclpy
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped, Twist
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from scan_tools_msgs.srv import SetPose
from sensor_msgs.msg import PointCloud2
from velmwheel_gazebo_msgs.msg import ContactState

from ros2_numpy.point_cloud2 import pointcloud2_to_array
from velmwheel_gym.constants import BASE_STEP_TIME
from velmwheel_gym.gazebo_env.ros_utils import call_service, create_ros_service_client
from velmwheel_gym.types import Point

logger = logging.getLogger(__name__)

ROBOT_MOVEMENT_TOPIC = "/velmwheel/base/velocity_setpoint"
ROBOT_POSE_SIMULATION_TOPIC = "/velmwheel/sim/pose"
ROBOT_COLLISION_TOPIC = "/velmwheel/contacts"
ODOM_LASER_POSE_TOPIC = "/velmwheel/odom/laser/pose"
LASER_SCAN_MATCHER_SET_POSE_TOPIC = "/velmwheel/laser_scan_matcher/set_pose"
ENCODERS_SET_POSE_TOPIC = "/velmwheel/odom/encoders/set_pose"
SET_ENTITY_STATE_TOPIC = "/set_entity_state"
LIDAR_TOPIC = "/velmwheel/lidars/cloud/combined"

STALE_MEASUREMENT_TIMEOUT_SEC = 5
STALE_MEASUREMENT_RECOVERY_TIMEOUT_SEC = 10


class VelmwheelRobot:
    def __init__(self):
        logger.debug("Creating VelmwheelRobot")

        self._is_collide: bool = False
        self._position: Optional[Point] = None
        self._theta: float = None
        self._position_tstamp: float = 0.0
        self._lidar_pointcloud_raw: Optional[np.array] = None
        self._lidar_data: Optional[list[float]] = None
        self._prev_lidar_data: Optional[list[float]] = None
        self._lidar_tstamp: float = 0.0
        self._lidar_max_range: Optional[float] = None
        self._simulation_time: Optional[int] = None
        self._real_time_factor: float = 1.0
        self._ignore_collisions_until: float = 0.0

        self._ros_init()

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
        self._lidar_sub = self._node.create_subscription(
            PointCloud2,
            LIDAR_TOPIC,
            self._lidar_callback,
            qos_profile=qos_profile_sensor_data,
        )

    @property
    def position(self) -> Point:
        """Current robot's position"""
        return self._position

    @property
    def theta(self) -> float:
        """Current robot's orientation."""
        return self._theta

    @property
    def position_tstamp(self) -> float:
        """Timestamp of the last received position."""
        return self._position_tstamp

    @position_tstamp.setter
    def position_tstamp(self, value: float):
        self._position_tstamp = value

    @property
    def lidar_pointcloud_raw(self) -> Optional[np.array]:
        """Raw LIDAR pointcloud data."""
        return self._lidar_pointcloud_raw

    @property
    def lidar_data(self) -> Optional[list[float]]:
        """LIDAR data."""
        return self._lidar_data

    @property
    def normalized_lidar_data(self) -> Optional[list[float]]:
        """Normalized LIDAR data. Measurements are scaled to the range [0, 1]."""
        if not self._lidar_data:
            return None
        return [x / 20.0 for x in self._lidar_data]

    @property
    def lidar_tstamp(self) -> float:
        """Timestamp of the last received LIDAR data."""
        return self._lidar_tstamp

    @lidar_tstamp.setter
    def lidar_tstamp(self, value: float):
        self._lidar_tstamp = value

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

    def reset(self, starting_position: Point, timeout_sec: float = 10.0) -> bool:
        """Resets robot's state."""
        start = time.time()

        self.stop()
        self._ignore_collisions_until = time.time() + 1
        self._is_collide = False
        self._position = None
        self._lidar_data = None

        while self._simulation_time is None:
            rclpy.spin_once(self._node, timeout_sec=1.0)
            if time.time() - start > timeout_sec:
                logger.warning("Failed to reset robot's state")
                return False

        self._set_entity_pose(starting_position)
        self._set_laser_scan_matcher_pose(starting_position)
        self._set_encoders_pose(starting_position)

        while self._position is None or self._lidar_data is None:
            rclpy.spin_once(self._node, timeout_sec=1.0)
            if time.time() - start > timeout_sec:
                logger.warning("Failed to reset robot's state")
                return False

        return True

    def update(self):
        """Updates robot's state measurements."""
        rclpy.spin_once(self._node, timeout_sec=BASE_STEP_TIME / self.real_time_factor)

        current_time = time.time()
        if (
            current_time - self._position_tstamp > STALE_MEASUREMENT_TIMEOUT_SEC
            or current_time - self._lidar_tstamp > STALE_MEASUREMENT_TIMEOUT_SEC
        ):
            logger.warning(
                f"Stale measurements {current_time=} {self._position_tstamp=} {self._lidar_tstamp=}"
            )
            return self._stale_measurement_recovery()
        return True

    def move(self, action):
        """Sends movement controls to the robot."""
        self._is_collide = False
        motion_cmd = Twist()

        motion_cmd.linear.x = 0.5 * action[0]
        motion_cmd.linear.y = 0.5 * action[1]
        motion_cmd.angular.z = action[2]

        self._movement_pub.publish(motion_cmd)

    def stop(self):
        """Stops robot's movement."""
        self.move([0.0, 0.0, 0.0])

    def _stale_measurement_recovery(self):
        logger.warning("Trying to recover from stale measurements")
        self.stop()
        start = time.time()
        self._position = None
        self._lidar_data = None
        self._simulation_time = None

        while (
            self._position is None
            or self._lidar_data is None
            or self._simulation_time is None
        ):
            rclpy.spin_once(self._node, timeout_sec=1.0)
            if time.time() - start > STALE_MEASUREMENT_RECOVERY_TIMEOUT_SEC:
                logger.error("Failed to recover from stale measurements")
                return False
        return True

    def _collision_callback(self, _):
        if time.time() > self._ignore_collisions_until:
            self._is_collide = True

    def _odom_laser_pose_callback(self, message: PoseStamped):
        self._simulation_time = message.header.stamp.sec

    def _position_callback(self, message: PoseStamped):
        if self._position is None:
            logger.debug(f"Position after reset: {message.pose.position}")
        p = message.pose.position
        self._position = Point(x=p.x, y=p.y)

        def quaternion_to_euler_z(q):
            q.w, q.x, q.y, q.z
            t1 = 2.0 * (q.w * q.z + q.x * q.y)
            t2 = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            return np.arctan2(t1, t2)

        self._theta = quaternion_to_euler_z(message.pose.orientation)
        while self._theta <= -np.pi:
            self._theta += 2 * np.pi
        while self._theta > np.pi:
            self._theta -= 2 * np.pi
        self._position_tstamp = time.time()

    def _lidar_callback(self, message: PointCloud2):
        if not self.position:
            logger.warning("Position is not set yet")
            return
        self._lidar_data = []
        self._lidar_pointcloud_raw = pointcloud2_to_array(message)

        for point in pointcloud2_to_array(message)[::12]:
            x, y, _, _ = point
            point_2d = np.array([x, y])
            r = np.sqrt(point_2d.dot(point_2d))
            self._lidar_data.append(min(r, 20.0))

        # remove NaNs
        for i in range(0, len(self._lidar_data), 2):
            if math.isnan(self._lidar_data[i]):
                self._lidar_data[i] = self._prev_lidar_data[i]
            if math.isnan(self._lidar_data[i + 1]):
                self._lidar_data[i + 1] = self._prev_lidar_data[i + 1]

        # remove Infs
        self._lidar_data = [x if not math.isinf(x) else 20.0 for x in self._lidar_data]
        self._prev_lidar_data = copy.deepcopy(self._lidar_data)

        self._lidar_tstamp = time.time()

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
