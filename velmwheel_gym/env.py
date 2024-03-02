""" TODO(kolasdam): write docstring. """

import logging
import math
import random

import gym
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.qos import qos_profile_system_default
from std_srvs.srv import Empty

from velmwheel_gym.constants import DEFAULT_QOS_PROFILE
from velmwheel_gym.robot import VelmwheelRobot
from velmwheel_gym.types import Point
from velmwheel_gym.utils import call_service, create_ros_service_client

logger = logging.getLogger(__name__)

START_SIM_TOPIC = "/start_sim"
STOP_SIM_TOPIC = "/stop_sim"
RESTART_SIM_TOPIC = "/restart_sim"
RESET_WORLD_TOPIC = "/reset_world"
NAVIGATION_GOAL_TOPIC = "/goal_pose"
GLOBAL_PLANNER_PATH_TOPIC = "/plan"


class VelmwheelEnv(gym.Env):
    def __init__(self):
        logger.debug("Creating VelmwheelEnv")
        super().__init__()

        self.action_space = gym.spaces.Discrete(5)
        self.observation_space = gym.spaces.Box(
            low=-100.0, high=100.0, shape=(4,), dtype=np.float64
        )
        self._goal: Point = None
        self._min_goal_dist: float = 0
        self._path = None

        rclpy.init()
        self._node = rclpy.create_node(self.__class__.__name__)

        self._start_sim_srv = create_ros_service_client(
            self._node, Empty, START_SIM_TOPIC
        )
        self._stop_sim_srv = create_ros_service_client(
            self._node, Empty, STOP_SIM_TOPIC
        )
        self._restart_sim_srv = create_ros_service_client(
            self._node, Empty, RESTART_SIM_TOPIC
        )

        call_service(self._start_sim_srv)

        self._reset_world_srv = self._node.create_client(Empty, RESET_WORLD_TOPIC)

        self._navigation_goal_pub = self._node.create_publisher(
            PoseStamped,
            NAVIGATION_GOAL_TOPIC,
            qos_profile=DEFAULT_QOS_PROFILE,
        )

        self._navigation_plan_sub = self._node.create_subscription(
            Path,
            GLOBAL_PLANNER_PATH_TOPIC,
            self._global_planner_callback,
            qos_profile=qos_profile_system_default,
        )

        self._robot = VelmwheelRobot()
        self._robot.update()

        self._episode = 0  # TODO: use episode count from gym.Env

        logger.debug("VelmwheelEnv created")

    @property
    def goal(self) -> Point:
        """Robot's navigation goal."""
        return self._goal

    @goal.setter
    def goal(self, point: Point):
        self._goal = point

    @property
    def min_goal_dist(self) -> float:
        """Minimum distance to goal to reach it"""
        return self._min_goal_dist

    @min_goal_dist.setter
    def min_goal_dist(self, dist: float):
        self._min_goal_dist = dist

    @property
    def path(self) -> list[Point]:
        """Path calculated by global planner."""
        return self._path

    @path.setter
    def path(self, path: list[Point]):
        self._path = path

    def step(self, action):
        rclpy.spin_once(self._node, timeout_sec=1.0)
        self._robot.move(action)
        self._robot.update()

        obs = self._observe()

        dist_to_goal = self._calculate_distance_to_goal(obs)

        reward = self._calculate_reward(dist_to_goal)
        done = self._robot.is_collide or dist_to_goal < self._min_goal_dist
        info = {}

        return obs, reward, done, info

    def reset(self):
        self._reset_simulation()
        self._robot.reset()

        self.path = None
        self._generate_next_goal()
        self._wait_for_new_path()

        return self._observe()

    def close(self):
        logger.info("Closing " + self.__class__.__name__ + " environment.")
        self._robot.move(0)
        self._node.destroy_node()
        rclpy.shutdown()

    def _observe(self) -> np.array:
        position = self._robot.position
        return np.array([position.x, position.y, self._goal.x, self._goal.y])

    def _calculate_distance_to_goal(self, obs: np.array) -> float:
        pos_x, pos_y, *_ = obs
        return math.dist(self._goal, (pos_x, pos_y))

    def _calculate_reward(self, dist_to_goal: float) -> float:
        if self._robot.is_collide:
            logger.debug("FAILURE: Robot collided with an obstacle")
            return -1.0

        if dist_to_goal < self._min_goal_dist:
            logger.debug(f"SUCCESS: Robot reached goal at {self._goal=}")
            return 1.0

        return 0.0

    def _reset_simulation(self):
        # TODO: use wait_for_service from utils
        while not self._reset_world_srv.wait_for_service(timeout_sec=1.0):
            logger.debug(f"{RESET_WORLD_TOPIC} service not available, waiting again...")

        reset_future = self._reset_world_srv.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self._node, reset_future)

    def _generate_next_goal(self):
        available_goals = [
            Point(3.0, 3.0),
            Point(3.0, -3.0),
            Point(-3.0, 3.0),
            Point(-3.0, -3.0),
        ]
        self.goal = random.choice(available_goals)
        logger.debug(f"{self.goal=}")

    def _publish_goal(self):
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.pose.position.x = self.goal.x
        goal.pose.position.y = self.goal.y
        self._navigation_goal_pub.publish(goal)

    def _wait_for_new_path(self):
        while not self.path:
            logger.debug("Waiting for global planner to calculate a new path")
            self._publish_goal()
            rclpy.spin_once(self._node, timeout_sec=1.0)

    def _global_planner_callback(self, message: Path):
        if self.path:  # update path only at the start of the episode
            return

        self.path = [pose_stamped.pose.position for pose_stamped in message.poses]
