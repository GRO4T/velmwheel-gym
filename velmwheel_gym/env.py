import logging
import math
import random
import time

import gym
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.qos import qos_profile_system_default
from std_srvs.srv import Empty

from velmwheel_gym.constants import BASE_STEP_TIME, DEFAULT_QOS_PROFILE
from velmwheel_gym.global_guidance_path import (
    POINT_REACHED_THRESHOLD,
    GlobalGuidancePath,
)
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

WAIT_FOR_NEW_PATH_TIMEOUT_SEC = 30
MAP_FRAME_POSITION_ERROR_TOLERANCE = 0.5


class VelmwheelEnv(gym.Env):
    def __init__(self):
        logger.debug("Creating VelmwheelEnv")
        super().__init__()

        self.action_space = gym.spaces.Box(
            low=-1.0, high=1.0, shape=(2,), dtype=np.float64
        )
        self.observation_space = gym.spaces.Box(
            low=-100.0, high=100.0, shape=(6,), dtype=np.float64
        )
        self._goal: Point = None
        self._min_goal_dist: float = 0
        self._real_time_factor: float = 1.0
        self._global_guidance_path = None

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
    def real_time_factor(self) -> float:
        """Real time factor for the simulation."""
        return self._real_time_factor

    @real_time_factor.setter
    def real_time_factor(self, factor: float):
        self._real_time_factor = factor
        self._robot.real_time_factor = factor

    def step(self, action):
        step_time = BASE_STEP_TIME / self.real_time_factor
        start = time.time()

        # NOTE: 50% of step time (adjusted for real time factor)
        # can be spent waiting for environment measurements
        rclpy.spin_once(self._node, timeout_sec=0.5 * step_time)

        self._robot.move(action)
        self._robot.update()

        obs = self._observe()

        dist_to_goal = self._calculate_distance_to_goal(obs)
        num_passed_points = self._global_guidance_path.update(self._robot.position)

        reward, done = self._calculate_reward(dist_to_goal, num_passed_points)
        info = {}

        end = time.time()
        elapsed = end - start
        if elapsed < step_time:
            time.sleep(step_time - elapsed)

        return obs, reward, done, info

    def reset(self):
        self._episode += 1
        self._reset_simulation()
        self._robot.reset()
        self._robot.update()

        self._global_guidance_path = None
        self._generate_next_goal()

        time.sleep(1.0)

        while not self._wait_for_new_path(WAIT_FOR_NEW_PATH_TIMEOUT_SEC):
            logger.warning("Simulation in a bad state. Restarting the simulation.")
            call_service(self._restart_sim_srv)

        return self._observe()

    def close(self):
        logger.info("Closing " + self.__class__.__name__ + " environment.")
        self._robot.move([0.0, 0.0])
        self._node.destroy_node()
        rclpy.shutdown()

    def _observe(self) -> np.array:
        position = self._robot.position

        if not self._global_guidance_path.points:
            logger.warning("No global guidance path points")
            return np.array(
                [
                    position.x,
                    position.y,
                    self._goal.x,
                    self._goal.y,
                    self._goal.x,
                    self._goal.y,
                ]
            )

        return np.array(
            [
                position.x,
                position.y,
                self._goal.x,
                self._goal.y,
                self._global_guidance_path.points[0].x,
                self._global_guidance_path.points[0].y,
            ]
        )

    def _calculate_distance_to_goal(self, obs: np.array) -> float:
        pos_x, pos_y, *_ = obs
        return math.dist(self._goal, (pos_x, pos_y))

    def _calculate_reward(
        self, dist_to_goal: float, num_passed_points: int
    ) -> tuple[float, bool]:
        if self._robot.is_collide:
            logger.debug("FAILURE: Robot collided with an obstacle")
            return -5.0, True

        if dist_to_goal < self._min_goal_dist:
            logger.debug(f"SUCCESS: Robot reached goal at {self._goal=}")
            return 5.0, True

        detour_penalty = 0
        if (
            self._global_guidance_path.points
            and self._global_guidance_path.points[0].dist(self._robot.position)
            >= POINT_REACHED_THRESHOLD
        ):
            detour_penalty = -1.0 / self.spec.max_episode_steps
        global_guidance_following_reward = 1.0 * (
            num_passed_points / self._global_guidance_path.original_num_points
        )

        return global_guidance_following_reward + detour_penalty, False

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
        self.goal = random.SystemRandom().choice(available_goals)
        logger.debug(f"{self.goal=}")

    def _publish_goal(self):
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.pose.position.x = self.goal.x
        goal.pose.position.y = self.goal.y
        self._navigation_goal_pub.publish(goal)

    def _wait_for_new_path(self, timeout_sec: int) -> bool:
        total = 0
        while not self._global_guidance_path:
            logger.debug("Waiting for global planner to calculate a new path")
            self._publish_goal()
            rclpy.spin_once(self._node, timeout_sec=1.0)
            total += 1
            if total > timeout_sec:
                logger.warning("Timeout reached while waiting for a new path.")
                return False
        return True

    def _global_planner_callback(self, message: Path):
        if self._global_guidance_path:  # update path only at the start of the episode
            return

        points = [
            Point(pose_stamped.pose.position.x, pose_stamped.pose.position.y)
            for pose_stamped in message.poses
        ]

        if not points:
            return

        self._analyze_path(points)

        if points[0].dist(Point(0, 0)) > MAP_FRAME_POSITION_ERROR_TOLERANCE:
            logger.warning(
                f"Path rejected: First point in the path is not close to the (0, 0): {points[0]}"
            )
            return

        if points[-1].dist(self.goal) > MAP_FRAME_POSITION_ERROR_TOLERANCE:
            logger.warning(
                f"Path rejected: Last point in the path is not close to the goal ({self.goal}): {points[-1]}"
            )
            return

        self._global_guidance_path = GlobalGuidancePath(points)

    def _analyze_path(self, points: list[Point]):
        logger.debug(f"New path has {len(points)} points")
        logger.debug(f"First point in the path: {points[0]}")
        logger.debug(f"Last point in the path: {points[-1]}")
        dist = 0
        n = len(points)
        for i in range(1, n):
            p1 = (points[i - 1].x, points[i - 1].y)
            p2 = (points[i].x, points[i].y)
            dist += math.dist(p1, p2)
        logger.debug(f"Average distance between points: {dist / n-1} meters")
