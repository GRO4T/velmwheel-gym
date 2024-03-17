import logging
import math
import time
from copy import deepcopy

import gym
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.qos import qos_profile_system_default
from std_srvs.srv import Empty

from velmwheel_gym.constants import BASE_STEP_TIME, LIDAR_DATA_SIZE
from velmwheel_gym.global_guidance_path import (
    POINT_REACHED_THRESHOLD,
    GlobalGuidancePath,
)
from velmwheel_gym.robot import VelmwheelRobot
from velmwheel_gym.start_position_and_goal_generator import (
    StartPositionAndGoalGenerator,
)
from velmwheel_gym.types import Point
from velmwheel_gym.utils import call_service, create_ros_service_client

logger = logging.getLogger(__name__)

START_SIM_TOPIC = "/start_sim"
STOP_SIM_TOPIC = "/stop_sim"
RESTART_SIM_TOPIC = "/restart_sim"
RESET_WORLD_TOPIC = "/reset_world"
NAVIGATION_GOAL_TOPIC = "/goal_pose"
GLOBAL_PLANNER_PATH_TOPIC = "/plan"

WAIT_FOR_NEW_PATH_TIMEOUT_SEC = 15
MAP_FRAME_POSITION_ERROR_TOLERANCE = 0.5

COLLISION_PENALTY = -5.0
DETOUR_PENALTY = -1.0
SUCCESS_REWARD = 5.0
PATH_FOLLOWING_REWARD = 5.0


class VelmwheelEnv(gym.Env):
    def __init__(self):
        logger.debug("Creating VelmwheelEnv")
        super().__init__()

        self.action_space = gym.spaces.Box(
            low=-1.0, high=1.0, shape=(2,), dtype=np.float64
        )
        self.observation_space = gym.spaces.Box(
            low=-100.0, high=100.0, shape=(6 + LIDAR_DATA_SIZE,), dtype=np.float64
        )
        self._start_position_and_goal_generator = StartPositionAndGoalGenerator()
        self._min_goal_dist: float = 0.0
        self._real_time_factor: float = 1.0
        self._global_guidance_path: GlobalGuidancePath = None
        self._global_guidance_path_cache: dict[
            tuple[Point, Point], GlobalGuidancePath
        ] = {}
        self._use_cache: bool = False
        self._steps: int = 0
        self._episode_reward: float = 0.0

        self._simulation_init()
        self._robot = VelmwheelRobot()

        logger.debug("VelmwheelEnv created")

    def _simulation_init(self):
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

        self._reset_world_srv = create_ros_service_client(
            self._node, Empty, RESET_WORLD_TOPIC
        )

        self._navigation_goal_pub = self._node.create_publisher(
            PoseStamped,
            NAVIGATION_GOAL_TOPIC,
            qos_profile=qos_profile_system_default,
        )

        self._navigation_plan_sub = self._node.create_subscription(
            Path,
            GLOBAL_PLANNER_PATH_TOPIC,
            self._global_planner_callback,
            qos_profile=qos_profile_system_default,
        )

    def _simulation_reinit(self):
        self._node.destroy_node()
        rclpy.shutdown()
        self._simulation_init()
        self._robot = VelmwheelRobot()

    @property
    def goal(self) -> Point:
        """Robot's navigation goal."""
        return self._start_position_and_goal_generator.goal

    @goal.setter
    def goal(self, point: Point):
        self._start_position_and_goal_generator.goal = point

    @property
    def starting_position(self) -> Point:
        """Robot's starting position."""
        return self._start_position_and_goal_generator.starting_position

    @starting_position.setter
    def starting_position(self, point: Point):
        self._start_position_and_goal_generator.starting_position = point

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
        self._steps += 1
        step_time = BASE_STEP_TIME / self.real_time_factor
        start = time.time()

        self._robot.move(action)
        if not self._robot.update():
            logger.warning("Robot update failed. Restarting the simulation.")
            call_service(self._stop_sim_srv)
            self._simulation_reinit()
            return None, 0, True, {}

        obs = self._observe()

        dist_to_goal = self._calculate_distance_to_goal(obs)
        num_passed_points = self._global_guidance_path.update(self._robot.position)

        reward, done = self._calculate_reward(dist_to_goal, num_passed_points)
        info = {}
        self._episode_reward += reward

        logger.trace(
            f"episode_reward={self._episode_reward} {reward=} {dist_to_goal=} goal={self.goal} position={self._robot.position} current_time={time.time()} position_tstamp={self._robot.position_tstamp} lidar_tstamp={self._robot.lidar_tstamp}"
        )

        end = time.time()
        elapsed = end - start
        if elapsed < step_time:
            time.sleep(step_time - elapsed)
        else:
            logger.warning(f"Step time ({step_time}) exceeded: {elapsed}")

        return obs, reward, done, info

    def reset(self):
        self._steps = 0
        self._episode_reward = 0.0
        call_service(self._reset_world_srv)
        if not self.goal or not self.starting_position:
            self._start_position_and_goal_generator.generate_next()
        self._robot.reset(self.starting_position)

        if (
            self._use_cache
            and (self.starting_position, self.goal) in self._global_guidance_path_cache
        ):
            self._get_global_guidance_path_from_cache()
        else:
            self._get_global_guidance_path_from_ros_navigation_stack()
            self._use_cache = True

        self._robot.position_tstamp = time.time()
        self._robot.lidar_tstamp = time.time()

        return self._observe()

    def close(self):
        logger.info("Closing " + self.__class__.__name__ + " environment.")
        self._robot.stop()
        self._node.destroy_node()
        rclpy.shutdown()

    def _get_global_guidance_path_from_cache(self):
        logger.debug(
            f"Using cached global guidance path from {self.starting_position} to {self.goal}"
        )
        self._global_guidance_path = self._global_guidance_path_cache[
            (self.starting_position, self.goal)
        ]

    def _get_global_guidance_path_from_ros_navigation_stack(self):
        logger.debug(
            f"Waiting for the global guidance path from {self.starting_position} to {self.goal} from the ROS navigation stack"
        )
        self._global_guidance_path = None
        while not self._wait_for_new_path(WAIT_FOR_NEW_PATH_TIMEOUT_SEC):
            logger.warning("Simulation in a bad state. Restarting the simulation.")
            call_service(self._stop_sim_srv)
            self._simulation_reinit()
            self._robot.reset(self.starting_position)

    def _observe(self) -> np.array:
        position = self._robot.position

        obs = [position.x, position.y, self.goal.x, self.goal.y]

        if self._global_guidance_path.points:
            obs.extend(
                [
                    self._global_guidance_path.points[0].x,
                    self._global_guidance_path.points[0].y,
                ]
            )
        else:
            logger.warning("No global guidance path points")
            obs.extend([self.goal.x, self.goal.y])

        obs.extend(self._robot.lidar_data)

        return obs

    def _calculate_distance_to_goal(self, obs: np.array) -> float:
        pos_x, pos_y, *_ = obs
        return math.dist(self.goal, (pos_x, pos_y))

    def _calculate_reward(
        self, dist_to_goal: float, num_passed_points: int
    ) -> tuple[float, bool]:
        if self._robot.is_collide:
            logger.debug("FAILURE: Robot collided with an obstacle")
            # After collision, we should use the navigation stack to get a new path,
            # so we will be able to detect if collision has broken odom -> map frame transformation.
            self._use_cache = False
            return self._calculate_collision_penalty(), True

        if self._is_goal_reached(dist_to_goal):
            logger.debug(f"SUCCESS: Robot reached goal at {self.goal=}")
            self._start_position_and_goal_generator.register_goal_reached()
            self._start_position_and_goal_generator.generate_next()
            return SUCCESS_REWARD, True

        if num_passed_points > 0:
            return (
                self._calculate_global_guidance_following_reward(num_passed_points),
                False,
            )

        if self._is_detoured_from_global_guidance_path():
            return DETOUR_PENALTY / self.spec.max_episode_steps, False

        return 0.0, False

    def _calculate_collision_penalty(self) -> float:
        remaining_detour_penalty = (
            (self.spec.max_episode_steps - self._steps)
            * DETOUR_PENALTY
            / self.spec.max_episode_steps
        )
        return COLLISION_PENALTY + remaining_detour_penalty

    def _is_goal_reached(self, dist_to_goal: float) -> bool:
        return dist_to_goal < self._min_goal_dist

    def _is_detoured_from_global_guidance_path(self) -> bool:
        if not self._global_guidance_path.points:
            return False
        return (
            self._global_guidance_path.points[0].dist(self._robot.position)
            >= POINT_REACHED_THRESHOLD
        )

    def _calculate_global_guidance_following_reward(
        self, num_passed_points: int
    ) -> float:
        return PATH_FOLLOWING_REWARD * (
            num_passed_points / self._global_guidance_path.original_num_points
        )

    def _publish_goal(self):
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.pose.position.x = self.goal.x
        goal.pose.position.y = self.goal.y
        self._navigation_goal_pub.publish(goal)

    def _wait_for_new_path(self, timeout_sec: int) -> bool:
        total = 0.0
        while not self._global_guidance_path:
            start = time.time()
            self._publish_goal()
            rclpy.spin_once(self._node, timeout_sec=1.0)
            end = time.time()
            total += end - start
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

        if points[0].dist(self.starting_position) > MAP_FRAME_POSITION_ERROR_TOLERANCE:
            logger.warning(
                f"Path rejected: First point in the path is not close to the starting position ({self.starting_position}): {points[0]}"
            )
            return

        if points[-1].dist(self.goal) > MAP_FRAME_POSITION_ERROR_TOLERANCE:
            logger.warning(
                f"Path rejected: Last point in the path is not close to the goal ({self.goal}): {points[-1]}"  # pylint: disable=line-too-long
            )
            return

        self._global_guidance_path = GlobalGuidancePath(self._robot.position, points)
        if (self.starting_position, self.goal) not in self._global_guidance_path_cache:
            self._global_guidance_path_cache[
                (self.starting_position, self.goal)
            ] = deepcopy(self._global_guidance_path)

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
