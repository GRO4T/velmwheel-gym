import copy
import logging
import math
import os
import pickle
import time
from copy import deepcopy
from typing import Optional

import gymnasium as gym
import matplotlib.pyplot as plt
import numpy as np
import rclpy
from gazebo_msgs.srv import DeleteEntity, SetEntityState, SpawnEntity
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Path
from rclpy.qos import qos_profile_system_default
from std_srvs.srv import Empty

from velmwheel_gym.constants import (
    BASE_STEP_TIME,
    COORDINATES_NORMALIZATION_FACTOR,
    GLOBAL_GUIDANCE_OBSERVATION_POINTS,
    LIDAR_DATA_SIZE,
)
from velmwheel_gym.gazebo_env.robot import VelmwheelRobot
from velmwheel_gym.gazebo_env.ros_utils import call_service, create_ros_service_client
from velmwheel_gym.gazebo_env.start_position_and_goal_generator import (
    StartPositionAndGoalGenerator,
)
from velmwheel_gym.global_guidance_path import (
    GlobalGuidancePath,
    get_n_points_evenly_spaced_on_path,
    next_segment,
)
from velmwheel_gym.reward import calculate_reward
from velmwheel_gym.types import NavigationDifficulty, Point

logger = logging.getLogger(__name__)

START_SIM_TOPIC = "/start_sim"
STOP_SIM_TOPIC = "/stop_sim"
RESTART_SIM_TOPIC = "/restart_sim"
RESET_WORLD_TOPIC = "/reset_world"
SPAWN_ENTITY_TOPIC = "/spawn_entity"
SET_ENTITY_STATE_TOPIC = "/set_entity_state"
DELETE_ENTITY_TOPIC = "/delete_entity"
NAVIGATION_GOAL_TOPIC = "/goal_pose"
GLOBAL_PLANNER_PATH_TOPIC = "/plan"

WAIT_FOR_NEW_PATH_TIMEOUT_SEC = 15
MAP_FRAME_POSITION_ERROR_TOLERANCE = 0.5


class VelmwheelEnv(gym.Env):
    metadata = {
        "render_modes": ["human"],
        "render_fps": 60,
    }

    def __init__(self, render_mode: Optional[str] = None, **kwargs):
        logger.debug("Creating VelmwheelEnv")
        super().__init__()

        self.render_mode = render_mode
        self._training_mode = kwargs["training_mode"]
        self._fig = None
        self._ax = None

        self.action_space = gym.spaces.Box(
            low=-1.0, high=1.0, shape=(2,), dtype=np.float64
        )
        self.observation_space = gym.spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(3 + 2 * GLOBAL_GUIDANCE_OBSERVATION_POINTS + LIDAR_DATA_SIZE,),
            dtype=np.float64,
        )
        self._start_position_and_goal_generator = StartPositionAndGoalGenerator()
        self._difficulty: NavigationDifficulty = kwargs["difficulty"]
        self._real_time_factor: float = kwargs["real_time_factor"]
        self._global_path: GlobalGuidancePath = None
        self._global_path_segment: GlobalGuidancePath = None
        if os.path.exists("state/nav2_cache.pkl"):
            with open("state/nav2_cache.pkl", "rb") as f:
                self._global_path_cache = pickle.load(f)
            logger.debug("Loaded global path cache")
        else:
            self._global_path_cache: dict[tuple[Point, Point], GlobalGuidancePath] = {}
        self._steps: int = 0
        self._episode_reward: float = 0.0
        self._generate_next_goal = True
        self._is_obstacles_spawned = False
        self._last_step_time = time.time()

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

        self._spawn_entity_srv = create_ros_service_client(
            self._node, SpawnEntity, SPAWN_ENTITY_TOPIC
        )

        self._delete_entity_srv = create_ros_service_client(
            self._node, DeleteEntity, DELETE_ENTITY_TOPIC
        )

        self._set_entity_state_srv = create_ros_service_client(
            self._node, SetEntityState, SET_ENTITY_STATE_TOPIC
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
        self._is_obstacles_spawned = False
        self._node.destroy_node()
        rclpy.shutdown()
        self._simulation_init()
        self._robot = VelmwheelRobot()

    @property
    def robot_position(self) -> Point:
        return self._robot.position

    @property
    def goal(self) -> Point:
        """Robot's navigation goal."""
        return self._start_position_and_goal_generator.goal

    @property
    def sub_goal(self) -> Point:
        goal_x = (
            self._global_path_segment.points[-1].x
            if self._global_path_segment.points
            else self.goal.x
        )
        goal_y = (
            self._global_path_segment.points[-1].y
            if self._global_path_segment.points
            else self.goal.y
        )
        return Point(goal_x, goal_y)

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
    def real_time_factor(self) -> float:
        """Real time factor for the simulation."""
        return self._real_time_factor

    @real_time_factor.setter
    def real_time_factor(self, factor: float):
        self._real_time_factor = factor
        self._robot.real_time_factor = factor

    @property
    def max_episode_steps(self) -> int:
        return self._time_limit_max_episode_steps

    @property
    def is_final_goal(self) -> bool:
        return not self._global_path.points

    def step(self, action):
        self._steps += 1
        step_time = BASE_STEP_TIME / self.real_time_factor

        self._robot.move(action)
        if not self._robot.update():
            logger.warning("Robot update failed. Restarting the simulation.")
            call_service(self._stop_sim_srv)
            self._simulation_reinit()
            return None, 0, True, False, {}

        obs = self._observe()

        num_passed_points = self._global_path_segment.update(self._robot.position)

        success, reward, terminated = calculate_reward(
            self.is_final_goal,
            self._robot.position,
            self.sub_goal,
            self._robot.is_collide,
            self._difficulty,
            num_passed_points,
            self._global_path_segment,
            self.max_episode_steps,
            self._steps,
        )
        self._episode_reward += reward

        if terminated:
            self._generate_next_goal = True
            if success:
                if self.is_final_goal:
                    logger.debug(f"SUCCESS: Robot reached goal at {self.goal=}")
                    self._start_position_and_goal_generator.register_goal_reached()
                else:
                    logger.debug("SUCCESS: Robot reached global path segment")
                    self._global_path.points, self._global_path_segment = next_segment(
                        self._global_path.points,
                        self._global_path_segment.points,
                        self._robot.position,
                        self._difficulty,
                    )
                    self._generate_next_goal = False

        logger.trace(
            f"episode_reward={self._episode_reward} {reward=} dist_to_goal={self.goal.dist(self._robot.position)} goal={self.goal} position={self._robot.position} current_time={time.time()} position_tstamp={self._robot.position_tstamp} lidar_tstamp={self._robot.lidar_tstamp}"
        )

        end = time.time()
        elapsed = end - self._last_step_time
        self._last_step_time = end
        if elapsed < step_time:
            time.sleep(step_time - elapsed)
        else:
            logger.warning(f"Step time ({step_time}) exceeded: {elapsed}")

        if self.render_mode == "human":
            self.render()

        return obs, reward, terminated, False, {}

    # pylint: disable=unused-argument
    def reset(self, seed=None, options=None):
        if self._steps >= self.max_episode_steps:
            if self.is_final_goal:
                logger.debug("Did not reach the final goal in time")
                self._generate_next_goal = True
            else:
                logger.debug("Did not reach global path segment in time")
                (
                    self._global_path.points,
                    self._global_path_segment,
                ) = next_segment(
                    self._global_path.points,
                    self._global_path_segment.points,
                    self.robot_position,
                    self._difficulty,
                )

        self._steps = 0
        self._episode_reward = 0.0

        if self._generate_next_goal:
            call_service(self._reset_world_srv)

            if options and "goal" in options and "starting_position" in options:
                if self._training_mode:
                    self._start_position_and_goal_generator.set(
                        options["starting_position"], options["goal"]
                    )
                else:
                    self._start_position_and_goal_generator._starting_position = (
                        options["starting_position"]
                    )
                    self._start_position_and_goal_generator._goal = options["goal"]
            else:
                self._start_position_and_goal_generator.generate_next()
            self._publish_goal()

            self._robot.reset(self.starting_position)

            self._get_global_path()
            self._spawn_random_obstacles(self._difficulty.dynamic_obstacle_count)

            self._robot.position_tstamp = time.time()
            self._robot.lidar_tstamp = time.time()

            if self.render_mode == "human":
                self.render()

        return self._observe(), {}

    def close(self):
        logger.info("Closing " + self.__class__.__name__ + " environment.")
        self._robot.stop()
        self._node.destroy_node()
        rclpy.shutdown()
        plt.close()

    def render(self, mode="human"):
        if self._fig is None:
            plt.ion()
            self._fig, self._ax = plt.subplots(figsize=(10, 10))

        if self._robot.lidar_pointcloud_raw is None:
            return

        X = [
            point[0] + self.robot_position.x
            for point in self._robot.lidar_pointcloud_raw
        ]
        Y = [
            point[1] + self.robot_position.y
            for point in self._robot.lidar_pointcloud_raw
        ]
        self._ax.clear()
        self._ax.set_xlim((-10, 10))
        self._ax.set_ylim((-10, 10))
        self._ax.invert_yaxis()
        self._ax.invert_xaxis()
        # Plot robot
        robot = plt.Rectangle(
            (self.robot_position.x, self.robot_position.y),
            1.0,
            1.0,
            color="orange",
            fill=True,
        )
        self._ax.add_patch(robot)
        # Plot goal
        goal = plt.Circle((self.goal.x, self.goal.y), 0.25, color="green", fill=True)
        self._ax.add_patch(goal)
        # Plot lidar points
        self._ax.plot(X, Y, "o", markersize=1, color="r")
        # Plot global guidance path
        px = [p.x for p in self._global_path.points]
        py = [p.y for p in self._global_path.points]
        self._ax.plot(px, py, ".g")
        sx = [s.x for s in self._global_path_segment.points]
        sy = [s.y for s in self._global_path_segment.points]
        self._ax.plot(sx, sy, ".y")
        plt.pause(0.02)
        self._fig.canvas.draw()

    def _spawn_random_obstacles(self, n: int = 5):
        logger.debug(f"Spawning {n} random obstacles")
        for i in range(n):
            while True:
                x = np.random.uniform(-9, 9)
                y = np.random.uniform(-9, 9)
                if (
                    Point(x, y).dist(self.goal) > 2.0
                    and Point(x, y).dist(self.starting_position) > 2.0
                ):
                    break

            if not self._is_obstacles_spawned:
                self._spawn_entity_srv.request = SpawnEntity.Request()
                self._spawn_entity_srv.request.name = "box" + str(i)
                self._spawn_entity_srv.request.xml = open("./box.sdf", "r").read()
                self._spawn_entity_srv.request.robot_namespace = "/"
                self._spawn_entity_srv.request.initial_pose = Pose()
                self._spawn_entity_srv.request.initial_pose.position.x = x
                self._spawn_entity_srv.request.initial_pose.position.y = y
                self._spawn_entity_srv.request.initial_pose.position.z = 0.0
                self._spawn_entity_srv.request.reference_frame = "world"
                call_service(self._spawn_entity_srv)
            else:
                req = self._set_entity_state_srv.request
                req.state.name = "box" + str(i)
                req.state.pose.position.x = x
                req.state.pose.position.y = y
                req.state.pose.position.z = 0.0
                req.state.pose.orientation.x = 0.0
                req.state.pose.orientation.y = 0.0
                req.state.pose.orientation.z = 0.0
                req.state.pose.orientation.w = 1.0
                call_service(self._set_entity_state_srv)

        self._is_obstacles_spawned = True

    def _get_global_path(self):
        if (self.starting_position, self.goal) in self._global_path_cache:
            self._get_global_guidance_path_from_cache()
        else:
            self._get_global_guidance_path_from_ros_navigation_stack()
        self._global_path.points, self._global_path_segment = next_segment(
            self._global_path.points, [], self.robot_position, self._difficulty
        )
        self._publish_goal_marker()

    def _get_global_guidance_path_from_cache(self):
        logger.debug(
            f"Using cached global guidance path from {self.starting_position} to {self.goal}"
        )
        self._global_path = copy.deepcopy(
            self._global_path_cache[(self.starting_position, self.goal)]
        )

    def _get_global_guidance_path_from_ros_navigation_stack(self):
        logger.debug(
            f"Waiting for the global guidance path from {self.starting_position} to {self.goal} from the ROS navigation stack"
        )
        self._global_path = None
        while not self._wait_for_new_path(WAIT_FOR_NEW_PATH_TIMEOUT_SEC):
            logger.warning("Simulation in a bad state. Restarting the simulation.")
            call_service(self._stop_sim_srv)
            self._simulation_reinit()
            self._robot.reset(self.starting_position)

    def _observe(self) -> np.array:
        goal_x_relative = self.sub_goal.x - self.robot_position[0]
        goal_y_relative = self.sub_goal.y - self.robot_position[1]
        obs = [
            goal_x_relative,
            goal_y_relative,
        ]

        obs.extend(
            get_n_points_evenly_spaced_on_path(
                self._global_path_segment.points,
                10,
                [goal_x_relative, goal_y_relative],
                self.robot_position,
            )
        )

        # normalize position and goal coordinates
        obs = [o / COORDINATES_NORMALIZATION_FACTOR for o in obs]
        obs.extend(self._robot.normalized_lidar_data)

        return np.array([1.0 if self.is_final_goal else 0.0] + obs)

    def _publish_goal(self):
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.pose.position.x = self.goal.x
        goal.pose.position.y = self.goal.y
        self._navigation_goal_pub.publish(goal)

    def _publish_goal_marker(self):
        self._delete_entity_srv.request = DeleteEntity.Request()
        self._delete_entity_srv.request.name = "goal"
        call_service(self._delete_entity_srv)
        time.sleep(0.001)

        self._spawn_entity_srv.request = SpawnEntity.Request()
        self._spawn_entity_srv.request.name = "goal"
        self._spawn_entity_srv.request.xml = open("./goal_marker.sdf", "r").read()
        self._spawn_entity_srv.request.robot_namespace = "/"
        self._spawn_entity_srv.request.initial_pose = Pose()
        self._spawn_entity_srv.request.initial_pose.position.x = self.goal.x
        self._spawn_entity_srv.request.initial_pose.position.y = self.goal.y
        self._spawn_entity_srv.request.initial_pose.position.z = 0.0
        self._spawn_entity_srv.request.reference_frame = "world"
        call_service(self._spawn_entity_srv)
        self._is_goal_spawned = True

    def _wait_for_new_path(self, timeout_sec: int) -> bool:
        total = 0.0
        while not self._global_path:
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
        if self._global_path:  # update path only at the start of the episode
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

        self._global_path = GlobalGuidancePath(
            self._robot.position, points, self._difficulty
        )
        if (self.starting_position, self.goal) not in self._global_path_cache:
            self._global_path_cache[(self.starting_position, self.goal)] = deepcopy(
                self._global_path
            )
            with open("state/nav2_cache.pkl", "wb") as f:
                pickle.dump(self._global_path_cache, f)

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
