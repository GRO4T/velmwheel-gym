import copy
import logging
import math
import os
import pickle
import time
from copy import deepcopy

import numpy as np
import rclpy
from gazebo_msgs.srv import DeleteEntity, SetEntityState, SpawnEntity
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from rclpy.qos import qos_profile_system_default
from std_srvs.srv import Empty, Trigger

from velmwheel_gym.base_env import VelmwheelBaseEnv
from velmwheel_gym.constants import BASE_STEP_TIME
from velmwheel_gym.gazebo_env.renderer import GazeboDebugRenderer
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
from velmwheel_gym.utils import angle_between_robot_and_goal, interpolate_coordinates

logger = logging.getLogger(__name__)

START_SIM_TOPIC = "/start_sim"
STOP_SIM_TOPIC = "/stop_sim"
RESTART_SIM_TOPIC = "/restart_sim"
RESET_WORLD_TOPIC = "/reset_world"
SPAWN_ENTITY_TOPIC = "/spawn_entity"
SET_ENTITY_STATE_TOPIC = "/set_entity_state"
DELETE_ENTITY_TOPIC = "/delete_entity"
NAVIGATION_GOAL_TOPIC = "/goal_pose"
NAVIGATION_INITIAL_POSE_TOPIC = "/initialpose"
GLOBAL_PLANNER_PATH_TOPIC = "/plan"

WAIT_FOR_NEW_PATH_TIMEOUT_SEC = 30
MAP_FRAME_POSITION_ERROR_TOLERANCE = 0.5


class VelmwheelGazeboEnv(VelmwheelBaseEnv):
    def __init__(self, **kwargs):
        logger.debug("Creating VelmwheelEnv")
        super().__init__(**kwargs)

        self._real_time_factor: float = kwargs["real_time_factor"]
        self.__global_path: GlobalGuidancePath = None
        self.__global_path_segment: GlobalGuidancePath = None
        if os.path.exists("state/nav2_cache.pkl"):
            # with open("state/nav2_cache.pkl", "rb") as f:
            #     self._global_path_cache = pickle.load(f)
            # logger.debug("Loaded global path cache")
            self._global_path_cache: dict[tuple[Point, Point], GlobalGuidancePath] = {}
        else:
            self._global_path_cache: dict[tuple[Point, Point], GlobalGuidancePath] = {}
        self._is_obstacles_spawned = False
        self._last_step_time = time.time()
        self._dynamic_obstacles_start_positions = None
        self._dynamic_obstacles_target_positions = None
        self._time = 0.0
        self._delta = 0.001

        self._simulation_init()
        self._robot = VelmwheelRobot()
        self.__start_position_and_goal_generator = StartPositionAndGoalGenerator(
            self._difficulty
        )
        self._renderer = GazeboDebugRenderer(window_title=self._name)

        logger.debug("VelmwheelEnv created")

    def _simulation_init(self):
        rclpy.init()
        self._node = rclpy.create_node(self.__class__.__name__)

        self._is_sim_running_srv = create_ros_service_client(
            self._node, Trigger, "/is_sim_running"
        )
        self._start_sim_srv = create_ros_service_client(
            self._node, Empty, START_SIM_TOPIC
        )
        self._stop_sim_srv = create_ros_service_client(
            self._node, Empty, STOP_SIM_TOPIC
        )
        self._restart_sim_srv = create_ros_service_client(
            self._node, Empty, RESTART_SIM_TOPIC
        )

        is_sim_running = call_service(self._is_sim_running_srv).success
        if not is_sim_running:
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

        self._nav_initial_position_pub = self._node.create_publisher(
            PoseWithCovarianceStamped,
            NAVIGATION_INITIAL_POSE_TOPIC,
            qos_profile=qos_profile_system_default,
        )

        self._navigation_plan_sub = self._node.create_subscription(
            Path,
            GLOBAL_PLANNER_PATH_TOPIC,
            self._global_planner_callback,
            qos_profile=qos_profile_system_default,
        )

    def _simulation_reinit(self):
        call_service(self._stop_sim_srv)
        self._is_obstacles_spawned = False
        self._node.destroy_node()
        rclpy.shutdown()
        self._simulation_init()
        self._robot = VelmwheelRobot()
        if not self._robot.reset(self.starting_position):
            logger.warning("Robot reset failed. Restarting the simulation.")
            self._simulation_reinit()

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

    @property
    def _start_position_and_goal_generator(self) -> StartPositionAndGoalGenerator:
        return self.__start_position_and_goal_generator

    @property
    def _global_path(self) -> GlobalGuidancePath:
        return self.__global_path

    @_global_path.setter
    def _global_path(self, value: GlobalGuidancePath):
        self.__global_path = value

    @property
    def _global_path_segment(self) -> GlobalGuidancePath:
        return self.__global_path_segment

    @_global_path_segment.setter
    def _global_path_segment(self, value: GlobalGuidancePath):
        self.__global_path_segment = value

    @property
    def robot_velocity(self) -> np.array:
        return np.array([0.0, 0.0])  # TODO: Implement this

    @property
    def robot_orientation(self) -> float:
        return self._robot.theta

    @property
    def obstacles(self) -> list[tuple[float, float]]:
        return np.array([]), np.array([])  # TODO: Implement this

    def step(self, action):
        self.prev_robot_position = self.robot_position
        self._steps += 1
        step_time = BASE_STEP_TIME / self.real_time_factor

        self._vx = 0.5 * action[0]
        self._vy = 0.5 * action[1]

        self._robot.move(action)

        if self._time > 1.0 or self._time < 0.0:
            self._delta = -self._delta
        self._time += self._delta
        if self._difficulty.dynamic_obstacle_motion:
            for i in range(self._difficulty.dynamic_obstacle_count):
                x, y = (
                    self._dynamic_obstacles_start_positions[i].x,
                    self._dynamic_obstacles_start_positions[i].y,
                )
                tx, ty = (
                    self._dynamic_obstacles_target_positions[i].x,
                    self._dynamic_obstacles_target_positions[i].y,
                )
                new_x, new_y = interpolate_coordinates(x, y, tx, ty, self._time)
                req = self._set_entity_state_srv.request
                req.state.name = "box" + str(i)
                req.state.pose.position.x = new_x
                req.state.pose.position.y = new_y
                req.state.pose.position.z = 0.0
                req.state.pose.orientation.x = 0.0
                req.state.pose.orientation.y = 0.0
                req.state.pose.orientation.z = 0.0
                req.state.pose.orientation.w = 1.0
                call_service(self._set_entity_state_srv)

        if not self._robot.update():
            logger.warning("Robot update failed. Restarting the simulation.")
            self._simulation_reinit()
            return None, 0, True, False, {}

        num_passed_points = self._global_path_segment.update(self._robot.position)

        alpha = angle_between_robot_and_goal(
            self.robot_position, self.sub_goal, self._robot.theta
        )
        if alpha > np.pi / 2:
            alpha = np.pi - alpha

        min_obstacle_dist = min(self._robot.lidar_data)

        success, reward, terminated = calculate_reward(
            self._variant,
            self.is_final_goal,
            self.prev_robot_position,
            self._robot.position,
            alpha,
            self.sub_goal,
            self._robot.is_collide,
            self._difficulty,
            num_passed_points,
            self._global_path_segment,
            self.max_episode_steps,
            self._steps,
            min_obstacle_dist,
            action[2],
        )
        self._metrics.episode_reward += reward

        if terminated:
            self._register_episode_terminated(success)
            if (
                self._get_current_level() < self._get_max_level()
                and self._metrics.global_success_rate > 0.7
            ):
                self._advance_to_next_level()

        logger.trace(
            f"{reward=} dist_to_goal={self.goal.dist(self._robot.position)} goal={self.goal} position={self._robot.position} current_time={time.time()} position_tstamp={self._robot.position_tstamp} lidar_tstamp={self._robot.lidar_tstamp}"
        )

        if self.render_mode == "human":
            self.render()

        if self._training_mode:
            self._metrics.export(self._get_current_level(), terminated)

        end = time.time()
        elapsed = end - self._last_step_time
        self._last_step_time = end
        if elapsed < step_time:
            time.sleep(step_time - elapsed)
        else:
            logger.warning(f"Step time ({step_time}) exceeded: {elapsed}")

        obs = self._observe()

        info = {}
        if success and not self.is_final_goal:
            info["status"] = "segment_reached"
        elif self._steps == self.max_episode_steps:
            info["status"] = "max_steps_reached"

        return obs, reward, terminated, False, info

    # pylint: disable=unused-argument
    def reset(self, seed=None, options=None):
        self._vx = 0.0
        self._vy = 0.0
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
        self._metrics.register_local_episode_start()

        if self._generate_next_goal:
            self._metrics.register_global_episode_start()
            self._generate_next_goal = False
            self._delete_entity_srv.request = DeleteEntity.Request()
            self._delete_entity_srv.request.name = "goal"
            call_service(self._delete_entity_srv)
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

            while not self._robot.reset(self.starting_position):
                logger.warning("Robot reset failed. Restarting the simulation.")
                self._simulation_reinit()

            self._get_global_path()
            self._spawn_random_obstacles()

            self._robot.position_tstamp = time.time()
            self._robot.lidar_tstamp = time.time()

            if self.render_mode == "human":
                self.render()

        self.prev_robot_position = self.robot_position

        return self._observe(), {}

    def close(self):
        logger.info("Closing " + self.__class__.__name__ + " environment.")
        self._robot.stop()
        self._node.destroy_node()
        rclpy.shutdown()
        self._renderer.close()

    def render(self, mode="human"):
        self._renderer.render(
            self.robot_position,
            self.goal,
            self._robot.lidar_pointcloud_raw,
            self._global_path.points,
            self._global_path_segment.points,
        )

    def _spawn_random_obstacles(self):
        n = len(self._difficulty.dynamic_obstacles)

        self._dynamic_obstacles_start_positions = [None] * n
        self._dynamic_obstacles_target_positions = [None] * n
        for i in range(n):
            x = self._difficulty.dynamic_obstacles[i][0]
            y = self._difficulty.dynamic_obstacles[i][1]
            tx = x
            ty = y

            self._dynamic_obstacles_start_positions[i] = Point(x, y)
            self._dynamic_obstacles_target_positions[i] = Point(tx, ty)

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
        if (
            self._training_mode
            and (self.starting_position, self.goal) in self._global_path_cache
        ):
            self._get_global_guidance_path_from_cache()
        else:
            self._get_global_guidance_path_from_ros_navigation_stack()
        self._global_path.points, self._global_path_segment = next_segment(
            self._global_path.points,
            [],
            self.robot_position,
            self._difficulty,
            self._global_path_segment_length,
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
            self._simulation_reinit()

    def _observe(self) -> np.array:
        step_normalized = 2 * self._steps / self.max_episode_steps - 1
        obs = [
            step_normalized,
            self._robot.theta,
            self.sub_goal.x,
            self.sub_goal.y,
        ]

        if self._variant == "EasierFollowing":
            obs.append(1.0 if self.is_final_goal else -1.0)

        obs.extend(
            get_n_points_evenly_spaced_on_path(
                self._global_path_segment.points,
                10,
                [self.sub_goal.x, self.sub_goal.y],
            )
        )
        ranges = [
            min(self._robot.lidar_data[i : i + 12])
            for i in range(0, len(self._robot.lidar_data), 12)
        ]
        obs.extend(ranges)
        return self._normalize_observation(self._relative_to_robot(np.array(obs)))

    def _publish_goal(self):
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.pose.position.x = self.goal.x
        goal.pose.position.y = self.goal.y
        self._navigation_goal_pub.publish(goal)
        logger.debug(f"Publish goal: {self.goal}")

    def _publish_navigation_initial_pose(self):
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.frame_id = "map"
        initial_pose.pose.pose.position.x = self.robot_position.x
        initial_pose.pose.pose.position.y = self.robot_position.y
        self._nav_initial_position_pub.publish(initial_pose)
        logger.debug(f"Publish initial pose: {self.robot_position}")

    def _publish_goal_marker(self):
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
            self._publish_navigation_initial_pose()
            self._publish_goal()
            rclpy.spin_once(self._node, timeout_sec=5.0)
            end = time.time()
            total += end - start
            if total > timeout_sec:
                logger.warning("Timeout reached while waiting for a new path.")
                return False
        logger.info(f"Path generation took {total} seconds")
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
        # if (
        #     self._training_mode
        #     and (self.starting_position, self.goal) not in self._global_path_cache
        # ):
        # self._global_path_cache[(self.starting_position, self.goal)] = deepcopy(
        #     self._global_path
        # )
        # with open("state/nav2_cache.pkl", "wb") as f:
        #     pickle.dump(self._global_path_cache, f)

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

    def _update_difficulty(self, difficulty: NavigationDifficulty):
        self._difficulty = difficulty
        self._global_path._difficulty = self._difficulty
        self._global_path_segment._difficulty = self._difficulty
