import logging
import math
import time
from copy import deepcopy
from typing import Optional

import gymnasium as gym
import matplotlib.pyplot as plt
import numpy as np
import rclpy
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
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
)
from velmwheel_gym.reward import calculate_reward
from velmwheel_gym.types import Point

logger = logging.getLogger(__name__)

START_SIM_TOPIC = "/start_sim"
STOP_SIM_TOPIC = "/stop_sim"
RESTART_SIM_TOPIC = "/restart_sim"
RESET_WORLD_TOPIC = "/reset_world"
SPAWN_ENTITY_TOPIC = "/spawn_entity"
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
        self._fig = None
        self._ax = None

        self.action_space = gym.spaces.Box(
            low=-1.0, high=1.0, shape=(2,), dtype=np.float64
        )
        self.observation_space = gym.spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(4 + 2 * GLOBAL_GUIDANCE_OBSERVATION_POINTS + LIDAR_DATA_SIZE,),
            dtype=np.float64,
        )
        self._start_position_and_goal_generator = StartPositionAndGoalGenerator()
        self._point_reached_threshold: float = kwargs["point_reached_threshold"]
        self._real_time_factor: float = kwargs["real_time_factor"]
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

        self._spawn_entity_srv = create_ros_service_client(
            self._node, SpawnEntity, SPAWN_ENTITY_TOPIC
        )

        self._delete_entity_srv = create_ros_service_client(
            self._node, DeleteEntity, DELETE_ENTITY_TOPIC
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
    def robot_position(self) -> Point:
        return self._robot.position

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

    def step(self, action):
        self._steps += 1
        step_time = BASE_STEP_TIME / self.real_time_factor
        start = time.time()

        self._robot.move(action)
        if not self._robot.update():
            logger.warning("Robot update failed. Restarting the simulation.")
            call_service(self._stop_sim_srv)
            self._simulation_reinit()
            return None, 0, True, False, {}

        obs = self._observe()

        num_passed_points = self._global_guidance_path.update(
            self._robot.position, self._point_reached_threshold
        )

        reward, terminated = calculate_reward(
            self._robot.position,
            self.goal,
            self._robot.is_collide,
            self._point_reached_threshold,
            num_passed_points,
            self._global_guidance_path,
            self.max_episode_steps,
            self._steps,
        )
        self._episode_reward += reward

        if terminated:
            if self._robot.is_collide:
                logger.debug("FAILURE: Robot collided with an obstacle")
                # After collision, we should use the navigation stack to get a new path,
                # so we will be able to detect if collision has broken odom -> map frame transformation.
                self._use_cache = False
            else:
                logger.debug(f"SUCCESS: Robot reached goal at {self.goal=}")
                self._start_position_and_goal_generator.register_goal_reached()

        logger.trace(
            f"episode_reward={self._episode_reward} {reward=} dist_to_goal={self.goal.dist(self._robot.position)} goal={self.goal} position={self._robot.position} current_time={time.time()} position_tstamp={self._robot.position_tstamp} lidar_tstamp={self._robot.lidar_tstamp}"
        )

        end = time.time()
        elapsed = end - start
        if elapsed < step_time:
            time.sleep(step_time - elapsed)
        else:
            logger.warning(f"Step time ({step_time}) exceeded: {elapsed}")

        if self.render_mode == "human":
            self.render()

        return obs, reward, terminated, False, {}

    # pylint: disable=unused-argument
    def reset(self, seed=None, options=None):
        self._steps = 0
        self._episode_reward = 0.0
        call_service(self._reset_world_srv)

        if options and "goal" in options and "starting_position" in options:
            self._start_position_and_goal_generator.set(
                options["starting_position"], options["goal"]
            )
        else:
            self._start_position_and_goal_generator.generate_next()
        self._publish_goal()

        self._robot.reset(self.starting_position)

        if (
            self._use_cache
            and (self.starting_position, self.goal) in self._global_guidance_path_cache
        ):
            self._get_global_guidance_path_from_cache()
        else:
            self._get_global_guidance_path_from_ros_navigation_stack()
            self._use_cache = True

        self._spawn_random_obstacles()

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
        px = [p.x for p in self._global_guidance_path.points]
        py = [p.y for p in self._global_guidance_path.points]
        self._ax.plot(px, py, ".g")
        plt.pause(0.02)
        self._fig.canvas.draw()

    def _spawn_random_obstacles(self, n: int = 10):
        for i in range(n):
            self._delete_entity_srv.request = DeleteEntity.Request()
            self._delete_entity_srv.request.name = "box" + str(i)
            call_service(self._delete_entity_srv)

        logger.debug(f"Spawning {n} random obstacles")
        for i in range(n):
            while True:
                x = np.random.uniform(-5, 5)
                y = np.random.uniform(-5, 5)
                if (
                    Point(x, y).dist(self.goal) > 1.0
                    and Point(x, y).dist(self.starting_position) > 1.0
                ):
                    break

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

        obs.extend(
            get_n_points_evenly_spaced_on_path(
                self._global_guidance_path, 10, [self.goal.x, self.goal.y]
            )
        )

        # normalize position and goal coordinates
        obs = [o / COORDINATES_NORMALIZATION_FACTOR for o in obs]

        obs.extend(self._robot.normalized_lidar_data)

        return np.clip(np.array(obs), -1.0, 1.0)

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
