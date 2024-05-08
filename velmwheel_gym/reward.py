import logging

from velmwheel_gym.constants import POINT_REACHED_THRESHOLD
from velmwheel_gym.gazebo_env.robot import VelmwheelRobot
from velmwheel_gym.global_guidance_path import GlobalGuidancePath
from velmwheel_gym.types import Point

logger = logging.getLogger(__name__)


COLLISION_PENALTY = -0.9
DETOUR_PENALTY = -0.1
SUCCESS_REWARD = 0.5
PATH_FOLLOWING_REWARD = 0.5


def calculate_reward(
    robot_position: Point,
    goal: Point,
    is_robot_collide: bool,
    goal_reached_threshold: float,
    num_passed_points: int,
    global_guidance_path: GlobalGuidancePath,
    max_episode_steps: int,
    steps: int,
) -> tuple[float, bool]:
    """Calculates the reward.

    Parameters
    ----------
    robot : VelmwheelRobot
        The robot object.
    goal : Point
        The robot's goal.
    goal_reached_threshold : float
        The threshold for the robot to reach the goal.
    num_passed_points : int
        The number of global guidance points passed by the robot in current step.
    global_guidance_path : GlobalGuidancePath
        The global guidance path.
    max_episode_steps : int
        The maximum number of steps in an episode.
    steps : int
        The current step number.

    Returns
    -------
    tuple[float, bool]
        The reward and a boolean indicating if the episode is done.
    """
    if is_robot_collide:
        remaining_detour_penalty = (
            (max_episode_steps - steps) * DETOUR_PENALTY / max_episode_steps
        )
        return COLLISION_PENALTY + remaining_detour_penalty, True

    if robot_position.dist(goal) < goal_reached_threshold:
        return SUCCESS_REWARD, True

    if num_passed_points > 0:
        return (
            PATH_FOLLOWING_REWARD
            * (num_passed_points / global_guidance_path.original_num_points),
            False,
        )

    if _is_detoured_from_global_guidance_path(global_guidance_path, robot_position):
        return DETOUR_PENALTY / max_episode_steps, False

    return 0.0, False


def _is_detoured_from_global_guidance_path(
    global_guidance_path: GlobalGuidancePath, robot_position: Point
) -> bool:
    if not global_guidance_path.points:
        return False
    return (
        global_guidance_path.points[0].dist(robot_position) >= POINT_REACHED_THRESHOLD
    )
