import logging

from velmwheel_gym.global_guidance_path import GlobalGuidancePath
from velmwheel_gym.types import NavigationDifficulty, Point

logger = logging.getLogger(__name__)


COLLISION_PENALTY = -0.9
DETOUR_PENALTY = -0.1
SUCCESS_REWARD = 0.5
PATH_FOLLOWING_REWARD = 0.5


# pylint: disable=too-many-arguments
def calculate_reward(
    is_final_goal: bool,
    robot_position: Point,
    goal: Point,
    is_robot_collide: bool,
    difficulty: NavigationDifficulty,
    num_passed_points: int,
    global_guidance_path: GlobalGuidancePath,
    max_episode_steps: int,
    steps: int,
) -> tuple[bool, float, bool]:
    if is_robot_collide:
        remaining_detour_penalty = (
            (max_episode_steps - steps) * DETOUR_PENALTY / max_episode_steps
        )
        return False, COLLISION_PENALTY + remaining_detour_penalty, True

    threshold = (
        difficulty.goal_reached_threshold
        if is_final_goal
        else difficulty.driving_in_path_tolerance
    )
    if robot_position.dist(goal) < threshold:
        if global_guidance_path.original_num_points > 0:
            global_guidance_left_reward = (
                PATH_FOLLOWING_REWARD
                * len(global_guidance_path.points)
                / global_guidance_path.original_num_points
            )
            return True, SUCCESS_REWARD + global_guidance_left_reward, True
        return True, SUCCESS_REWARD, True
    if num_passed_points > 0:
        return (
            False,
            PATH_FOLLOWING_REWARD
            * (num_passed_points / global_guidance_path.original_num_points),
            False,
        )

    return False, DETOUR_PENALTY / max_episode_steps, False
