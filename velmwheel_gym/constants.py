from velmwheel_gym.types import NavigationDifficulty, Point

# Step time in seconds at real time factor 1.0
BASE_STEP_TIME = 0.050

# Number of lidar rays
TARGET_LIDAR_RAY_COUNT = 45

# Number of coordinates in global guidance observation
GLOBAL_GUIDANCE_OBSERVATION_POINTS = 10

# All coordinates are divided by this factor while being part of an observation
COORDINATES_NORMALIZATION_FACTOR = 20.0

# Maximum range of the LIDAR sensor
LIDAR_MAX_RANGE = 20.0

# Maximum linear velocity of the robot
MAX_LINEAR_VELOCITY = 0.5

_GOAL_REACHED_THRESHOLD = 0.25
_DRIVING_IN_PATH_TOLERANCE = 1.0

# Navigation difficulty levels
NAVIGATION_DIFFICULTIES = [
    NavigationDifficulty(
        goal_reached_threshold=_GOAL_REACHED_THRESHOLD,
        driving_in_path_tolerance=_DRIVING_IN_PATH_TOLERANCE,
        dynamic_obstacles=[],
        dynamic_obstacle_motion=False,
        extend_segment=False,
        starting_rect=(Point(-2.0, 2.0), Point(-2.0, 2.0)),
        goal_rect=(Point(0.0, 0.0), Point(0.0, 0.0)),
        raw_lidar_ray_count=45,
        maneuvers=[],
    ),
    NavigationDifficulty(
        goal_reached_threshold=_GOAL_REACHED_THRESHOLD,
        driving_in_path_tolerance=_DRIVING_IN_PATH_TOLERANCE,
        dynamic_obstacles=[],
        dynamic_obstacle_motion=False,
        extend_segment=False,
        starting_rect=(Point(-2.0, 2.0), Point(-2.0, 2.0)),
        goal_rect=(Point(-2.0, -2.0), Point(2.0, 2.0)),
        raw_lidar_ray_count=45,
        maneuvers=[],
    ),
    NavigationDifficulty(
        goal_reached_threshold=_GOAL_REACHED_THRESHOLD,
        driving_in_path_tolerance=_DRIVING_IN_PATH_TOLERANCE,
        dynamic_obstacles=[],
        dynamic_obstacle_motion=False,
        extend_segment=False,
        starting_rect=(Point(-4.0, 4.0), Point(-4.0, 4.0)),
        goal_rect=(Point(-4.0, 4.0), Point(-4.0, 4.0)),
        raw_lidar_ray_count=45,
        maneuvers=[],
    ),
    NavigationDifficulty(
        goal_reached_threshold=_GOAL_REACHED_THRESHOLD,
        driving_in_path_tolerance=_DRIVING_IN_PATH_TOLERANCE,
        dynamic_obstacles=[
            (0.0, 3.0),
            (0.0, -3.0),
            (3.0, 0.0),
            (-3.0, 0.0),
        ],
        dynamic_obstacle_motion=False,
        extend_segment=False,
        starting_rect=(Point(-4.0, 4.0), Point(-4.0, 4.0)),
        goal_rect=(Point(-4.0, 4.0), Point(-4.0, 4.0)),
        raw_lidar_ray_count=45,
        maneuvers=[],
    ),
    NavigationDifficulty(
        goal_reached_threshold=_GOAL_REACHED_THRESHOLD,
        driving_in_path_tolerance=_DRIVING_IN_PATH_TOLERANCE,
        dynamic_obstacles=[
            (0.0, 3.0),
            (0.0, -3.0),
            (3.0, 0.0),
            (-3.0, 0.0),
            (0.0, 0.0),
        ],
        dynamic_obstacle_motion=False,
        extend_segment=False,
        starting_rect=(Point(-4.0, 4.0), Point(-4.0, 4.0)),
        goal_rect=(Point(-4.0, 4.0), Point(-4.0, 4.0)),
        raw_lidar_ray_count=45,
        maneuvers=[],
    ),
    NavigationDifficulty(
        goal_reached_threshold=_GOAL_REACHED_THRESHOLD,
        driving_in_path_tolerance=_DRIVING_IN_PATH_TOLERANCE,
        dynamic_obstacles=[
            (0.0, 3.0),
            (0.0, -3.0),
            (3.0, 0.0),
            (-3.0, 0.0),
            (0.0, 0.0),
        ],
        dynamic_obstacle_motion=False,
        extend_segment=False,
        starting_rect=(Point(-4.0, 4.0), Point(-8.0, 4.0)),
        goal_rect=(Point(-4.0, 4.0), Point(-8.0, 4.0)),
        raw_lidar_ray_count=180,
        maneuvers=[],
    ),
    NavigationDifficulty(
        goal_reached_threshold=_GOAL_REACHED_THRESHOLD,
        driving_in_path_tolerance=_DRIVING_IN_PATH_TOLERANCE,
        dynamic_obstacles=[
            (0.0, 3.0),
            (0.0, -3.0),
            (3.0, 0.0),
            (-3.0, 0.0),
            (0.0, 0.0),
        ],
        dynamic_obstacle_motion=False,
        extend_segment=True,
        starting_rect=(Point(-4.0, 4.0), Point(-8.0, 4.0)),
        goal_rect=(Point(-4.0, 4.0), Point(-8.0, 4.0)),
        raw_lidar_ray_count=180,
        maneuvers=[],
    ),
    NavigationDifficulty(
        goal_reached_threshold=_GOAL_REACHED_THRESHOLD,
        driving_in_path_tolerance=_DRIVING_IN_PATH_TOLERANCE,
        dynamic_obstacles=[
            (0.0, 3.0),
            (0.0, -3.0),
            (3.0, 0.0),
            (-3.0, 0.0),
            (0.0, 0.0),
        ],
        dynamic_obstacle_motion=False,
        extend_segment=True,
        starting_rect=(Point(-9.0, 9.0), Point(-9.0, 9.0)),
        goal_rect=(Point(-9.0, 9.0), Point(-9.0, 9.0)),
        raw_lidar_ray_count=180,
        maneuvers=[],
    ),
    # NavigationDifficulty(
    #     goal_reached_threshold=_GOAL_REACHED_THRESHOLD,
    #     driving_in_path_tolerance=_DRIVING_IN_PATH_TOLERANCE,
    #     dynamic_obstacles=[
    #         (0.0, 0.0),
    #         (-2.0, -5.0),
    #         (-3.0, -5.0),
    #         (-4.0, -5.0),
    #     ],
    #     dynamic_obstacle_motion=False,
    #     extend_segment=True,
    #     starting_rect=(Point(-9.0, 9.0), Point(-9.0, 9.0)),
    #     goal_rect=(Point(-9.0, 9.0), Point(-9.0, 9.0)),
    #     raw_lidar_ray_count=180,
    #     maneuvers=[
    #         # obstacles
    #         (Point(0.0, -2.0), Point(0.0, 2.0)),
    #         (Point(0.0, 2.0), Point(0.0, -2.0)),
    #         (Point(-2.0, 0.0), Point(2.0, 0.0)),
    #         (Point(2.0, 0.0), Point(-2.0, 0.0)),
    #         (Point(2.0, 2.0), Point(-2.0, -2.0)),
    #         (Point(-2.0, -2.0), Point(2.0, 2.0)),
    #         (Point(-2.0, 2.0), Point(2.0, -2.0)),
    #         (Point(2.0, -2.0), Point(-2.0, 2.0)),
    #         (Point(2.0, 2.0), Point(-2.0, -2.0)),
    #         (Point(-2.0, -2.0), Point(2.0, 2.0)),
    #         (Point(0.0, -5.0), Point(-5.0, -5.0)),
    #         (Point(-5.0, -5.0), Point(0.0, -5.0)),
    #         # turns
    #         (Point(7.0, 4.0), Point(3.0, 7.0)),
    #         (Point(3.0, 7.0), Point(7.0, 4.0)),
    #         (Point(-7.0, 4.0), Point(-3.0, 7.0)),
    #         (Point(-3.0, 7.0), Point(-7.0, 4.0)),
    #         (Point(-4.0, -7.0), Point(-7.0, -4.0)),
    #         (Point(-7.0, -4.0), Point(-4.0, -7.0)),
    #         # gates
    #         (Point(1.0, -7.0), Point(1.0, -5.0)),
    #         (Point(5.0, -7.0), Point(5.0, -5.0)),
    #         (Point(1.0, -5.0), Point(1.0, -7.0)),
    #         (Point(5.0, -5.0), Point(5.0, -7.0)),
    #         (Point(-7.0, -4.0), Point(-4.0, -4.0)),
    #         (Point(-4.0, -4.0), Point(-7.0, -4.0)),
    #         (Point(-4.0, 2.0), Point(-7.0, 2.0)),
    #         (Point(-7.0, 2.0), Point(-4.0, 2.0)),
    #     ],
    # ),
]

STATS_BUFFER_SIZE = 50
