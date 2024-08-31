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

# Navigation difficulty levels
NAVIGATION_DIFFICULTIES = [
    NavigationDifficulty(
        goal_reached_threshold=0.25,
        driving_in_path_tolerance=1.0,
        dynamic_obstacles=[],
        dynamic_obstacle_motion=False,
        extend_segment=False,
        starting_rect=(Point(-2.0, 2.0), Point(-2.0, 2.0)),
        goal_rect=(Point(0.0, 0.0), Point(0.0, 0.0)),
        raw_lidar_ray_count=45,
        maneuvers=[],
    ),
    NavigationDifficulty(
        goal_reached_threshold=0.25,
        driving_in_path_tolerance=1.0,
        dynamic_obstacles=[],
        dynamic_obstacle_motion=False,
        extend_segment=False,
        starting_rect=(Point(-2.0, 2.0), Point(-2.0, 2.0)),
        goal_rect=(Point(-2.0, -2.0), Point(2.0, 2.0)),
        raw_lidar_ray_count=45,
        maneuvers=[],
    ),
    NavigationDifficulty(
        goal_reached_threshold=0.25,
        driving_in_path_tolerance=1.0,
        dynamic_obstacles=[],
        dynamic_obstacle_motion=False,
        extend_segment=False,
        starting_rect=(Point(-4.0, 4.0), Point(-4.0, 4.0)),
        goal_rect=(Point(-4.0, 4.0), Point(-4.0, 4.0)),
        raw_lidar_ray_count=45,
        maneuvers=[],
    ),
    NavigationDifficulty(
        goal_reached_threshold=0.25,
        driving_in_path_tolerance=1.0,
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
        goal_reached_threshold=0.25,
        driving_in_path_tolerance=1.0,
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
        goal_reached_threshold=0.25,
        driving_in_path_tolerance=1.0,
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
        goal_reached_threshold=0.25,
        driving_in_path_tolerance=1.0,
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
        goal_reached_threshold=0.25,
        driving_in_path_tolerance=1.0,
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
    #     goal_reached_threshold=0.25,
    #     driving_in_path_tolerance=1.0,
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
    #         (Point(-3.5, -2.0), Point(-3.5, 0.75)),
    #         # (Point(-8.0, -8.0), Point(3.0, -9.0)),
    #         (Point(0.0, -2.0), Point(0.0, 2.0)),
    #         (Point(0.0, -5.0), Point(-5.0, -5.0)),
    #         (Point(8.0, 1.0), Point(3.0, 1.0)),
    #     ],
    # ),
]

STATS_BUFFER_SIZE = 50
