from velmwheel_gym.types import NavigationDifficulty

# Step time in seconds at real time factor 1.0
BASE_STEP_TIME = 0.050

# Number of lidar rays
LIDAR_DATA_SIZE = 90

# Number of coordinates in global guidance observation
GLOBAL_GUIDANCE_OBSERVATION_POINTS = 10

# All coordinates are divided by this factor while being part of an observation
COORDINATES_NORMALIZATION_FACTOR = 20.0

# Maximum range of the LIDAR sensor
LIDAR_MAX_RANGE = 20.0

# Navigation difficulty levels
NAVIGATION_DIFFICULTIES = [
    NavigationDifficulty(
        goal_reached_threshold=0.25,
        driving_in_path_tolerance=0.25,
        dynamic_obstacle_count=0,
        dynamic_obstacle_motion=False,
        extend_segment=False,
    ),
    NavigationDifficulty(
        goal_reached_threshold=0.25,
        driving_in_path_tolerance=0.25,
        dynamic_obstacle_count=5,
        dynamic_obstacle_motion=False,
        extend_segment=True,
    ),
    NavigationDifficulty(
        goal_reached_threshold=0.25,
        driving_in_path_tolerance=0.25,
        dynamic_obstacle_count=10,
        dynamic_obstacle_motion=False,
        extend_segment=True,
    ),
    NavigationDifficulty(
        goal_reached_threshold=0.25,
        driving_in_path_tolerance=0.25,
        dynamic_obstacle_count=20,
        dynamic_obstacle_motion=False,
        extend_segment=True,
    ),
    NavigationDifficulty(
        goal_reached_threshold=0.25,
        driving_in_path_tolerance=0.25,
        dynamic_obstacle_count=20,
        dynamic_obstacle_motion=True,
        extend_segment=True,
    ),
]

OBSTACLES_EASY = [
    (1.5, 1.5),
    (1.5, -1.5),
    (-1.5, 1.5),
    (-1.5, -1.5),
]

STATS_BUFFER_SIZE = 50
