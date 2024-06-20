from velmwheel_gym.types import NavigationDifficulty

# Step time in seconds at real time factor 1.0
BASE_STEP_TIME = 0.16

# Number of lidar rays
LIDAR_DATA_SIZE = 90

# Number of coordinates in global guidance observation
GLOBAL_GUIDANCE_OBSERVATION_POINTS = 10

# All coordinates are divided by this factor while being part of an observation
COORDINATES_NORMALIZATION_FACTOR = 20.0

# Navigation difficulty levels
NAVIGATION_DIFFICULTIES = [
    NavigationDifficulty(
        goal_reached_threshold=1.0,
        driving_in_path_tolerance=1.0,
        dynamic_obstacle_count=0,
    ),
    NavigationDifficulty(
        goal_reached_threshold=0.5,
        driving_in_path_tolerance=0.5,
        dynamic_obstacle_count=0,
    ),
    NavigationDifficulty(
        goal_reached_threshold=0.25,
        driving_in_path_tolerance=0.5,
        dynamic_obstacle_count=0,
    ),
    NavigationDifficulty(
        goal_reached_threshold=0.25,
        driving_in_path_tolerance=0.5,
        dynamic_obstacle_count=5,
    ),
]

STATS_BUFFER_SIZE = 10
