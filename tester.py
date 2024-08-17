import configparser
import math
import pickle
import time

import gymnasium as gym

from velmwheel_gym import *  # pylint: disable=wildcard-import, unused-wildcard-import
from velmwheel_gym.constants import NAVIGATION_DIFFICULTIES
from velmwheel_gym.logger import init_logging
from velmwheel_gym.types import Point
from velmwheel_rl.common import ParameterReader, bootstrap_argument_parser, load_model

# ---------------------------------------------------------------------------- #
#                                 Configuration                                #
# ---------------------------------------------------------------------------- #

parser = bootstrap_argument_parser()
parser.add_argument("--goal_x", type=float, help="Goal x coordinate")
parser.add_argument("--goal_y", type=float, help="Goal y coordinate")
parser.add_argument("--start_x", type=float, help="Start x coordinate")
parser.add_argument("--start_y", type=float, help="Start y coordinate")
parser.add_argument(
    "--render", type=bool, help="Render the environment", required=False
)
parser.add_argument(
    "--generate_next_goal",
    type=str,
    help="Generate a new goal",
    required=False,
    default="false",
)
parser.add_argument(
    "--save_footprint",
    type=str,
    help="Save the robot footprint",
    required=False,
    default="false",
)
parser.add_argument(
    "--time_limit",
    type=int,
    help="Time limit for the episode",
    required=False,
    default=3600,
)

args = parser.parse_args()
config = configparser.ConfigParser()
config.read([args.config])
param_reader = ParameterReader("tester", args, config)

# pylint: disable=duplicate-code
log_level = param_reader.read("log_level")
gym_env = param_reader.read("gym_env")
algorithm = param_reader.read("algorithm")
model_path = param_reader.read("model")
replay_buffer_path = param_reader.read("replay_buffer")
navigation_difficulty_level = int(param_reader.read("navigation_difficulty_level"))
real_time_factor = float(param_reader.read("real_time_factor"))
goal_x = float(param_reader.read("goal_x"))
goal_y = float(param_reader.read("goal_y"))
start_x = float(param_reader.read("start_x"))
start_y = float(param_reader.read("start_y"))
render = param_reader.read("render")
generate_next_goal = (
    True if param_reader.read("generate_next_goal") == "true" else False
)
save_footprint = True if param_reader.read("save_footprint") == "true" else False
time_limit = int(param_reader.read("time_limit"))

init_logging(log_level)

# ---------------------------------------------------------------------------- #
#                               Testing the model                              #
# ---------------------------------------------------------------------------- #

starting_position = Point(0.0, 0.0)
goal = [goal_x, goal_y]
min_dist_to_goal = math.inf  # pylint: disable=invalid-name
difficulty = NAVIGATION_DIFFICULTIES[navigation_difficulty_level]

env = gym.make(
    gym_env,
    difficulty=difficulty,
    real_time_factor=real_time_factor,
    render_mode="human" if render == "true" else None,
    training_mode=False,
    global_path_segment_length=float(
        param_reader.read("global_path_segment_length", "VelmwheelGym")
    ),
    variant=param_reader.read("variant", "VelmwheelGym"),
)

model, _ = load_model(
    algorithm, env, param_reader, model_path, replay_buffer_path, test_mode=True
)

obs, _ = env.reset(
    options={
        "starting_position": Point(start_x, start_y),
        "goal": Point(goal_x, goal_y),
    }
)

footprints = {"position": [], "orientation": [], "time": [], "obstacles": []}
steps = 0
start = time.time()
total = 0.0
while True:
    action, _ = model.predict(obs, deterministic=True)

    obs, reward, terminated, truncated, info = env.step(action)
    if terminated:
        steps = 0

    footprints["position"].append(env.env.robot_position)
    footprints["time"].append(start)

    # footprints["obstacles"].append(
    #     [(x, y) for x, y in zip(env.env.robot.env.dynamic_obstacles_x, env.env.robot.env.dynamic_obstacles_y)]
    # )

    print("----------------------------------------------")
    print(f"{action=}")
    print(f"{reward=}")
    print(f"{terminated=}")
    dist_to_goal = math.dist(env.env.goal, env.env.robot_position)
    print(f"{dist_to_goal=}")
    print(f"{min_dist_to_goal=}")
    print(f"{env.env.starting_position=}")
    print(f"{env.env.goal=}")
    print(f"{env.env.robot_position=}")
    print(f"{info=}")
    print(f"{obs=}")
    print("----------------------------------------------")

    if info.get("status", None) == "segment_reached":
        steps = 0
        env.reset()

    if terminated and reward < 0:
        if info.get("status", None) == "max_steps_reached":
            print("Max steps reached!")
        else:
            print("Collision detected!")
        env.step([0.0, 0.0, 0.0])
        if generate_next_goal:
            steps = 0
            obs, _ = env.reset()
        else:
            break

    if dist_to_goal < min_dist_to_goal:
        min_dist_to_goal = dist_to_goal

    if dist_to_goal < difficulty.goal_reached_threshold:
        print("Goal reached!")
        env.step([0.0, 0.0, 0.0])
        if generate_next_goal:
            steps = 0
            obs, _ = env.reset()
        else:
            break

    steps += 1
    # if steps > env.env.max_episode_steps:
    #     print("Max steps reached!")
    #     env.step([0.0, 0.0, 0.0])
    #     if not env.env.is_final_goal or (env.env.is_final_goal and generate_next_goal):
    #         steps = 0
    #         obs, _ = env.reset()
    #     else:
    #         break

    if render == "true":
        env.render()

    end = time.time()
    elapsed = end - start
    start = end
    # if elapsed < 0.050:
    #     time.sleep(0.050 - elapsed)
    print(f"fps: {1 / elapsed}")

    if total > time_limit:
        print("Time limit reached!")
        env.step([0.0, 0.0, 0.0])
        break

with open("footprint.pkl", "wb") as f:
    pickle.dump(footprints, f)
