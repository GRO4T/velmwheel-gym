import configparser
import math
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

steps = 0
start = time.time()
while True:
    action, _ = model.predict(obs, deterministic=True)

    obs, reward, terminated, truncated, info = env.step(action)

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
    print(f"{obs=}")
    print("----------------------------------------------")

    if dist_to_goal < min_dist_to_goal:
        min_dist_to_goal = dist_to_goal

    if dist_to_goal < difficulty.goal_reached_threshold:
        obs, _ = env.reset()
    steps += 1
    if steps > env.env.max_episode_steps:
        steps = 0
        env.reset()

    if render == "true":
        env.render()

    end = time.time()
    elapsed = end - start
    start = end
    if elapsed < 0.016:
        time.sleep(0.016 - elapsed)
    print(f"fps: {1 / elapsed}")
