import configparser
import math
import subprocess

import gymnasium as gym

from velmwheel_gym import *  # pylint: disable=wildcard-import, unused-wildcard-import
from velmwheel_gym.logger import init_logging
from velmwheel_gym.types import Point
from velmwheel_rl.common import ParameterReader, bootstrap_argument_parser, load_model

# ---------------------------------------------------------------------------- #
#                                 Configuration                                #
# ---------------------------------------------------------------------------- #

parser = bootstrap_argument_parser()
parser.add_argument("--goal_x", type=float, help="Goal >x coordinate")
parser.add_argument("--goal_y", type=float, help="Goal y coordinate")

args = parser.parse_args()
config = configparser.ConfigParser()
config.read(["config.ini"])
param_reader = ParameterReader("tester", args, config)

# pylint: disable=duplicate-code
log_level = param_reader.read("log_level")
gym_env = param_reader.read("gym_env")
algorithm = param_reader.read("algorithm")
model_path = param_reader.read("model")
replay_buffer_path = param_reader.read("replay_buffer")
goal_reached_threshold = float(param_reader.read("goal_reached_threshold"))
real_time_factor = float(param_reader.read("real_time_factor"))
goal_x = float(param_reader.read("goal_x"))
goal_y = float(param_reader.read("goal_y"))

init_logging(log_level)

# ---------------------------------------------------------------------------- #
#                               Testing the model                              #
# ---------------------------------------------------------------------------- #

starting_position = Point(0.0, 0.0)
goal = [goal_x, goal_y]
min_dist_to_goal = math.inf  # pylint: disable=invalid-name

env = gym.make(
    gym_env, min_goal_dist=goal_reached_threshold, real_time_factor=real_time_factor
)

model, _ = load_model(
    algorithm, env, param_reader, model_path, replay_buffer_path, test_mode=True
)

obs, _ = env.reset()

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

    env.render()

    if dist_to_goal < goal_reached_threshold:
        env.reset()
        value = input("Next goal? (y/n): ")
        if value.lower() == "n":
            subprocess.run("./kill_sim.sh", shell=True, check=True)
            break
        goal_x = float(input("Goal x: "))
        goal_y = float(input("Goal y: "))
        obs, _ = env.reset(
            options={
                "starting_position": starting_position,
                "goal": Point(goal_x, goal_y),
            }
        )
