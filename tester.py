import configparser
import math
import subprocess

import gym

from velmwheel_ai.common import ParameterReader, bootstrap_argument_parser, load_model
from velmwheel_gym.env import VelmwheelEnv
from velmwheel_gym.logger import init_logging
from velmwheel_gym.types import Point

init_logging()

# ---------------------------------------------------------------------------- #
#                                 Configuration                                #
# ---------------------------------------------------------------------------- #

parser = bootstrap_argument_parser()
parser.add_argument("--goal_x", type=float, help="Goal x coordinate")
parser.add_argument("--goal_y", type=float, help="Goal y coordinate")

args = parser.parse_args()
config = configparser.ConfigParser()
config.read(["config.ini"])
param_reader = ParameterReader("tester", args, config)

gym_env = param_reader.read("gym_env")
algorithm = param_reader.read("algorithm")
model_path = param_reader.read("model")
replay_buffer_path = param_reader.read("replay_buffer")
goal_reached_threshold = float(param_reader.read("goal_reached_threshold"))
real_time_factor = float(param_reader.read("real_time_factor"))
goal_x = float(param_reader.read("goal_x"))
goal_y = float(param_reader.read("goal_y"))

# ---------------------------------------------------------------------------- #
#                               Testing the model                              #
# ---------------------------------------------------------------------------- #

goal = [goal_x, goal_y]
min_dist_to_goal = math.inf

env = gym.make(gym_env)
env.env.real_time_factor = real_time_factor
env.env.goal = Point(*goal)

model = load_model(algorithm, env, model_path, replay_buffer_path)

obs = env.reset()

while True:
    action, _states = model.predict(obs, deterministic=True)

    obs, rewards, dones, info = env.step(action)

    print("----------------------------------------------")
    print(f"action={action}")
    print(f"{obs=}")
    pos_x, pos_y, *_ = obs
    dist_to_goal = math.dist(goal, (pos_x, pos_y))
    print(f"{dist_to_goal=}")
    print(f"{min_dist_to_goal=}")
    print("----------------------------------------------")

    if dist_to_goal < min_dist_to_goal:
        min_dist_to_goal = dist_to_goal

    if dist_to_goal < goal_reached_threshold:
        env.env._robot.move([0.0, 0.0])
        value = input("Next goal? (y/n): ")
        if value.lower() == "n":
            subprocess.run("./kill_sim.sh", shell=True, check=True)
            break
        goal_x = float(input("Goal x: "))
        goal_y = float(input("Goal y: "))
        goal = [goal_x, goal_y]
        env.env._goal_manager._current_goal = Point(*goal)
        obs = env.reset()
