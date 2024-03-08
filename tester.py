import configparser
import math

import gym
from stable_baselines3 import DDPG

from velmwheel_ai.common import ParameterReader, bootstrap_argument_parser, load_model
from velmwheel_gym.env import VelmwheelEnv
from velmwheel_gym.logger import init_logging
from velmwheel_gym.types import Point

init_logging()

# ---------------------------------------------------------------------------- #
#                                 Configuration                                #
# ---------------------------------------------------------------------------- #

args = bootstrap_argument_parser().parse_args()
config = configparser.ConfigParser()
config.read(["config.ini"])
param_reader = ParameterReader(args, config)

gym_env = param_reader.read("gym_env")
algorithm = param_reader.read("algorithm")
model_path = param_reader.read("model")
replay_buffer_path = param_reader.read("replay_buffer")
goal_reached_threshold = float(param_reader.read("goal_reached_threshold"))
real_time_factor = float(param_reader.read("real_time_factor"))

# ---------------------------------------------------------------------------- #
#                               Testing the model                              #
# ---------------------------------------------------------------------------- #

env = gym.make(gym_env)
env.env.real_time_factor = real_time_factor

model = DDPG.load(model_path, env=env)
model = load_model(algorithm, env, model_path, replay_buffer_path)

obs = env.reset()

goal = [-3, 3]
min_dist_to_goal = math.inf

env.env.goal = Point(*goal)

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
        break
