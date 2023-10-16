import argparse
import configparser
import math

import gym
from stable_baselines3 import DQN

from velmwheel_gym.constants import ACTION_TO_DIRECTION
from velmwheel_gym.env import VelmwheelEnv
from velmwheel_gym.logger import init_logging

init_logging()

# ---------------------------------------------------------------------------- #
#                               Argument parsing                               #
# ---------------------------------------------------------------------------- #

parser = argparse.ArgumentParser(
    prog="Tester script",
    description="Script for testing reinforcement learning models for WUT Velmwheel robot",
)
parser.add_argument(
    "--gym_env", type=str, help="Name of the Gym environment", required=False
)
parser.add_argument("--model", type=str, help="Model to load path", required=False)
parser.add_argument(
    "--replay_buffer", type=str, help="Replay buffer to load", required=False
)

args = parser.parse_args()

# ---------------------------------------------------------------------------- #
#                             Reading configuration                            #
# ---------------------------------------------------------------------------- #

config = configparser.ConfigParser()
config.read(["config.ini"])


# ---------------------------------------------------------------------------- #
#                             Evaluating parameters                            #
# ---------------------------------------------------------------------------- #
class ParameterReader:
    def __init__(self, args: argparse.Namespace, config: configparser.ConfigParser):
        self._args = args
        self._config = config

    def read(self, name: str):
        value = self._config.get("tester", name) if self._config else self._args[name]
        print(f"{name}={value}")
        return value


param_reader = ParameterReader(args, config)

gym_env = param_reader.read("gym_env")
model_path = param_reader.read("model")
replay_buffer_path = param_reader.read("replay_buffer")

# ---------------------------------------------------------------------------- #
#                               Testing the model                              #
# ---------------------------------------------------------------------------- #

env = gym.make(gym_env)

model = DQN.load(model_path)
model.load_replay_buffer(replay_buffer_path)
model.set_env(env)

obs = env.reset()

goal = [3, 3]
min_dist_to_goal = math.inf

while True:
    action, _states = model.predict(obs, deterministic=True)

    obs, rewards, dones, info = env.step(int(action))

    print("----------------------------------------------")
    print(f"direction={ACTION_TO_DIRECTION[int(action)]}")
    print(f"{obs=}")
    dist_to_goal = math.dist(goal, obs)
    print(f"{dist_to_goal=}")
    print(f"{min_dist_to_goal=}")
    print("----------------------------------------------")

    if dist_to_goal < min_dist_to_goal:
        min_dist_to_goal = dist_to_goal

    if dist_to_goal < 0.1:
        break
