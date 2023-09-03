import math
import argparse

import gym
from stable_baselines3 import DQN

from velmwheel_gym.constants import ACTION_TO_DIRECTION
from velmwheel_gym.logger import init_logging
from velmwheel_gym.envs.v1.env import VelmwheelEnv


init_logging()

# ---------------------------------------------------------------------------- #
#                               Argument parsing                               #
# ---------------------------------------------------------------------------- #

parser = argparse.ArgumentParser(
    prog="Tester script",
    description="Script for testing reinforcement learning models for WUT Velmwheel robot",
)
parser.add_argument(
    "--gym_env", type=str, help="Name of the Gym environment", required=True
)
parser.add_argument("--model", type=str, help="Model to load path", required=False)
parser.add_argument(
    "--replay_buffer", type=str, help="Replay buffer to load", required=False
)

args = parser.parse_args()

# ---------------------------------------------------------------------------- #
#                               Testing the model                              #
# ---------------------------------------------------------------------------- #

env = gym.make(args.gym_env)

model = DQN.load(args.model)
model.load_replay_buffer(args.replay_buffer)
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
