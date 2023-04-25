import math

import gym
from stable_baselines3 import DQN

from velmwheel_gym.robot import ACTION_TO_DIRECTION
from velmwheel_gym.logger import init_logging
from velmwheel_gym.envs.velmwheel import VelmwheelEnv


init_logging()


# env = gym.make("Velmwheel-v0")
env = gym.make("Velmwheel-v3")


model = DQN.load("./models/velmwheel_v2/dqn_20000_steps")

obs = env.reset()
env.step(1)
