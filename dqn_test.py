import math

import gym
from stable_baselines3 import DQN

from velmwheel_gym.robot import ACTION_TO_DIRECTION
from velmwheel_gym.logger import init_logging
from velmwheel_gym.envs.velmwheel import VelmwheelEnv


init_logging()


# env = gym.make("Velmwheel-v0")
env = gym.make("Velmwheel50-v2")


model = DQN.load("./models/velmwheel_v1/dqn/3/dqn_90000_steps")

obs = env.reset()

goal = [3, 3]

while True:
    action, _states = model.predict(obs, deterministic=True)

    print(f"direction={ACTION_TO_DIRECTION[int(action)]}")

    obs, rewards, dones, info = env.step(int(action))

    print(f"{obs=}")
    dist_to_goal = math.dist(goal, obs)
    print(f"{dist_to_goal=}")

    if dist_to_goal < 0.1:
        break
