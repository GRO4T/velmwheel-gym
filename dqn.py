import gym

from envs.velmwheel import VelmwheelEnv
from stable_baselines3 import DQN
from stable_baselines3.common.evaluation import evaluate_policy


env = gym.make("Velmwheel-v0")

model = DQN("MlpPolicy", env, verbose=1)

model.learn(total_timesteps=1000, progress_bar=True)
