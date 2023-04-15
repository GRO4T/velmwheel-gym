import gym
from stable_baselines3 import DQN
from stable_baselines3.common.evaluation import evaluate_policy
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.logger import HParam

from velmwheel_gym.envs.velmwheel import VelmwheelEnv
from velmwheel_gym.logger import init_logging


init_logging()


# env = gym.make("Velmwheel-v0")
env = gym.make("Velmwheel-v2")


model = DQN(
    "MlpPolicy",
    env,
    verbose=1,
    tensorboard_log="./logs/tensorboard",
    learning_starts=0,
    target_update_interval=1000,
    # batch_size=1,
    learning_rate=0.001,
)

print(model.policy)

model.learn(total_timesteps=100000, progress_bar=True)
