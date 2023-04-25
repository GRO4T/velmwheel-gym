import os

import gym
from stable_baselines3 import DQN
from stable_baselines3.common.callbacks import CheckpointCallback

from velmwheel_gym.envs.velmwheel import VelmwheelEnv
from velmwheel_gym.logger import init_logging


init_logging()


# env = gym.make("Velmwheel-v0")
# env = gym.make("Velmwheel-v2")
env = gym.make("Velmwheel-v4")

MODEL_DIRECTORY = "./models/velmwheel_v2/dqn/2"
MODEL_FILENAME = "dqn_20000_steps"
MODEL_REPLAY_BUFFER_FILENAME = "dqn_replay_buffer_20000_steps"

model = DQN.load(os.path.join(MODEL_DIRECTORY, MODEL_FILENAME))
model.load_replay_buffer(os.path.join(MODEL_DIRECTORY, MODEL_REPLAY_BUFFER_FILENAME))
model.set_env(env)

# Save a checkpoint every 1000 steps
checkpoint_callback = CheckpointCallback(
    save_freq=10000,
    save_path=MODEL_DIRECTORY,
    name_prefix="dqn",
    save_replay_buffer=True,
    save_vecnormalize=True,
)

model.learn(
    total_timesteps=20000,
    progress_bar=True,
    callback=checkpoint_callback,
    reset_num_timesteps=False,
)
