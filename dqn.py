import os

import gym
from stable_baselines3 import DQN
from stable_baselines3.common.callbacks import CheckpointCallback

from velmwheel_gym.envs.v1.env import VelmwheelEnv
from velmwheel_gym.logger import init_logging


def get_model_path() -> str:
    model_base_path = "./models/velmwheel_v1/dqn"
    run_id = 1
    for path in os.listdir(model_base_path):
        if run_id == int(path):
            run_id += 1

    return os.path.join(model_base_path, str(run_id))


init_logging()


env = gym.make("Velmwheel50-v1")


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

# Save a checkpoint every 1000 steps
checkpoint_callback = CheckpointCallback(
    save_freq=10000,
    save_path=get_model_path(),
    name_prefix="dqn",
    save_replay_buffer=True,
    save_vecnormalize=True,
)

model.learn(total_timesteps=40000, progress_bar=True, callback=checkpoint_callback)
