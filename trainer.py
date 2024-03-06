import argparse
import configparser
import os

import gym
from stable_baselines3 import DQN
from stable_baselines3.common.callbacks import CheckpointCallback

from velmwheel_gym.env import VelmwheelEnv
from velmwheel_gym.logger import init_logging

init_logging()

# ---------------------------------------------------------------------------- #
#                               Argument parsing                               #
# ---------------------------------------------------------------------------- #

parser = argparse.ArgumentParser(
    prog="Trainer script",
    description="Script for training reinforcement learning models for WUT Velmwheel robot",
)
parser.add_argument(
    "--gym_env", type=str, help="Name of the Gym environment", required=False
)
parser.add_argument("--model", type=str, help="Model to load path", required=False)
parser.add_argument(
    "--replay_buffer", type=str, help="Replay buffer to load", required=False
)
parser.add_argument("--timesteps", type=int, help="Number of timesteps", required=False)

parser.add_argument("--min_goal_dist", type=float, help="Minimum distance to goal")

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
        value = self._config.get("trainer", name) if self._config else self._args[name]
        print(f"{name}={value}")
        return value


param_reader = ParameterReader(args, config)

gym_env = param_reader.read("gym_env")
model_path = param_reader.read("model")
replay_buffer_path = param_reader.read("replay_buffer")
timesteps = int(param_reader.read("timesteps"))
min_goal_dist = float(param_reader.read("min_goal_dist"))
real_time_factor = float(param_reader.read("real_time_factor"))

# ---------------------------------------------------------------------------- #
#                               Helper functions                               #
# ---------------------------------------------------------------------------- #


def get_model_save_path(model_path: str, replay_buffer_path: str) -> str:
    if model_path and replay_buffer_path:
        return os.path.dirname(model_path)

    model_save_dir = "./models/velmwheel_v1/dqn"
    run_id = 1
    for _ in os.listdir(model_save_dir):
        run_id += 1

    return os.path.join(model_save_dir, str(run_id))


# ---------------------------------------------------------------------------- #
#                              Training the model                              #
# ---------------------------------------------------------------------------- #

# Define model saving callback
checkpoint_callback = CheckpointCallback(
    save_freq=1000,
    save_path=get_model_save_path(model_path, replay_buffer_path),
    name_prefix="dqn",
    save_replay_buffer=True,
    save_vecnormalize=True,
)

# Create/Load the model
model = None
env = gym.make(gym_env)
env.env.min_goal_dist = min_goal_dist
env.env.real_time_factor = real_time_factor

if model_path:
    model = DQN.load(model_path, env=env, exploration_final_eps=0.0)
    model.load_replay_buffer(replay_buffer_path)
else:
    model = DQN(
        "MlpPolicy",
        env,
        exploration_final_eps=0.0,
        learning_rate=0.001,
        learning_starts=500,
        target_update_interval=100,
        verbose=1,
        tensorboard_log="./logs/tensorboard",
        device="cuda",
    )

# Train the model
model.learn(
    total_timesteps=timesteps,
    progress_bar=True,
    callback=checkpoint_callback,
    reset_num_timesteps=False,
)
