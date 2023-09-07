import os
import argparse

import gym
from stable_baselines3 import DQN
from stable_baselines3.common.callbacks import CheckpointCallback

from velmwheel_gym.envs.v1.env import VelmwheelEnv
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
    "--gym_env", type=str, help="Name of the Gym environment", required=True
)
parser.add_argument("--model", type=str, help="Model to load path", required=False)
parser.add_argument(
    "--replay_buffer", type=str, help="Replay buffer to load", required=False
)
parser.add_argument("--timesteps", type=int, help="Number of timesteps", required=True)

parser.add_argument("--min_goal_dist", type=float, help="Minimum distance to goal")

args = parser.parse_args()

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
    save_freq=10000,
    save_path=get_model_save_path(args.model, args.replay_buffer),
    name_prefix="dqn",
    save_replay_buffer=True,
    save_vecnormalize=True,
)

# Create/Load the model
model = None
env = gym.make(args.gym_env)
env.env.min_goal_dist = args.min_goal_dist

if args.model:
    model = DQN.load(args.model)
    model.load_replay_buffer(args.replay_buffer)
    model.set_env(env)
else:
    model = DQN(
        "MlpPolicy",
        env,
        verbose=1,
        tensorboard_log="./logs/tensorboard",
        learning_starts=0,
        target_update_interval=1000,
        learning_rate=0.001,
    )

# Train the model
model.learn(
    total_timesteps=args.timesteps,
    progress_bar=True,
    callback=checkpoint_callback,
    reset_num_timesteps=False,
)
