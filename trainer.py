# pylint: disable=redefined-outer-name, c-extension-no-member
import configparser
import signal
import sys

import gymnasium as gym
from stable_baselines3.common.base_class import BaseAlgorithm
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.env_util import make_vec_env

import wandb
from velmwheel_gym import *  # pylint: disable=wildcard-import, unused-wildcard-import
from velmwheel_gym.logger import init_logging
from velmwheel_rl.common import (
    ParameterReader,
    bootstrap_argument_parser,
    create_model,
    get_model_save_path_and_tb_log_name,
    load_model,
)


def monitor_network(algorithm: str, model: BaseAlgorithm):
    match algorithm:
        case "PPO":
            wandb.watch(model.policy, log="all")
        case "TD3":
            wandb.watch(
                (
                    model.policy.actor,
                    model.policy.actor_target,
                    model.policy.critic,
                    model.policy.critic_target,
                ),
                log_freq=1,
                log="all",
            )
        case "DDPG":
            wandb.watch(
                (
                    model.policy.actor,
                    model.policy.critic,
                ),
                log_freq=1,
                log="all",
            )
        case _:
            raise ValueError(f"Unsupported algorithm: {algorithm}")


# ---------------------------------------------------------------------------- #
#                                 Configuration                                #
# ---------------------------------------------------------------------------- #

parser = bootstrap_argument_parser()
parser.add_argument("--timesteps", type=int, help="Number of timesteps", required=False)
parser.add_argument(
    "--envs",
    type=int,
    help="Number of parallel environments",
    required=False,
    default=1,
)
parser.add_argument(
    "--save_freq", type=int, help="Frequency of saving the model", required=False
)

args = parser.parse_args()
config = configparser.ConfigParser()
config.read([args.config])
param_reader = ParameterReader("trainer", args, config)

# pylint: disable=duplicate-code
log_level = param_reader.read("log_level")
gym_env = param_reader.read("gym_env")
algorithm = param_reader.read("algorithm")
model_path = param_reader.read("model")
replay_buffer_path = param_reader.read("replay_buffer")
timesteps = int(param_reader.read("timesteps"))
goal_reached_threshold = float(param_reader.read("goal_reached_threshold"))
real_time_factor = float(param_reader.read("real_time_factor"))
envs = int(param_reader.read("envs"))

init_logging(log_level)

# ---------------------------------------------------------------------------- #
#                              Training the model                              #
# ---------------------------------------------------------------------------- #

model_save_path, tb_log_name = get_model_save_path_and_tb_log_name(
    algorithm, model_path
)

extra_params = dict(
    min_goal_dist=goal_reached_threshold, real_time_factor=real_time_factor
)
if envs > 1:
    env = make_vec_env(gym_env, n_envs=4, **extra_params)
else:
    env = gym.make(gym_env, **extra_params)

if model_path:
    model, model_config = load_model(
        algorithm, env, param_reader, model_path, replay_buffer_path
    )
else:
    model, model_config = create_model(algorithm, env, param_reader)

checkpoint_callback = CheckpointCallback(
    save_freq=int(param_reader.read("save_freq")),
    save_path=model_save_path,
    name_prefix=algorithm.lower(),
    save_replay_buffer=True,
    save_vecnormalize=True,
)

run = wandb.init(
    project="velmwheel",
    config=model_config,
    sync_tensorboard=True,
)


# pylint: disable=unused-argument, global-statement
def sigint_handler(sig, frame):
    global model
    if model:
        del model
    run.finish()
    sys.exit(0)


signal.signal(signal.SIGINT, sigint_handler)

if str(param_reader.read("monitor_network")) == "true":
    monitor_network(algorithm, model)

model.learn(
    total_timesteps=timesteps,
    progress_bar=True,
    callback=checkpoint_callback,
    tb_log_name=tb_log_name,
    reset_num_timesteps=False,
)

run.finish()
