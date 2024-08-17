# pylint: disable=redefined-outer-name, c-extension-no-member
import configparser
import shutil
import signal
import sys
from pathlib import Path

import gymnasium as gym
from stable_baselines3.common.base_class import BaseAlgorithm
from stable_baselines3.common.callbacks import CheckpointCallback, EvalCallback
from stable_baselines3.common.env_util import make_vec_env

import wandb
from velmwheel_gym import *  # pylint: disable=wildcard-import, unused-wildcard-import
from velmwheel_gym.constants import NAVIGATION_DIFFICULTIES
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
parser.add_argument(
    "--batch_size", type=int, help="Batch size for training", required=False
)
parser.add_argument(
    "--buffer_size", type=int, help="Size of the replay buffer", required=False
)
parser.add_argument(
    "--actor_lr", type=float, help="Learning rate for the actor", required=False
)
parser.add_argument(
    "--critic_lr", type=float, help="Learning rate for the critic", required=False
)
parser.add_argument(
    "--render", type=bool, help="Render the environment", required=False
)
parser.add_argument("--run_id", type=str, help="Run ID", required=False, default="")
parser.add_argument("--global_path_segment_length", type=float, required=False)
parser.add_argument("--noise_decay_steps", type=int, required=False)
parser.add_argument("--variant", type=str, required=False)
parser.add_argument("--gamma", type=float, required=False)

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
navigation_difficulty_level = int(param_reader.read("navigation_difficulty_level"))
real_time_factor = float(param_reader.read("real_time_factor"))
envs = int(param_reader.read("envs"))
render = param_reader.read("render")
run_id = param_reader.read("run_id")

init_logging(log_level, f"./logs/velmwheel/default{run_id}.log")

# ---------------------------------------------------------------------------- #
#                              Training the model                              #
# ---------------------------------------------------------------------------- #

model_save_path, tb_log_name = get_model_save_path_and_tb_log_name(
    algorithm, model_path
)

extra_params = dict(
    difficulty=NAVIGATION_DIFFICULTIES[navigation_difficulty_level],
    real_time_factor=real_time_factor,
    render_mode="human" if render == "true" else None,
    training_mode=True,
    name=run_id,
    global_path_segment_length=float(
        param_reader.read("global_path_segment_length", "VelmwheelGym")
    ),
    render_freq=int(param_reader.read("render_freq", "VelmwheelGym")),
    variant=param_reader.read("variant", "VelmwheelGym"),
    env_name=gym_env,
)
if envs > 1:
    env = make_vec_env(gym_env, n_envs=4, **extra_params)
else:
    # env = gym.make(gym_env, **extra_params)
    env = make_vec_env(gym_env, n_envs=1, env_kwargs=extra_params)
    # env = VecCheckNan(env, raise_exception=True)

if model_path:
    model, model_config = load_model(
        algorithm, env, param_reader, model_path, replay_buffer_path
    )
else:
    model, model_config = create_model(algorithm, env, param_reader)

callbacks = []

callbacks.append(
    CheckpointCallback(
        save_freq=int(param_reader.read("save_freq")),
        save_path=model_save_path,
        name_prefix=algorithm.lower(),
        save_replay_buffer=False,
        save_vecnormalize=False,
    )
)

callbacks.append(
    CheckpointCallback(
        save_freq=int(param_reader.read("replay_buffer_save_freq")),
        save_path=model_save_path,
        name_prefix=algorithm.lower(),
        save_replay_buffer=True,
        save_vecnormalize=True,
    )
)


# Save config and cmd
Path(model_save_path).mkdir(parents=True, exist_ok=True)
with open(f"{model_save_path}/cmdline.txt", "w+") as f:
    f.write(" ".join(sys.argv))
with open(f"{model_save_path}/config.ini", "w+") as f:
    config.write(f)
model_config["local_model_save_path"] = model_save_path

# Compress velmwheel_rl and velmwheel_gym and save them in the model_save_path
Path(model_save_path).mkdir(parents=True, exist_ok=True)
shutil.make_archive(f"{model_save_path}/velmwheel_rl", "zip", "velmwheel_rl")
shutil.make_archive(f"{model_save_path}/velmwheel_gym", "zip", "velmwheel_gym")
shutil.copy("trainer.py", model_save_path)
shutil.copy("tester.py", model_save_path)

# if "2D" in gym_env:
#     eval_env = gym.make(gym_env, **extra_params)
#     callbacks.append(
#         EvalCallback(
#             eval_env,
#             best_model_save_path=model_save_path,
#             eval_freq=10000,
#             deterministic=True,
#             render=False,
#         )
#     )


run = wandb.init(
    project="velmwheel",
    config=model_config,
    sync_tensorboard=True,
)

wandb.define_metric("level")
wandb.define_metric("episode_reward")
wandb.define_metric("mean_reward")
wandb.define_metric("local_success_rate")
wandb.define_metric("global_success_rate")


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
    callback=callbacks,
    tb_log_name=tb_log_name,
    reset_num_timesteps=False,
)

run.finish()
