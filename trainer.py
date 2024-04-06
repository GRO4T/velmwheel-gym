import configparser

import gymnasium as gym
from stable_baselines3.common.callbacks import (
    CheckpointCallback,
    EvalCallback,
    StopTrainingOnNoModelImprovement,
)
from stable_baselines3.common.env_util import make_vec_env

from velmwheel_gym import *  # pylint: disable=unused-import
from velmwheel_gym.logger import init_logging
from velmwheel_rl.common import (
    ParameterReader,
    bootstrap_argument_parser,
    create_model,
    get_model_save_path_and_tb_log_name,
    load_model,
)

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

args = parser.parse_args()
config = configparser.ConfigParser()
config.read(["config.ini"])
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
    model = load_model(algorithm, env, model_path, replay_buffer_path)
else:
    model = create_model(algorithm, env)

checkpoint_callback = CheckpointCallback(
    save_freq=5000,
    save_path=model_save_path,
    name_prefix=algorithm.lower(),
    save_replay_buffer=True,
    save_vecnormalize=True,
)

model.learn(
    total_timesteps=timesteps,
    progress_bar=True,
    callback=checkpoint_callback,
    tb_log_name=tb_log_name,
    reset_num_timesteps=False,
)
