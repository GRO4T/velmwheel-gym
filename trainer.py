import configparser

import gym
from stable_baselines3.common.callbacks import (
    CheckpointCallback,
    EvalCallback,
    StopTrainingOnNoModelImprovement,
)

from velmwheel_ai.common import (
    ParameterReader,
    bootstrap_argument_parser,
    create_model,
    get_model_save_path_and_tb_log_name,
    load_model,
)
from velmwheel_gym.env import VelmwheelEnv
from velmwheel_gym.logger import init_logging

init_logging()

# ---------------------------------------------------------------------------- #
#                                 Configuration                                #
# ---------------------------------------------------------------------------- #

parser = bootstrap_argument_parser()
parser.add_argument("--timesteps", type=int, help="Number of timesteps", required=False)

args = parser.parse_args()
config = configparser.ConfigParser()
config.read(["config.ini"])
param_reader = ParameterReader("trainer", args, config)

gym_env = param_reader.read("gym_env")
algorithm = param_reader.read("algorithm")
model_path = param_reader.read("model")
replay_buffer_path = param_reader.read("replay_buffer")
timesteps = int(param_reader.read("timesteps"))
goal_reached_threshold = float(param_reader.read("goal_reached_threshold"))
real_time_factor = float(param_reader.read("real_time_factor"))

# ---------------------------------------------------------------------------- #
#                              Training the model                              #
# ---------------------------------------------------------------------------- #

model_save_path, tb_log_name = get_model_save_path_and_tb_log_name(
    algorithm, model_path
)

env = gym.make(gym_env)
env.env.min_goal_dist = goal_reached_threshold
env.env.real_time_factor = real_time_factor

if model_path:
    model = load_model(algorithm, env, model_path, replay_buffer_path)
else:
    model = create_model(algorithm, env)

checkpoint_callback = CheckpointCallback(
    save_freq=1000,
    save_path=model_save_path,
    name_prefix=algorithm.lower(),
    save_replay_buffer=True,
    save_vecnormalize=True,
)

stop_train_callback = StopTrainingOnNoModelImprovement(
    max_no_improvement_evals=10, min_evals=10, verbose=1
)
eval_callback = EvalCallback(
    env, eval_freq=1000, callback_after_eval=stop_train_callback, verbose=1
)

model.learn(
    total_timesteps=timesteps,
    progress_bar=True,
    callback=checkpoint_callback,
    tb_log_name=tb_log_name,
    reset_num_timesteps=False,
)
