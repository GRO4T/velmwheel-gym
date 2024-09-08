import argparse
import configparser
import os
from typing import Callable

import gymnasium as gym
import numpy as np
import torch
from stable_baselines3 import DDPG, PPO, SAC, TD3
from stable_baselines3.common.base_class import BaseAlgorithm

from velmwheel_rl.noise import OrnsteinUhlenbeckActionNoiseWithDecay


class ParameterReader:
    def __init__(
        self,
        default_section: str,
        args: argparse.Namespace,
        config: configparser.ConfigParser,
    ):
        self._default_section = default_section
        self._args = args
        self._config = config

    def read(self, name: str, section: str = None):
        value = (
            vars(self._args)[name]
            if name in self._args and vars(self._args)[name] is not None
            else self._config.get(section if section else self._default_section, name)
        )
        print(f"{name}={value}")
        return value


def get_model_save_path_and_tb_log_name(
    algorithm: str, model_path: str, override_run_id: int = None
) -> tuple[str, str]:
    if model_path:
        if override_run_id:
            return (
                os.path.join(f"./models/{algorithm.lower()}", str(override_run_id)),
                f"{algorithm}_{override_run_id}",
            )
        return (
            os.path.dirname(model_path),
            f"{algorithm}_{os.path.basename(os.path.dirname(model_path))}",
        )

    model_save_dir = f"./models/{algorithm.lower()}"
    max_run_id = 1
    for filename in os.listdir(model_save_dir):
        try:
            run_id = int(filename)
            if run_id > max_run_id:
                max_run_id = run_id
        except ValueError:
            continue

    return (
        os.path.join(model_save_dir, str(max_run_id + 1)),
        f"{algorithm}_{max_run_id + 1}",
    )


def linear_schedule(initial_value: float) -> Callable[[float], float]:
    """
    Linear learning rate schedule.

    :param initial_value: Initial learning rate.
    :return: schedule that computes
    current learning rate depending on remaining progress
    """

    def func(progress_remaining: float) -> float:
        """
        Progress will decrease from 1 (beginning) to 0.

        :param progress_remaining:
        :return: current learning rate
        """
        return progress_remaining * initial_value

    return func


def create_model(
    algorithm: str, env: gym.Env, param_reader: ParameterReader
) -> tuple[BaseAlgorithm, dict]:
    match algorithm:
        case "DDPG":
            n_actions = env.action_space.shape[-1]
            action_noise = OrnsteinUhlenbeckActionNoiseWithDecay(
                mean=np.zeros(n_actions),
                sigma=0.5 * np.ones(n_actions),
                decay=0.998,
                target_sigma=0.1 * np.ones(n_actions),
                delay_decay_for=int(param_reader.read("learning_starts", "DDPG"))
                - 200000,
            )
            # policy_kwargs = dict(
            #     net_arch=dict(pi=[400, 300], qf=[200, 150]),
            # )
            model = DDPG(
                "MlpPolicy",
                env,
                verbose=1,
                action_noise=action_noise,
                tensorboard_log="./logs/tensorboard",
                device="cuda",
                gamma=float(param_reader.read("gamma", "DDPG")),
                buffer_size=int(param_reader.read("buffer_size", "DDPG")),
                batch_size=int(param_reader.read("batch_size", "DDPG")),
                learning_starts=int(param_reader.read("learning_starts", "DDPG")),
                # policy_kwargs=policy_kwargs,
            )

            model.actor.optimizer.weight_decay = 0.01
            model.critic.optimizer.weight_decay = 0.01

            model.actor.optimizer.lr = float(param_reader.read("actor_lr", "DDPG"))
            model.critic.optimizer.lr = float(param_reader.read("critic_lr", "DDPG"))
        case "TD3":
            n_actions = env.action_space.shape[-1]
            action_noise = OrnsteinUhlenbeckActionNoiseWithDecay(
                mean=np.zeros(n_actions),
                sigma=1.0 * np.ones(n_actions),
                target_sigma=0.1 * np.ones(n_actions),
                decay_steps=int(param_reader.read("noise_decay_steps", "TD3")),
            )
            policy_kwargs = dict(
                # net_arch=dict(pi=[1024, 512, 256], qf=[1024, 512, 256]),
            )
            model = TD3(
                "MlpPolicy",
                # "MultiInputPolicy",
                env,
                verbose=1,
                action_noise=action_noise,
                tensorboard_log="./logs/tensorboard",
                device="cuda",
                buffer_size=int(param_reader.read("buffer_size", "TD3")),
                batch_size=int(param_reader.read("batch_size", "TD3")),
                learning_starts=int(param_reader.read("learning_starts", "TD3")),
                gamma=float(param_reader.read("gamma", "TD3")),
                optimize_memory_usage=True,
                replay_buffer_kwargs=dict(handle_timeout_termination=False),
                policy_kwargs=policy_kwargs,
                learning_rate=float(param_reader.read("actor_lr", "TD3")),
            )

            # model.actor.optimizer.param_groups[0]["weight_decay"] = 0.00001
            # model.critic.optimizer.param_groups[0]["weight_decay"] = 0.00001

            model.actor.optimizer.param_groups[0]["lr"] = float(
                param_reader.read("actor_lr", "TD3")
            )
            model.critic.optimizer.param_groups[0]["lr"] = float(
                param_reader.read("critic_lr", "TD3")
            )
        case "PPO":
            policy_kwargs = dict(net_arch=dict(pi=[256, 256], vf=[256, 128]))
            model = PPO(
                "MlpPolicy",
                env,
                verbose=1,
                tensorboard_log="./logs/tensorboard",
                device="cuda",
                policy_kwargs=policy_kwargs,
                gamma=float(param_reader.read("gamma", "PPO")),
                n_steps=int(param_reader.read("n_steps", "PPO")),
                learning_rate=float(param_reader.read("learning_rate", "PPO")),
                batch_size=int(param_reader.read("batch_size", "PPO")),
            )

            # model.policy.optimizer.param_groups[0]["weight_decay"] = 0.00001

        case "SAC":
            model = SAC(
                "MlpPolicy",
                env,
                verbose=1,
                tensorboard_log="./logs/tensorboard",
                device="cuda",
                batch_size=1024,
                buffer_size=1000000,
                learning_rate=0.0003,
            )
        case _:
            raise ValueError(f"Unknown algorithm: {algorithm}")

    wb_run_params = _get_wb_run_params(model, param_reader)

    print(model.policy)
    return model, wb_run_params


def _get_wb_run_params(model: BaseAlgorithm, param_reader: ParameterReader) -> dict:
    algorithm = param_reader.read("algorithm")

    wb_run_params = {
        "env_id": param_reader.read("gym_env"),
        "env_variant": param_reader.read("variant", "VelmwheelGym"),
        "algorithm": algorithm,
        "total_timesteps": int(param_reader.read("timesteps")),
        "policy_type": "MlpPolicy",
        "run_id": param_reader.read("run_id"),
    }

    match algorithm:
        case "DDPG" | "TD3":
            wb_run_params["learning_starts"] = model.learning_starts
            wb_run_params["action_noise"] = repr(model.action_noise)
            wb_run_params["gamma"] = model.gamma
            wb_run_params["buffer_size"] = model.buffer_size
            wb_run_params["batch_size"] = model.batch_size
            wb_run_params["actor_lr"] = param_reader.read("actor_lr", algorithm)
            wb_run_params["critic_lr"] = param_reader.read("critic_lr", algorithm)
            wb_run_params["noise_decay_steps"] = param_reader.read(
                "noise_decay_steps", algorithm
            )

    return wb_run_params


def _load_replay_buffer(model: BaseAlgorithm, replay_buffer_path: str):
    model.load_replay_buffer(replay_buffer_path)
    print(f"Replay buffer loaded: {model.replay_buffer.size()} transitions")


# pylint: disable=too-many-arguments
def load_model(
    algorithm: str,
    env: gym.Env,
    param_reader: ParameterReader,
    model_path: str,
    replay_buffer_path: str,
    test_mode: bool = False,
) -> tuple[BaseAlgorithm, dict]:
    match algorithm:
        case "DDPG":
            model = DDPG.load(
                model_path,
                env=env,
                learning_starts=int(param_reader.read("learning_starts", "DDPG")),
            )
            if replay_buffer_path:
                _load_replay_buffer(model, replay_buffer_path)
        case "TD3":
            model = DDPG.load(
                model_path,
                env=env,
                learning_starts=int(param_reader.read("learning_starts", "TD3")),
                batch_size=int(param_reader.read("batch_size", "TD3")),
            )

            # model.action_noise._target_sigma = 0.5 * np.ones(3)

            if replay_buffer_path:
                _load_replay_buffer(model, replay_buffer_path)

            model.actor.optimizer.param_groups[0]["lr"] = float(
                param_reader.read("actor_lr", "TD3")
            )
            model.critic.optimizer.param_groups[0]["lr"] = float(
                param_reader.read("critic_lr", "TD3")
            )

            # for param in model.actor.parameters():
            #     param.requires_grad = False

            # def weight_reset(m):
            #     if isinstance(m, nn.Linear):
            #         m.reset_parameters()

            # model.critic.apply(weight_reset)
            # model.critic_target.apply(weight_reset)
        case "PPO":
            model = PPO.load(model_path, env=env)
        case "SAC":
            model = SAC.load(model_path, env=env)
            if replay_buffer_path:
                _load_replay_buffer(model, replay_buffer_path)
        case _:
            raise ValueError(f"Unknown algorithm: {algorithm}")

    if not test_mode:
        wb_run_params = _get_wb_run_params(model, param_reader)
    else:
        wb_run_params = {}

    print(model.policy)
    return model, wb_run_params


def bootstrap_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="Tester script",
        description="Script for testing reinforcement learning models for WUT Velmwheel robot",
    )
    parser.add_argument(
        "-c",
        "--config",
        type=str,
        help="Path to the configuration file",
        required=False,
        default="config.ini",
    )
    parser.add_argument(
        "--gym_env", type=str, help="Name of the Gym environment", required=False
    )
    parser.add_argument(
        "--algorithm", type=str, help="Name of the algorithm", required=False
    )
    parser.add_argument("--model", type=str, help="Model to load path", required=False)
    parser.add_argument(
        "--replay_buffer", type=str, help="Replay buffer to load", required=False
    )
    parser.add_argument(
        "--navigation_difficulty_level",
        type=int,
        help="Navigation difficulty level",
        required=False,
    )
    parser.add_argument(
        "--real_time_factor",
        type=float,
        help="Real time factor for the simulation",
        required=False,
    )
    return parser
