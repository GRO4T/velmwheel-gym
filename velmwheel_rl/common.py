import argparse
import configparser
import os

import numpy as np
from gymnasium import gym
from stable_baselines3 import DDPG, PPO, SAC, TD3
from stable_baselines3.common.base_class import BaseAlgorithm
from stable_baselines3.common.noise import (
    NormalActionNoise,
    OrnsteinUhlenbeckActionNoise,
)


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

    def read(self, name: str):
        value = (
            vars(self._args)[name]
            if name in self._args and vars(self._args)[name] is not None
            else self._config.get(self._default_section, name)
        )
        print(f"{name}={value}")
        return value


def get_model_save_path_and_tb_log_name(
    algorithm: str, model_path: str
) -> tuple[str, str]:
    if model_path:
        return (
            os.path.dirname(model_path),
            f"{algorithm}_{os.path.basename(os.path.dirname(model_path))}",
        )

    model_save_dir = f"./models/velmwheel_v1/{algorithm.lower()}"
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


def create_model(algorithm: str, env: gym.Env) -> BaseAlgorithm:
    match algorithm:
        case "DDPG":
            n_actions = env.action_space.shape[-1]
            action_noise = OrnsteinUhlenbeckActionNoise(
                mean=np.zeros(n_actions), sigma=float(0.5) * np.ones(n_actions)
            )
            model = DDPG(
                "MlpPolicy",
                env,
                verbose=1,
                action_noise=action_noise,
                tensorboard_log="./logs/tensorboard",
                device="cuda",
            )
        case "TD3":
            n_actions = env.action_space.shape[-1]
            action_noise = NormalActionNoise(
                mean=np.zeros(n_actions), sigma=np.ones(n_actions)
            )
            policy_kwargs = dict(net_arch=dict(pi=[800, 600, 600], qf=[800, 600, 600]))
            model = TD3(
                "MlpPolicy",
                env,
                verbose=1,
                action_noise=action_noise,
                tensorboard_log="./logs/tensorboard",
                device="cuda",
                buffer_size=int(1e6),
                learning_starts=100,
                policy_kwargs=policy_kwargs,
                train_freq=(2, "episode"),
                batch_size=40,
                gamma=0.9999,
                seed=0,
                optimize_memory_usage=True,
                replay_buffer_kwargs=dict(handle_timeout_termination=False),
            )
        case "PPO":
            policy_kwargs = dict(net_arch=dict(pi=[512, 512], vf=[512, 512]))
            model = PPO(
                "MlpPolicy",
                env,
                verbose=1,
                tensorboard_log="./logs/tensorboard",
                device="cuda",
                policy_kwargs=policy_kwargs,
                gamma=0.9999,
                # n_steps=4096,
            )
        case "SAC":
            model = SAC(
                "MlpPolicy",
                env,
                verbose=1,
                tensorboard_log="./logs/tensorboard",
                device="cuda",
            )
        case _:
            raise ValueError(f"Unknown algorithm: {algorithm}")

    print(model.policy)
    return model


def _load_replay_buffer(model: BaseAlgorithm, replay_buffer_path: str):
    model.load_replay_buffer(replay_buffer_path)
    print(f"Replay buffer loaded: {model.replay_buffer.size()} transitions")


def load_model(
    algorithm: str,
    env: gym.Env,
    model_path: str,
    replay_buffer_path: str,
) -> BaseAlgorithm:
    match algorithm:
        case "DDPG":
            model = DDPG.load(model_path, env=env)
            if replay_buffer_path:
                _load_replay_buffer(model, replay_buffer_path)
        case "TD3":
            model = TD3.load(model_path, env=env)
            if replay_buffer_path:
                _load_replay_buffer(model, replay_buffer_path)
        case "PPO":
            model = PPO.load(model_path, env=env)
        case "SAC":
            model = SAC.load(model_path, env=env)
            if replay_buffer_path:
                _load_replay_buffer(model, replay_buffer_path)
        case _:
            raise ValueError(f"Unknown algorithm: {algorithm}")

    print(model.policy)
    return model


def bootstrap_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="Tester script",
        description="Script for testing reinforcement learning models for WUT Velmwheel robot",
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
        "--goal_reached_threshold",
        type=float,
        help="Minimum distance to goal to reach it",
        required=False,
    )
    parser.add_argument(
        "--real_time_factor",
        type=float,
        help="Real time factor for the simulation",
        required=False,
    )
    return parser
