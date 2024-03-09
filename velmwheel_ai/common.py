import argparse
import configparser
import os

import gym
import numpy as np
from stable_baselines3 import DDPG, PPO, SAC, TD3
from stable_baselines3.common.base_class import BaseAlgorithm
from stable_baselines3.common.noise import (
    NormalActionNoise,
    OrnsteinUhlenbeckActionNoise,
)


class ParameterReader:
    def __init__(self, args: argparse.Namespace, config: configparser.ConfigParser):
        self._args = args
        self._config = config

    def read(self, name: str):
        value = self._config.get("trainer", name) if self._config else self._args[name]
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
    run_id = 1
    for _ in os.listdir(model_save_dir):
        run_id += 1

    return os.path.join(model_save_dir, str(run_id)), f"{algorithm}_{run_id}"


def create_model(algorithm: str, env: gym.Env) -> BaseAlgorithm:
    match algorithm:
        case "DDPG":
            n_actions = env.action_space.shape[-1]
            action_noise = OrnsteinUhlenbeckActionNoise(
                mean=np.zeros(n_actions), sigma=float(0.5) * np.ones(n_actions)
            )
            return DDPG(
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
                mean=np.zeros(n_actions), sigma=0.1 * np.ones(n_actions)
            )
            return TD3(
                "MlpPolicy",
                env,
                verbose=1,
                action_noise=action_noise,
                tensorboard_log="./logs/tensorboard",
                device="cuda",
            )
        case "PPO":
            return PPO(
                "MlpPolicy",
                env,
                verbose=1,
                tensorboard_log="./logs/tensorboard",
                device="cuda",
            )
        case "SAC":
            return SAC(
                "MlpPolicy",
                env,
                verbose=1,
                tensorboard_log="./logs/tensorboard",
                device="cuda",
            )
        case _:
            raise ValueError(f"Unknown algorithm: {algorithm}")


def _load_replay_buffer(model: BaseAlgorithm, replay_buffer_path: str):
    model.load_replay_buffer(replay_buffer_path)
    print(f"Replay buffer loaded: {model.replay_buffer.size()} transitions")


def load_model(
    algorithm: str, env: gym.Env, model_path: str, replay_buffer_path: str
) -> BaseAlgorithm:
    match algorithm:
        case "DDPG":
            model = DDPG.load(model_path, env=env)
            if replay_buffer_path:
                _load_replay_buffer(model, replay_buffer_path)
            return model
        case "TD3":
            model = TD3.load(model_path, env=env)
            if replay_buffer_path:
                _load_replay_buffer(model, replay_buffer_path)
            return model
        case "PPO":
            return PPO.load(model_path, env=env)
        case "SAC":
            model = SAC.load(model_path, env=env)
            if replay_buffer_path:
                _load_replay_buffer(model, replay_buffer_path)
            return model
        case _:
            raise ValueError(f"Unknown algorithm: {algorithm}")


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
