from collections import deque

import numpy as np

import wandb
from velmwheel_gym.constants import STATS_BUFFER_SIZE
from velmwheel_gym.types import NavigationDifficulty


class Metrics:
    def __init__(self):
        self._data = {
            "local_episode": 0,
            "global_episode": 0,
            "episode_reward": 0,
            "rewards": deque(maxlen=STATS_BUFFER_SIZE),
            "local_results": deque(maxlen=STATS_BUFFER_SIZE),
            "global_results": deque(maxlen=STATS_BUFFER_SIZE),
        }

    @property
    def mean_reward(self) -> float:
        if len(self._data["rewards"]) < STATS_BUFFER_SIZE:
            return -2.0
        return np.mean(self._data["rewards"])

    @property
    def global_success_rate(self) -> float:
        if len(self._data["global_results"]) < STATS_BUFFER_SIZE:
            return -2.0
        return np.mean(self._data["global_results"])

    @property
    def local_success_rate(self) -> float:
        if len(self._data["local_results"]) < STATS_BUFFER_SIZE:
            return -2.0
        return np.mean(self._data["local_results"])

    @property
    def episode_reward(self) -> float:
        return self._data["episode_reward"]

    @episode_reward.setter
    def episode_reward(self, value: float):
        self._data["episode_reward"] = value

    @property
    def global_episode(self) -> int:
        return self._data["global_episode"]

    def advance_to_next_level(self):
        self._data["rewards"].clear()
        self._data["local_results"].clear()
        self._data["global_results"].clear()

    def export(self, difficulty: NavigationDifficulty, terminated: bool):
        export_metrics = {
            "level": difficulty,
            "mean_reward": self.mean_reward,
            "local_success_rate": self.local_success_rate,
            "global_success_rate": self.global_success_rate,
        }
        if terminated:
            export_metrics["episode_reward"] = self._data["episode_reward"]
        wandb.log(export_metrics)

    def register_local_episode_start(self):
        self._data["local_episode"] += 1
        self._data["episode_reward"] = 0
        self._data["local_results"].append(0)

    def register_global_episode_start(self):
        self._data["global_episode"] += 1
        self._data["global_results"].append(0)

    def register_local_episode_result(self, result: int):
        self._data["local_results"][-1] = result
        self._data["rewards"].append(self._data["episode_reward"])

    def register_global_episode_result(self, result: int):
        self._data["global_results"][-1] = result
