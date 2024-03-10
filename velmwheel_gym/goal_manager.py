import logging
import random

from velmwheel_gym.types import Point

logger = logging.getLogger(__name__)


class GoalManager:
    def __init__(self):
        self._current_goal = None
        self._current_goal_idx = 0
        self._available_goals = [
            Point(3.0, 3.0),
            Point(3.0, -3.0),
            Point(-3.0, 3.0),
            Point(-3.0, -3.0),
        ]
        self._stats = [0] * len(self._available_goals)

    @property
    def current_goal(self):
        return self._current_goal

    @current_goal.setter
    def current_goal(self, value):
        self._current_goal = value
        self._current_goal_idx = self._available_goals.index(value)

    def register_goal_reached(self):
        self._stats[self._current_goal_idx] += 1

    def generate_next_goal(self):
        inverse_stats = [1 / (1 + x) for x in self._stats]
        total = sum(inverse_stats)
        probabilities = [x / total for x in inverse_stats]
        self._current_goal_idx = random.SystemRandom().choices(
            range(len(self._available_goals)), weights=probabilities, k=1
        )[0]
        self._current_goal = self._available_goals[self._current_goal_idx]

        logger.debug(f"Stats: {self._stats}")
        logger.debug(f"Probabilities: {probabilities}")
        logger.debug(f"Chosen goal: {self._current_goal}")
