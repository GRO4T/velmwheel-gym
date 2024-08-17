import logging
import os
import pickle
import random

import numpy as np

from velmwheel_gym.types import Point

logger = logging.getLogger(__name__)


POINTS = [
    Point(0.0, 0.0),
    Point(3.0, 3.0),
    Point(3.0, -3.0),
    Point(-3.0, 3.0),
    Point(-3.0, -3.0),
    Point(7.0, 7.0),
    Point(-7.0, 7.0),
    Point(7.0, -7.0),
    Point(-7.0, -7.0),
]


class StartPositionAndGoalGenerator:
    def __init__(self):
        self._starting_rotation = None
        self._starting_position = None
        self._goal = None
        self._idx = None
        self._combinations = [(x, y) for x in POINTS for y in POINTS if x != y]
        self._stats = [0] * len(self._combinations)
        self._statfile = f"state/stats_{os.getpid()}.pkl"

        if os.path.exists(self._statfile):
            with open(self._statfile, "rb") as f:
                self._stats = pickle.load(f)
            logger.debug("Loaded stats")
            for i, (x, y) in enumerate(self._combinations):
                logger.debug(f"{x} -> {y}: {self._stats[i]}")

    def set(self, starting_position: Point, goal: Point):
        self._starting_position = starting_position
        self._goal = goal
        self._idx = self._combinations.index((starting_position, goal))

    @property
    def starting_position(self):
        return self._starting_position

    @starting_position.setter
    def starting_position(self, starting_position):
        self._starting_position = starting_position
        self._idx = self._combinations.index((starting_position, self._goal))

    @property
    def goal(self):
        return self._goal

    @goal.setter
    def goal(self, goal):
        self._goal = goal
        self._idx = self._combinations.index((self._starting_position, goal))

    @property
    def starting_rotation(self):
        return self._starting_rotation

    def register_goal_reached(self):
        pass
        # self._stats[self._idx] += 1
        # with open(self._statfile, "wb") as f:
        #     pickle.dump(self._stats, f)

    def generate_next(self):
        sx = random.SystemRandom().uniform(-2.0, 2.0)
        sy = random.SystemRandom().uniform(-2.0, 2.0)
        # sx = random.SystemRandom().uniform(-4.0, 4.0)
        # sy = random.SystemRandom().uniform(-4.0, 4.0)
        self._starting_position = Point(sx, sy)
        self._starting_rotation = random.SystemRandom().uniform(-np.pi, np.pi)
        # self._goal = random.SystemRandom().choice(POINTS)
        # gx = random.SystemRandom().uniform(-2.0, 2.0)
        # gy = random.SystemRandom().uniform(-2.0, 2.0)
        # self._goal = Point(gx, gy)
        self._goal = Point(0.0, 0.0)

        # inverse_stats = [1 / (1 + x) for x in self._stats]
        # total = sum(inverse_stats)
        # probabilities = [x / total for x in inverse_stats]
        # self._idx = random.SystemRandom().choices(
        #     range(len(self._combinations)), weights=probabilities, k=1
        # )[0]
        # self._starting_position = self._combinations[self._idx][0]
        # self._goal = self._combinations[self._idx][1]

        # logger.debug("Stats")
        # for i, (x, y) in enumerate(self._combinations):
        #     logger.debug(f"{x} -> {y}: {self._stats[i]}")
        # logger.debug(f"Probabilities: {probabilities}")
        # logger.debug(f"Chosen starting position: {self._starting_position}")
        # logger.debug(f"Chosen goal: {self._goal}")
