import logging
import os
import pickle
import random

import numpy as np

from velmwheel_gym.types import NavigationDifficulty, Point

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
    def __init__(self, difficulty: NavigationDifficulty):
        self._starting_rotation = None
        self._starting_position = None
        self._goal = None
        self._idx = None
        self._combinations = [(x, y) for x in POINTS for y in POINTS if x != y]
        self._stats = [0] * len(self._combinations)
        self._statfile = f"state/stats_{os.getpid()}.pkl"
        self._difficulty = difficulty

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
        if self._difficulty.maneuvers:
            maneuver = random.SystemRandom().choice(self._difficulty.maneuvers)
            self._starting_position = maneuver[0]
            self._starting_rotation = random.SystemRandom().uniform(-np.pi, np.pi)
            self._goal = maneuver[1]
            return

        # points = [
        #     Point(3.0, 3.0),
        #     Point(3.0, -3.0),
        #     Point(-3.0, 3.0),
        #     Point(-3.0, -3.0),
        #     Point(7.0, 7.0),
        #     Point(-7.0, 7.0),
        #     Point(7.0, -7.0),
        #     Point(-7.0, -7.0),
        # ]
        # combos = [(x, y) for x in points for y in points if x != y]
        # combo = random.SystemRandom().choice(combos)
        # self._starting_position = combo[0]
        # self._goal = combo[1]
        # self._starting_rotation = random.SystemRandom().uniform(-np.pi, np.pi)

        sx = random.SystemRandom().uniform(*self._difficulty.starting_rect[0])
        sy = random.SystemRandom().uniform(*self._difficulty.starting_rect[1])
        self._starting_position = Point(sx, sy)
        self._starting_rotation = random.SystemRandom().uniform(-np.pi, np.pi)
        gx = random.SystemRandom().uniform(*self._difficulty.goal_rect[0])
        gy = random.SystemRandom().uniform(*self._difficulty.goal_rect[1])
        self._goal = Point(gx, gy)
