import math
import logging
import sys
from velmwheel_gym.robot import VelmwheelRobot


logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)
handler = logging.StreamHandler(sys.stdout)
handler.setLevel(logging.DEBUG)
formatter = logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s")
handler.setFormatter(formatter)
logger.addHandler(handler)


class VelmwheelReward:
    def __init__(self, robot: VelmwheelRobot):
        self._robot = robot
        self._dist = None

    def calculate(self) -> float:
        if self._robot.is_collide():
            return -100.0

        robot_position = self._robot.get_position()
        robot_position = [robot_position.x, robot_position.y]
        target = [3, 3]

        self._dist = math.dist(robot_position, target)

        if self._dist < 1.0:
            logger.debug("Success!")
            return 200.0

        return -0.001

    def is_done(self) -> bool:
        return self._robot.is_collide() or self._dist < 1.0