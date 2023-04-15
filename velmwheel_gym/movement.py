import sys
import logging
import rclpy
from geometry_msgs.msg import Twist


logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)
handler = logging.StreamHandler(sys.stdout)
handler.setLevel(logging.DEBUG)
formatter = logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s")
handler.setFormatter(formatter)
logger.addHandler(handler)


class VelmwheelMovementController:
    def __init__(self):
        # Initialize and configure ROS2 node
        self._node = rclpy.create_node(self.__class__.__name__)

        self._movement_pub = self._node.create_publisher(
            Twist,
            "/velmwheel/base/velocity_setpoint",
            qos_profile=10,
        )
