import sys
import logging
import rclpy
from geometry_msgs.msg import Twist


logger = logging.getLogger(__name__)


class VelmwheelMovementController:
    def __init__(self):
        # Initialize and configure ROS2 node
        self._node = rclpy.create_node(self.__class__.__name__)

        self._movement_pub = self._node.create_publisher(
            Twist,
            "/velmwheel/base/velocity_setpoint",
            qos_profile=10,
        )
