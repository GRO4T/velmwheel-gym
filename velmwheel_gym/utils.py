import logging
from collections import namedtuple
from typing import Optional

import rclpy
import rclpy.executors
from rclpy.node import Client, Node

logger = logging.getLogger(__name__)

RosServiceClientWrapper = namedtuple(
    "RosServiceClientWrapper", "node client request ros_topic"
)


def wait_for_service(service_client: Client, ros_topic: str):
    while not service_client.wait_for_service(timeout_sec=1.0):
        logger.debug(f"{ros_topic} service not available, waiting again...")


def create_ros_service_client(
    node: Node, service_interface: object, ros_topic: str
) -> RosServiceClientWrapper:
    client = node.create_client(service_interface, ros_topic)
    wait_for_service(client, ros_topic)
    request = service_interface.Request()
    return RosServiceClientWrapper(node, client, request, ros_topic)


def call_service(
    service_client: RosServiceClientWrapper, timeout_sec: float = 10.0
) -> Optional[object]:
    wait_for_service(service_client.client, service_client.ros_topic)
    node, client, request, ros_topic = service_client
    logger.debug(f"Call {ros_topic} with {request=}")
    future = client.call_async(request)
    try:
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
        response = future.result()
        logger.debug(f"Response from {ros_topic}: {response}")
        return response
    except rclpy.executors.TimeoutException as timeout_exception:
        logger.error(f"Service call to {ros_topic} timed out: {timeout_exception}")
        return None
