# pylint: disable=consider-using-with
import os
import subprocess

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty, Trigger


# pylint: disable=too-few-public-methods
class SimulationController(Node):
    def __init__(self):
        super().__init__("simulation_controller")

        self._sim_log = None
        self._sim = None
        self._nav_stack_log = None
        self._nav_stack = None

        self._is_sim_running_srv = self.create_service(
            Trigger, "/is_sim_running", self._is_sim_running
        )
        self._start_sim_srv = self.create_service(Empty, "/start_sim", self._start_sim)
        self._stop_sim_srv = self.create_service(Empty, "/stop_sim", self._stop_sim)
        self._restart_sim_srv = self.create_service(
            Empty, "/restart_sim", self._restart_sim
        )

    def destroy_node(self):
        if self._sim_log is not None:
            self._sim_log.close()
        if self._nav_stack_log is not None:
            self._nav_stack_log.close()
        super().destroy_node()

    def _is_sim_running(self, request, response):  # pylint: disable=unused-argument
        # Sim is running if there is a process named "velmwheel_bringup"
        try:
            subprocess.check_output(
                "ps -ef | grep velmwheel_bringup | grep -v grep", shell=True
            )
            is_sim_running = True
        except subprocess.CalledProcessError as _:
            is_sim_running = False

        self.get_logger().info(f"Is simulation running: {is_sim_running}")
        response.success = is_sim_running
        return response

    def _start_sim(self, request, response):  # pylint: disable=unused-argument
        self.get_logger().info("Starting simulation")
        self._start_sim_processes()
        return response

    def _stop_sim(self, request, response):  # pylint: disable=unused-argument
        self.get_logger().info("Stopping simulation")
        self._stop_sim_processes()
        return response

    def _restart_sim(self, request, response):  # pylint: disable=unused-argument
        self.get_logger().info("Restarting simulation")
        while not self._stop_sim_processes():
            self.get_logger().info("Failed to stop simulation, retrying")
        self._start_sim_processes()
        return response

    def _start_sim_processes(self):
        self._sim_log = open(
            "sim.log", "w", encoding="utf-8"
        )  # TODO: try to use context manager
        self._sim = subprocess.Popen(
            "./start_sim.sh",
            shell=True,
            env=os.environ,
            stdout=self._sim_log,
            stderr=self._sim_log,
        )
        self._nav_stack_log = open("nav_stack.log", "w", encoding="utf-8")
        self._nav_stack = subprocess.Popen(
            "./start_nav_stack.sh",
            shell=True,
            env=os.environ,
            stdout=self._nav_stack_log,
            stderr=self._nav_stack_log,
        )

    def _stop_sim_processes(self) -> bool:
        try:
            subprocess.run("./kill_sim.sh", shell=True, check=True)
            self._sim = None
            return True
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"Failed to kill simulation processes: {e}")
        return False


def main(args=None):
    rclpy.init(args=args)

    node = SimulationController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
