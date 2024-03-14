import os
import subprocess

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty


class SimulationController(Node):
    def __init__(self):
        super().__init__("simulation_controller")

        self._sim_log = None
        self._sim = None
        self._nav_stack_log = None
        self._nav_stack = None

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

    def _start_sim(self, request, response):
        self.get_logger().info("Starting simulation")
        self._start_sim_processes()
        return response

    def _stop_sim(self, request, response):
        self.get_logger().info("Stopping simulation")
        self._stop_sim_processes()
        return response

    def _restart_sim(self, request, response):
        self.get_logger().info("Restarting simulation")
        self._stop_sim_processes()
        self._start_sim_processes()
        return response

    def _start_sim_processes(self):
        self._sim_log = open("sim.log", "w")  # TODO: try to use context manager
        self._sim = subprocess.Popen(
            "./start_sim.sh",
            shell=True,
            env=os.environ,
            stdout=self._sim_log,
            stderr=self._sim_log,
        )
        self._nav_stack_log = open("nav_stack.log", "w")
        self._nav_stack = subprocess.Popen(
            "./start_nav_stack.sh",
            shell=True,
            env=os.environ,
            stdout=self._nav_stack_log,
            stderr=self._nav_stack_log,
        )

    def _stop_sim_processes(self):
        subprocess.run("./kill_sim.sh", shell=True, check=True)


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