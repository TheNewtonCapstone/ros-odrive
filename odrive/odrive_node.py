import math
import os
from ament_index_python import get_package_share_directory
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rich.console import Console
from .can_interface import CanInterface
from .device_manager import ODriveManager
from std_msgs.msg import Float32MultiArray, Bool
from pprint import pprint
import time

# import numpy as np


class ODriveNode(Node):
    def __init__(self):
        """Initialize the ODrive CAN controller node"""

        super().__init__("odrive_node")
        self.console = Console()
        self.console.print("[bold green]ODrive Node[/bold green]")
        self.can_interface = CanInterface()
        self.manager = ODriveManager(self.can_interface)

        share_directory = get_package_share_directory("n_odrive")
        config_file_path = os.path.join(share_directory, "config/newton.yaml")
        self.manager.start(config_file_path=config_file_path)

        # create subs
        self.position_sub = self.create_subscription(
            Float32MultiArray,
            "joints_cmd_positions",
            self.position_callback,
            10,
        )

        self.position_pub = self.create_publisher(
            Float32MultiArray,
            "joint_state_positions",
            10,
        )
        self.velocity_pub = self.create_publisher(
            Float32MultiArray,
            "joint_state_velocities",
            10,
        )
        self.odrive_ready_pub = self.create_publisher(
            Bool,
            "odrive_ready",
            10,
        )
        # TODO: MAKE A VAR FOR THIS RATE

        self_state_timer = self.create_timer(0.02, self.publish_joint_states)

        self.odrive_ready_pub.publish(
            Bool(
                data=self.manager.get_devices_calibrated(),
            )
        )


    def position_callback(self, msg):
        # # command for position command messages
        pprint(f"received position command {msg.data}")
        devices = self.manager.get_devices()
        size = msg.layout.dim[0].size
        # added this as a hack to get the position of the joints as quick. a better way would be to create 
        # ros2 message types for the joints

        for i in range(size):
            if i in devices:
                self.manager.set_position(node_id=i, position=msg.data[i])
                # self.console.print(f"Setting position for {i} to {msg.data[i]}")

        # self.manager.set_all_positions(msg.data)

    def publish_joint_states(self):
        if not self.manager.get_devices_calibrated():
            return

        positions_msg = Float32MultiArray()
        velocity_msg = Float32MultiArray()

        positions_msg.data = self.manager.get_position_all()
        velocity_msg.data = self.manager.get_all_velocities()

        self.position_pub.publish(positions_msg)
        self.velocity_pub.publish(velocity_msg)
        pprint(f"published positions {positions_msg.data}")
        pprint(f"published velocities {velocity_msg.data}")

        return

    def shutdown(self):
        self.manager.estop_all()

        pass


def main():
    rclpy.init()
    node = ODriveNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except ExternalShutdownException:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
