import math
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rich.console import Console
from .can_interface import CanInterface
from .device_manager import ODriveManager
from std_msgs.msg import Float32MultiArray
from pprint import pprint
import time

# import numpy as np


class ODriveNode(Node):
    def __init__(self):
        """Initialize the ODrive CAN controller node"""

        super().__init__("odeive_node")
        self.console = Console()
        self.console.print("Initializing ODrive CAN controller node")
        self.can_interface = CanInterface()
        self.manager = ODriveManager(self.can_interface)
        config_file_path = "config/newton.yaml"
        self.manager.load_configs_from_file(config_file_path)

        self_state_timer = self.create_timer(0.01, self.publish_joint_states)
        # create subs
        self.position_sub = self.create_subscription(
            Float32MultiArray, "joints_cmd_positions", self.position_callback, 10
        )


        self.position_pub = self.create_publisher(
            Float32MultiArray, "joints_state_positions", 10
        )
        self.velocity_pub = self.create_publisher(
            Float32MultiArray, "joint_state_velocities", 10
        )
        self.joint_state_pub = self.create_publisher(
            Float32MultiArray, "joints_state_velocities", 10
        )
        # TODO: MAKE A VAR FOR THIS RATE

        self.can_interface.start(self.manager.process_can_message)
        time.sleep(1)
        # self.manager.get_device(8).request_heartbeat() 
        # self.manager.calibrate_one(9)
        self.manager.calibrate_all()
        # print(self.manager.get_device(2).request_heartbeat())
        

        # self.manager.initialize_all()
        # self.manager.calibrate_one(0)
        # self.manager.calibrate_one(1)
        # self.manager.arm_all()

        # pass in a gait
        standing_gait = { 
            0: 0.0,
            1:-0.5,
            2: 1, 
            3: 0.0,
            4: -0.5 ,
            5: 1,
            6: 0.0,
            7: - 0.5,
            8: 1,
            9: 0.0,
            10: -0.5,
            11: 1
            }
        self.manager.set_all_positions(standing_gait)

        

    def position_callback(self, msg):
        # # command for position command messages
        pprint(f"received position command {msg.data}")
        devices = self.manager.get_devices()
        for i in range(11):
            if i in devices:
                self.manager.set_position(node_id=i, position=msg.data[i])
        # self.manager.set_all_positions(msg.data)

    def publish_joint_states(self):
        if not self.manager.devices:
            return
        positions_msg = Float32MultiArray()
        velocity_msg = Float32MultiArray()

        positions_msg.data = self.manager.get_all_positions()
        velocity_msg.data = self.manager.get_all_velocities()
        pprint(positions_msg.data)

        self.position_pub.publish(positions_msg)
        self.velocity_pub.publish(velocity_msg)


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
