import rclpy
from rclpy.executors import MultiThreadedExecutor
import os
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rich.console import Console
from .device_manager import ODriveManager
from .can_interface import CanInterface, AsyncCanInterface
from std_msgs.msg import Float32MultiArray, Bool
from sensor_msgs.msg import JointState
from pprint import pprint
# import numpy as np
import asyncio

class ODriveNode(Node):
    def __init__(self):
        """Initialize the ODrive CAN controller node"""

        super().__init__("odrive_node")
        self.console = Console()
        self.console.print("Initializing ODrive CAN controller node")
        self.can_interface = CanInterface()
        self.manager = ODriveManager(self.can_interface)
        config_file_path = "config/newton.yaml"
        self.manager.load_configs_from_file(config_file_path)
        
        
        
        # create subs 
        self.position_sub = self.create_subscription(
            Float32MultiArray, 
            'joints_cmd_positions', 
            self.position_callback, 10
        )
        
        
        self.position_pub = self.create_publisher(
            Float32MultiArray, 
            'joints_state_positions', 
            10
        )
        self.velocity_pub = self.create_publisher(
            Float32MultiArray, 
            'joint_state_velocities', 
            10
        )
        self.joint_state_pub = self.create_publisher(
            Float32MultiArray, 
            'joints_state_velocities', 
            10
        )
        # TODO: MAKE A VAR FOR THIS RATE
        self_state_timer = self.create_timer(0.009, self.publish_joint_states)
    
        self.can_interface.start(self.manager.process_can_message)
        # self.manager.calibrate_all()
        # self.manager.initialize_all()
        # self.manager.arm_all()
        

        # pass in a gait 
        standing_gait = [0, 0.5, -1, 0.0, 0.5, -1, 0.0, 0.5, -1, 0.0, 0.5, -1]
        standing_gait = [0, -0.5, 1]
        self.manager.set_all_positions(standing_gait)

    
    def position_callback(self, msg):
        # command for position command messages
        self.manager.set_all_positions(msg.data)

    def publish_joint_states(self):
        # publish joint states
        if not self.manager.devices:
            return
        positions_msg = Float32MultiArray()
        velocity_msg = Float32MultiArray()

        positions_msg.data = self.manager.get_all_positions()
        velocity_msg.data = self.manager.get_all_velocities()
        
        

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
