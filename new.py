import rclpy
from rclpy.executors import MultiThreadedExecutor
import os
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rich.console import Console
from .device_manager import ODriveManager
from .can_interface import CanInterface, AsyncCanInterface
import asyncio
from std_msgs.msg import Float32MultiArray, Bool
from sensor_msgs.msg import JointState
import numpy as np

class ODriveNode(Node):
    def __init__(self):
        """Initialize the ODrive CAN controller node"""
        super().__init__("odrive_node")
        self.console = Console()
        self.console.print("Initializing ODrive CAN controller node")
        
        # Initialize CAN interface and ODrive manager
        self.can_interface = CanInterface()
        self.manager = ODriveManager(self.can_interface)
        config_file_path = "config/newton.yaml"
        self.manager.load_configs_from_file(config_file_path)
        self.can_interface.start(self.manager.process_can_message)
        
        # Create subscribers
        self.position_sub = self.create_subscription(
            Float32MultiArray,
            'joint_positions',
            self.position_callback,
            10
        )
        
        self.estop_sub = self.create_subscription(
            Bool,
            'emergency_stop',
            self.estop_callback,
            10
        )
        
        # Create publishers
        self.joint_state_pub = self.create_publisher(
            JointState,
            'joint_states',
            10
        )
        
        # Timer for publishing joint states
        self.state_timer = self.create_timer(0.02, self.publish_joint_states)  # 50Hz
        
        # Initialize motors and set to standing position
        self.manager.initialize_all()
        self.manager.arm_all()
        
        # Default standing gait
        standing_gait = [0.0, 0.5, -1.0, 0.0, 0.5, -1.0, 0.0, 0.5, -1.0, 0.0, 0.5, -1.0]
        self.manager.set_all_positions(standing_gait)
    
    def position_callback(self, msg):
        """Handle incoming position commands"""
        positions = msg.data
        if len(positions) != len(self.manager.devices):
            self.console.print(f"[red]Received {len(positions)} positions, but have {len(self.manager.devices)} devices[/red]")
            return
        
        self.manager.set_all_positions(positions)
    
    def estop_callback(self, msg):
        """Handle emergency stop command"""
        if msg.data:
            self.console.print("[red]Emergency stop triggered[/red]")
            self.manager.estop_all()
    
    def publish_joint_states(self):
        """Publish current joint states"""
        if not self.manager.devices:
            return
            
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        
        # Add data for each motor
        for node_id, device in self.manager.devices.items():
            joint_state.name.append(device.name)
            joint_state.position.append(device.turns_to_rad(device.position))
            joint_state.velocity.append(device.velocity * device.TURNS_TO_RAD)
            joint_state.effort.append(device.torque_estimate)
        
        self.joint_state_pub.publish(joint_state)
    
    def shutdown(self):
        """Safely shut down all motors"""
        self.console.print("[yellow]Shutting down ODrive node...[/yellow]")
        self.manager.estop_all()
        self.can_interface.stop()

def main():
    rclpy.init()
    node = ODriveNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
    import rclpy
from rclpy.executors import MultiThreadedExecutor
import os
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rich.console import Console
from .device_manager import ODriveManager
from .can_interface import CanInterface, AsyncCanInterface
import asyncio
from std_msgs.msg import Float32MultiArray, Bool
from sensor_msgs.msg import JointState
import numpy as np

class ODriveNode(Node):
    def __init__(self):
        """Initialize the ODrive CAN controller node"""
        super().__init__("odrive_node")
        self.console = Console()
        self.console.print("Initializing ODrive CAN controller node")
        
        # Initialize CAN interface and ODrive manager
        self.can_interface = CanInterface()
        self.manager = ODriveManager(self.can_interface)
        config_file_path = "config/newton.yaml"
        self.manager.load_configs_from_file(config_file_path)
        self.can_interface.start(self.manager.process_can_message)
        
        # Create subscribers
        self.position_sub = self.create_subscription(
            Float32MultiArray,
            'joint_positions',
            self.position_callback,
            10
        )
        
        self.estop_sub = self.create_subscription(
            Bool,
            'emergency_stop',
            self.estop_callback,
            10
        )
        
        # Create publishers
        self.joint_state_pub = self.create_publisher(
            JointState,
            'joint_states',
            10
        )
        
        # Additional publisher for raw positions
        self.raw_positions_pub = self.create_publisher(
            Float32MultiArray,
            'raw_motor_positions',
            10
        )
        
        # Additional publisher for motor states
        self.motor_states_pub = self.create_publisher(
            Float32MultiArray, 
            'motor_states',
            10
        )
        
        # Create timers for publishing data
        self.state_timer = self.create_timer(0.02, self.publish_joint_states)  # 50Hz
        self.raw_data_timer = self.create_timer(0.01, self.publish_raw_data)   # 100Hz
        
        # Initialize motors and set to standing position
        self.manager.initialize_all()
        self.manager.arm_all()
        
        # Default standing gait
        standing_gait = [0.0, 0.5, -1.0, 0.0, 0.5, -1.0, 0.0, 0.5, -1.0, 0.0, 0.5, -1.0]
        self.manager.set_all_positions(standing_gait)
    
    def position_callback(self, msg):
        """Handle incoming position commands"""
        positions = msg.data
        if len(positions) != len(self.manager.devices):
            self.console.print(f"[red]Received {len(positions)} positions, but have {len(self.manager.devices)} devices[/red]")
            return
        
        self.manager.set_all_positions(positions)
    
    def estop_callback(self, msg):
        """Handle emergency stop command"""
        if msg.data:
            self.console.print("[red]Emergency stop triggered[/red]")
            self.manager.estop_all()
    
    def publish_joint_states(self):
        """Publish current joint states in standard ROS format"""
        if not self.manager.devices:
            return
            
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        
        # Add data for each motor
        for node_id, device in self.manager.devices.items():
            joint_state.name.append(device.name)
            joint_state.position.append(device.turns_to_rad(device.position))
            joint_state.velocity.append(device.velocity * device.TURNS_TO_RAD)
            joint_state.effort.append(device.torque_estimate)
        
        self.joint_state_pub.publish(joint_state)
    
    def publish_raw_data(self):
        """Publish raw motor positions and states"""
        if not self.manager.devices:
            return
            
        # Create message for raw positions
        positions_msg = Float32MultiArray()
        positions = []
        
        # Create message for motor states (includes positions, velocities, errors, states)
        states_msg = Float32MultiArray()
        states = []
        
        # Sort devices by node_id to ensure consistent ordering
        ordered_devices = sorted(self.manager.devices.items(), key=lambda x: x[0])
        
        for node_id, device in ordered_devices:
            # Add raw position to positions message
            positions.append(device.position)  # Raw position in turns
            
            # Add comprehensive motor state
            states.extend([
                device.position,                # Raw position
                device.velocity,                # Raw velocity
                float(device.axis_error),       # Error code
                float(device.axis_state.value), # State enum value
                device.torque_estimate,         # Torque estimate
                device.torque_target,           # Target torque
                float(device.procedure_result), # Procedure result
                float(device.trajectory_done)   # Trajectory completion flag
            ])
        
        # Publish raw positions
        positions_msg.data = positions
        self.raw_positions_pub.publish(positions_msg)
        
        # Publish comprehensive state data
        states_msg.data = states
        self.motor_states_pub.publish(states_msg)
    
    def shutdown(self):
        """Safely shut down all motors"""
        self.console.print("[yellow]Shutting down ODrive node...[/yellow]")
        self.manager.estop_all()
        self.can_interface.stop()

def main():
    rclpy.init()
    node = ODriveNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()