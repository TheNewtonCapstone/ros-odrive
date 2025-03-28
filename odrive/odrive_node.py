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
        # self.manager.load_configs_from_file(config_file_path)

        self_state_timer = self.create_timer(0.02, self.publish_joint_states)
        # create subs
        self.position_sub = self.create_subscription(
            Float32MultiArray,
            "joint_cmd_positions",
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

        self.manager.start(
            config_file_path=config_file_path,
            odrive_ready_cb=lambda ready: self.odrive_ready_pub.publish(
                Bool(data=ready)
            ),
        )
        # timer = self.create_timer(0.01, self.data_collection)
        # for i in range(12):
        #    self.manager.get_device(i).request_heartbeat()
        # self.manager.get_device(5).request_heartbeat()
        # self.manager.get_device(8).calibrate()
        # self.manager.get_device(8).set_controller_mode(control_mode=ControlMode.POSITION_CONTROL, input_mode=InputMode.TRAP_TRAJ)
        # self.manager.get_device(8).set_axis_state(AxisState.CLOSED_LOOP_CONTROL)
        # self.manager.calibrate_one(9)
        # print(self.manager.get_device(2).request_heartbeat())
        # print(self.manager.get_device(8).request_heartbeat())
        # print(self.manager.get_device(11).request_heartbeat())
        # self.manager.enumerate_devices()
        # self.manager.calibrate_all()
        # self.console.print(self.manager.get_device(2).request_heartbeat())

        # self.manager.initialize_all()
        # self.manager.calibrate_one(0)
        # self.manager.calibrate_one(1)
        # self.manager.arm_all()

        # pass in a gait
        # self.manager.set_all_positions(standing_gait)

    def position_callback(self, msg):
        # # command for position command messages
        # pprint(f"received position command {msg.data}")
        devices = self.manager.get_devices()

        for i in range(12):
            if i in devices:
                self.manager.set_position(node_id=i, position=msg.data[i])
                # self.console.print(f"Setting position for {i} to {msg.data[i]}")

        # self.manager.set_all_positions(msg.data)

    def data_collection(self):
        control_frequency = 50  # Hz - slower control updates
        control_period = 1.0 / control_frequency

        data_frequency = 100  # Hz - faster data collection
        data_period = 1.0 / data_frequency

        amplitudes = [0.4, 0.5, 0.6, 0.7, 0.8]
        frequencies = [1, 1.5, 2, 2.5, 3]

        standing_gait = {
            0: -0.8,
            1: 1.4,
            2: -0.8,
            3: 1.4,
            4: -0.8,
            5: 1.4,
            6: -0.8,
            7: 1.4,
        }
        new_gait = {
            0: 0.0,
            1: 0.0,
            2: 0.0,
            3: 0.0,
            4: 0.0,
            5: 0.0,
            6: 0.0,
            7: 0.0,
        }

        import csv

        with open("data.csv", mode="a") as file:
            writer = csv.writer(file)
            writer.writerow(
                [
                    "time",
                    "target_0",
                    "target_1",
                    "target_2",
                    "target_3",
                    "target_4",
                    "target_5",
                    "target_6",
                    "target_7",
                    "pos_0",
                    "pos_1",
                    "pos_2",
                    "pos_3",
                    "pos_4",
                    "pos_5",
                    "pos_6",
                    "pos_7",
                    "vel_0",
                    "vel_1",
                    "vel_2",
                    "vel_3",
                    "vel_4",
                    "vel_5",
                    "vel_6",
                    "vel_7",
                    "tor_0",
                    "tor_1",
                    "tor_2",
                    "tor_3",
                    "tor_4",
                    "tor_5",
                    "tor_6",
                    "tor_7",
                    "tor_est_0",
                    "tor_est_1",
                    "tor_est_2",
                    "tor_est_3",
                    "tor_est_4",
                    "tor_est_5",
                    "tor_est_6",
                    "tor_est_7",
                    "amp",
                    "freq",
                ]
            )

            devices = self.manager.get_devices().items()
            base_position = 0
            hfe_offset = 0
            kfe_offset = 0

            for amp in amplitudes:
                for freq in frequencies:
                    print(f"Testing amplitude {amp} and frequency {freq}")

                    experiment_duration = 5.0
                    start_experiment_time = time.time()

                    last_control_time = 0
                    last_data_time = 0

                    while time.time() - start_experiment_time < experiment_duration:
                        current_time = time.time()
                        elapsed_time = current_time - start_experiment_time

                        if current_time - last_control_time >= control_period:
                            base_position = amp * math.sin(
                                2 * math.pi * freq * elapsed_time
                            )
                            hfe_offset = amp * base_position
                            kfe_offset = base_position * -2.0 * amp

                            for i, device in devices:
                                if device.get_id() % 2 == 0:
                                    new_gait[i] = standing_gait[i] + hfe_offset
                                else:
                                    new_gait[i] = standing_gait[i] + kfe_offset

                            self.manager.set_positions_all(new_gait)
                            last_control_time = current_time

                        ## collect data
                        if (current_time - last_data_time) >= data_period:
                            positions = self.manager.get_position_all()
                            velocities = self.manager.get_velocities_all()
                            torques = self.manager.get_torque_all()
                            torques_est = self.manager.get_torque_estimate_all()
                            writer.writerow(
                                [
                                    current_time,
                                    new_gait[0],
                                    new_gait[1],
                                    new_gait[2],
                                    new_gait[3],
                                    new_gait[4],
                                    new_gait[5],
                                    new_gait[6],
                                    new_gait[7],
                                    positions[0],
                                    positions[1],
                                    positions[2],
                                    positions[3],
                                    positions[4],
                                    positions[5],
                                    positions[6],
                                    positions[7],
                                    velocities[0],
                                    velocities[1],
                                    velocities[2],
                                    velocities[3],
                                    velocities[4],
                                    velocities[5],
                                    velocities[6],
                                    velocities[7],
                                    torques[0],
                                    torques[1],
                                    torques[2],
                                    torques[3],
                                    torques[4],
                                    torques[5],
                                    torques[6],
                                    torques[7],
                                    torques_est[0],
                                    torques_est[1],
                                    torques_est[2],
                                    torques_est[3],
                                    torques_est[4],
                                    torques_est[5],
                                    torques_est[6],
                                    torques_est[7],
                                    amp,
                                    freq,
                                ]
                            )

                            last_data_time = current_time
                            last_data_time = current_time

                        time.sleep(0.01)
                    self.console.print(
                        f"[blue]Finished testing amplitude {amp} and frequency {freq}[blue]"
                    )

        # get all the torque estimates

    def publish_joint_states(self):
        if not self.manager.get_devices_calibrated():
            return

        positions_msg = Float32MultiArray()
        velocity_msg = Float32MultiArray()

        positions_msg.data = self.manager.get_position_all()
        velocity_msg.data = self.manager.get_velocities_all()

        self.position_pub.publish(positions_msg)
        self.velocity_pub.publish(velocity_msg)

    def shutdown(self):
        self.manager.estop_all()


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
