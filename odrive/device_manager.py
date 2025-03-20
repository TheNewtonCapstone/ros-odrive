import time
from typing import Dict, List, Optional, Callable
from rich.console import Console

from odrive.error import ODriveErrorCode, ODriveProcedureResult

from .device import ODriveDevice, AxisState, ControlMode, InputMode, OdriveCANCommands
from .can_interface import CanInterface, Arbitration
from .exception import CalibrationFailedException, DeviceNotFoundException
import yaml
import os
import rclpy
import pprint

console = Console()


class ODriveManager:
    """
    Manager class for handling multiple ODrive devices on a CAN bus
    """

    def __init__(self, can_interface):
        """
        Args:
            can_interface: CAN interface for communication
        """
        self.can_interface = can_interface
        self.devices: Dict[int, ODriveDevice] = {}  # node_id -> device
        self.running = False
        self.motor_calibrated = False

    def start(self, config_file_path: str) -> None:
        """
        Start the manager and all devices
        """
        self.can_interface.start(self.process_can_message)
        # check if the devices in the config file match the discovered devices
        # discovered_nodes = self.enumerate_devices()
        # for node_id, device in self.devices.items():
        #     if node_id not in discovered_nodes:
        #         raise Exception(
        #             f"Device with node_id {node_id} in config file not found"
        #         )

    def load_configs_from_file(self, config_file_path: str) -> None:
        config = {}
        try:
            with open(config_file_path, "r") as file:
                config = yaml.safe_load(file)
            motor_params = config.get("motors_params", {})

            for motor_name, motor_params in config["motors_params"].items():
                if motor_params.get("enabled", True):
                    node_id = motor_params["node_id"]
                    name = motor_params["name"].lower()
                    direction = motor_params["direction"]
                    position_limit = motor_params["position_limit"]
                    gear_ratio = motor_params["gear_ratio"]

                    self.add_device(
                        node_id=node_id,
                        name=name,
                        direction=direction,
                        position_limit=position_limit,
                        gear_ratio=gear_ratio,
                    )

            # print all the added devices
            for node_id, device in self.devices.items():
                console.print(
                    f"[green]Added device {device.name} with node_id {node_id}[/green]"
                )

        except FileNotFoundError:
            console.print(
                f"[red]Failed to load config file: {config_file_path}. Current working directory is {os.getcwd()}[/red]"
            )
            self.stop()
            return

    def add_device(
        self,
        node_id: int,
        name: str,
        direction: float,
        position_limit: float,
        gear_ratio: float,
    ) -> ODriveDevice:
        if node_id in self.devices:
            console.print(
                f"[yellow]Warning: Device with node_id {node_id} already exists. Returning existing device.[/yellow]"
            )
            return self.devices[node_id]

        device = ODriveDevice(
            node_id,
            name,
            direction,
            position_limit,
            gear_ratio,
            self.send_can_frame,
            self.request,
        )

        self.devices[node_id] = device
        return device

    def remove_device(self, node_id: int) -> bool:
        if node_id in self.devices:
            del self.devices[node_id]
            return True
        return False

    def get_all_positions(self) -> List[float]:
        """
        Get positions of all devices
        """
        positions = []

        # loop and position missinf from - 0 to 11
        for i in range(11):
            if i in self.devices:
                positions.append(self.devices[i].get_position())
            else:
                positions.append(0.0)
        return positions

    def get_all_velocities(self) -> List[float]:
        """
        Get velocities of all devices
        """
        velocities = []
        for i in range(11):
            if i in self.devices:
                velocities.append(self.devices[i].get_velocity())
            else:
                velocities.append(0.0)
        return velocities

    def get_device(self, node_id: int) -> Optional[ODriveDevice]:
        return self.devices.get(node_id)

    def get_devices(self) -> Dict[int, ODriveDevice]:
        return self.devices

    def request(
        self, node_id: int, cmd_id: int, data: bytes, timeout: float = 2.0
    ) -> bool:
        """
        Send a request message to a device

        Args:
            node_id: Node ID of the device
            cmd_id: Command ID
            data: Request data

        Returns:
            bool: True if successful, False otherwise
        """
        return self.can_interface.request(node_id, cmd_id, data)

    def send_can_frame(self, arbitration_id: int, data: bytes) -> bool:
        """
        Send a CAN frame

        Args:
            arbitration_id: CAN arbitration ID
            data: Data to send

        Returns:
            bool: True if successful, False otherwise
        """
        return self.can_interface.send_frame(arbitration_id, data)

    def process_can_message(self, node_id: int, cmd_id: int, data: bytearray) -> None:
        """
        Process an incoming CAN message

        Args:
            node_id: Node ID of the sender
            cmd_id: Command ID of the message
            data: Message data
        """
        device = self.get_device(node_id)
        if device:
            device.process_can_message(cmd_id, data)
        else:
            # pprint.pprint(self.devices)
            # console.print(
            #     f"[dim]Received message from unknown device: node_id={node_id}, cmd_id={cmd_id}[/dim]"
            # )
            pass

    def stop(self) -> None:
        """
        Stop the manager and all devices
        """
        console.print("[blue]Stopping ODrive manager...[/blue]")
        self.running = False
        # for device in self.devices.values():
        #     device.estop()

        self.can_interface.stop()

    def enumerate_devices(self, timeout: float = 3.0) -> List[int]:
        """
        Attempt to discover ODrive devices on the bus by requesting heartbeat
        messages from all possible node IDs from odrive can_enumerate.py

        Args:
            timeout: Time to wait for responses in seconds

        Returns:
            List[int]: List of discovered node IDs
        """
        # Maximum node ID in ODrive CAN protocol is 0x3E (62)
        max_node_id = 0x3E
        discovered_nodes = []

        # Create temporary callback to record responses
        original_callback = self.can_interface.callback

        def discovery_callback(node_id: int, cmd_id: int, data: bytes) -> None:
            if cmd_id == OdriveCANCommands.GET_HEARTBEAT:
                if node_id not in discovered_nodes:
                    discovered_nodes.append(node_id)
                    console.print(
                        f"[green]Discovered ODrive with node_id {node_id}[/green]"
                    )

        try:
            # Set discovery callback
            self.can_interface.callback = discovery_callback

            # Request heartbeat from all possible node IDs
            console.print("[blue]Enumerating ODrive devices...[/blue]")
            for node_id in range(max_node_id + 1):
                arb_id = (
                    node_id << Arbitration.NODE_ID_SIZE
                ) | OdriveCANCommands.GET_HEARTBEAT
                self.can_interface.send_frame(arb_id, b"")

            # Wait for responses
            start_time = time.time()
            while time.time() - start_time < timeout:
                time.sleep(0.1)

        finally:
            # Restore original callback
            self.can_interface.callback = original_callback

        return discovered_nodes

    def calibrate_one(self, node_id):
        device = self.devices.get(node_id)
        if not device:
            raise DeviceNotFoundException(f"Device with node_id {node_id} not found")

        console.print(f"[blue]Starting calibration of {device.name}...[/blue]")

        err, state, procedure_res, _ = device.calibrate()

        if (
            err != ODriveErrorCode.NO_ERROR
            or procedure_res != ODriveProcedureResult.SUCCESS
        ):
            self.stop()

        return err, state, procedure_res, _

    def initialize_all(
        self,
        control_mode: ControlMode = ControlMode.POSITION_CONTROL,
        input_mode: InputMode = InputMode.TRAP_TRAJ,
        closed_loop: bool = True,
        calibrate: bool = True,
        set_zero_pos: bool = True,
    ) -> None:
        """
        Initialize all devices with common settings

        Args:
            control_mode: Control mode to set
            input_mode: Input mode to set
            closed_loop: Whether to enter closed loop control
        """

        # clear errors
        for node_id, device in self.devices.items():
            if not device.clear_errors():
                console.print(
                    f"[yellow]Failed to clear errors for device {node_id}[/yellow]"
                )
                time.sleep(0.5)
                continue

        if set_zero_pos:
            # set current position as zero
            for node_id, device in self.devices.items():
                if not device.set_absolute_position(0.0):
                    console.print(
                        f"[yellow]Failed to set zero position for device {node_id}[/yellow]"
                    )
                    time.sleep(0.5)
                    continue

        self.calibrate_all()
        time.sleep(3.0)

    def set_all_positions(self, positions: dict[int, float]) -> None:
        """
        Set positions for all devices

        Args:
            positions: List of positions to set
        """
        # get list of available devices
        available_devices = list(self.devices.keys())

        # get the array of avaialble devices
        for i, (node_id, device) in enumerate(self.devices.items()):
            if i < len(positions):
                if node_id in available_devices:
                    device.set_position(positions[i])
            else:
                console.print(
                    f"[yellow]Not enough positions provided for device {node_id}[/yellow]"
                )

    def arm_all(self) -> None:
        # set control mode
        for node_id, device in self.devices.items():
            if not device.set_axis_state(AxisState.CLOSED_LOOP_CONTROL):
                console.print(
                    f"[red]Failed to enter closed loop control for device {node_id}[/red]"
                )
                time.sleep(0.5)
                continue

    def init_device(
        self,
        node_id: int,
        control_mode: ControlMode = ControlMode.POSITION_CONTROL,
        input_mode: InputMode = InputMode.TRAP_TRAJ,
        closed_loop: bool = True,
        set_zero_pos: bool = True,
    ) -> None:
        """
        Initialize all devices with common settings

        Args:
            control_mode: Control mode to set
            input_mode: Input mode to set
            closed_loop: Whether to enter closed loop control
        """
        device = self.devices.get(node_id)

        if not device:
            console.print(f"[red]Device with node_id {node_id} not found[/red]")
            return

        # Clear errors
        if not device.clear_errors():
            console.print(
                f"[yellow]Failed to clear errors for device {node_id}[/yellow]"
            )
            return

        # Set zero position
        if set_zero_pos:
            if not device.set_zero_position():
                console.print(
                    f"[yellow]Failed to set zero position for device {node_id}[/yellow]"
                )
                return

        # Set control mode
        if not device.set_controller_mode(control_mode, input_mode):
            console.print(
                f"[red]Failed to set controller mode for device {node_id}[/red]"
            )
            return

        # Enter closed loop control if requested
        if closed_loop:
            if not device.set_axis_state(AxisState.CLOSED_LOOP_CONTROL):
                console.print(
                    f"[red]Failed to enter closed loop control for device {node_id}[/red]"
                )
                return

        console.print(f"[green]Initialized device {node_id}[/green]")

    def estop_all(self) -> None:
        """
        Emergency stop all devices
        """
        for node_id, device in self.devices.items():
            if not device.estop():
                console.print(f"[red]Failed to estop device {node_id}[/red]")
            else:
                console.print(f"[green]E-stopped device {node_id}[/green]")

    def arm_one(self, node_id):
        device = self.devices.get(node_id)
        if not device:
            console.print(f"[red]Device with node_id {node_id} not found[/red]")
            return
        if not device.set_axis_state(AxisState.CLOSED_LOOP_CONTROL):
            console.print(
                f"[red]Failed to enter closed loop control for device {node_id}[/red]"
            )
            return
        console.print(f"[green]Initialized device {node_id}[/green]")

    def set_position(self, node_id: int, position: float):
        device = self.devices.get(node_id)
        if not device:
            console.print(f"[red]Device with node_id {node_id} not found[/red]")
            return
        device.set_position(position)

    def calibrate_all(self) -> None:
        """
        Calibrate all devices sequentially, grouped by joint type
        """
        # Get all the node_ids
        node_ids = list(self.devices.keys())
        # Group by joint type (Hip Abduction/Adduction, Hip Flexion/Extension, Knee Flexion/Extension)
        haa_ids = [x for x in node_ids if x % 3 == 0]
        hfe_ids = [x for x in node_ids if x % 3 == 1]
        kfe_ids = [x for x in node_ids if x % 3 == 2]

        # Calibrate each group sequentially
        joint_groups = [
            ("kfe", kfe_ids),
            ("hfe", hfe_ids),
            ("haa", haa_ids),
        ]

        for group_name, ids in joint_groups:
            console.print(
                f"[blue]Starting calibration of {group_name} joints...[/blue]"
            )

            for node_id in ids:
                device = self.devices[node_id]
                console.print(
                    f"[blue]Calibrating {device.name} (Node ID: {node_id})...[/blue]"
                )

                # Clear errors before calibration
                device.clear_errors()

                # Perform motor calibration first
                console.print(
                    f"[yellow]Starting motor calibration for {device.name}...[/yellow]"
                )

                err, state, procedure_result, _ = self.calibrate_one(node_id)

                if (
                    err != ODriveErrorCode.NO_ERROR
                    or procedure_result != ODriveProcedureResult.SUCCESS
                ):
                    console.print(
                        f"[red]Motor calibration failed for {device.name}, stopping...[/red]"
                    )

                    self.stop()
                    return

                if state == AxisState.IDLE:
                    if group_name == "kfe":
                        self.arm_one(node_id)
                        self.set_position(node_id, -3.14)
                    elif group_name == "hfe":
                        self.arm_one(node_id)
                        self.set_position(node_id, -1.57)
                    elif group_name == "haa":
                        self.arm_one(node_id)
                        self.set_position(node_id, 0)

                    console.print(
                        f"[green]Motor calibration completed for {device.name}[/green]"
                    )

                # Wait a moment between calibration steps
                time.sleep(0.5)

            console.print(f"[blue]Completed calibration of {group_name} joints[/blue]")

            # Wait between joint groups
            time.sleep(2.0)

        console.print(f"[green]All device calibration completed[/green]")
        self.motor_calibrated = True

        
        
