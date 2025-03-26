import struct
import time
from typing import Dict, List, Optional, Callable
from rich.console import Console
from .device import ODriveDevice, AxisState, ControlMode, InputMode, OdriveCANCommands
from .can_interface import CanInterface, Arbitration
from .exception import (
    CalibrationFailedException,
    DeviceNotFoundException,
    NotArmedException,
)
from .types import Heartbeat, ODriveErrorCode, ODriveProcedureResult
import yaml
import os
import rclpy
import pprint

console = Console()


class ODriveManager:
    """
    Manager class for handling multiple ODrive devices on a CAN bus
    """

    def __init__(self, can_interface: CanInterface):
        """
        Args:
            can_interface: CAN interface for communication
        """
        self.can_interface: CanInterface = can_interface
        self.devices: Dict[int, ODriveDevice] = {}  # node_id -> device
        self.num_devices = 0
        self.running = False
        self._devices_calibrated = False

    def start(self, config_file_path: str) -> None:
        """
        Start the manager and all devices
        """
        self.can_interface.start(self.process_can_message)
        # check if the devices in the config file match the discovered devices
        discovered_nodes = self.enumerate_devices()
        self.load_configs_from_file(config_file_path, discovered_nodes)
        for node_id, device in self.devices.items():
            if node_id not in discovered_nodes:
                raise Exception(
                    f"Device with node_id {node_id} in config file not found"
                )
        self.clear_errors()
        # set control mode first to avoid arming and disarming 
        self.set_controller_mode_all(ControlMode.POSITION_CONTROL, InputMode.POS_FILTER)

        not_calibrated = self.get_calibrated_devices()

        # calibrate per group
        hfe_ids = [x for x in not_calibrated if x % 2 == 0]
        kfe_ids = [x for x in not_calibrated if x % 2 != 0]

        console.print(f"Calibrating KFE: {kfe_ids}")
        self.calibrate_group(kfe_ids)
        console.print(f"Calibrating HFE: {hfe_ids}")
        self.calibrate_group(hfe_ids)
        self.arm_all() 

        for node_id, device in self.devices.items():
            self.set_position(node_id, device.starting_position)
            # device.set_position(device.starting_position)
            console.print(f"Node ID: {node_id} at position {device.starting_position} current position: {device.get_position()}")
            time.sleep(0.5)
            
        self._devices_calibrated = True
        

    def load_configs_from_file(self, config_file_path: str, discovered_nodes: List[int]):
        config = {}
        try:
            with open(config_file_path, "r") as file:
                config = yaml.safe_load(file)

            motor_params = config.get("motors_params", {})

            for motor_name, motor_params in config["motors_params"].items():
                self.num_devices += 1

                if motor_params.get("enabled", True) and motor_params["node_id"] in discovered_nodes:
                    node_id = motor_params["node_id"]
                    name = motor_params["name"].lower()
                    direction = motor_params["direction"]
                    position_limit = motor_params["position_limit"]
                    gear_ratio = motor_params["gear_ratio"]
                    starting_position = motor_params.get("start_pos", 0.0)

                    self.add_device(
                        node_id=node_id,
                        name=name,
                        direction=direction,
                        position_limit=position_limit,
                        gear_ratio=gear_ratio,
                        starting_position=starting_position,
                    )
                elif motor_params.get("enabled", True) and motor_params["node_id"] not in discovered_nodes:
                    raise Exception(f"Device with node_id {motor_params['node_id']} not found")

            # print all the added devices
            for node_id, device in self.devices.items():
                console.print(
                    f"[green]Added device {device.name} with node_id {node_id}[/green]"
                )
            time.sleep(3)
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
        starting_position: float,
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
            starting_position,
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
        for i in range(12):
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

        for i in range(12):
            if i in self.devices:
                velocities.append(self.devices[i].get_velocity())
            else:
                velocities.append(0.0)

        return velocities

    def get_device(self, node_id: int) -> Optional[ODriveDevice]:
        return self.devices.get(node_id)

    def get_devices(self) -> Dict[int, ODriveDevice]:
        return self.devices

    def get_devices_calibrated(self) -> bool:
        return self._devices_calibrated

    def request(
        self,
        node_id: int,
        cmd_id: int,
        data: bytes,
        timeout: float = 2.0,
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
        return self.can_interface.request(node_id, cmd_id, data, timeout)

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
                    device.set_position(positions.get(node_id))
            else:
                console.print(
                    f"[yellow]Not enough positions provided for device {node_id}[/yellow]"
                )

    def arm_all(self) -> None:
        # set control mode
        for node_id, device in self.devices.items():
            if device.is_calibrated:
                device.arm()
                continue

            if not device.is_calibrated:
                console.print(f"[red]Device with node_id {node_id} not calibrated[/red]")
                device.calibrate()
                device.set_controller_mode(ControlMode.POSITION_CONTROL, InputMode.POS_FILTER)
                device.arm()

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

        hb = device.set_axis_state(AxisState.CLOSED_LOOP_CONTROL)

        if (
            hb.error == ODriveErrorCode.CALIBRATION_ERROR
            or hb.result == ODriveProcedureResult.NOT_CALIBRATED
        ):
            raise CalibrationFailedException(
                node_id=node_id,
                message=f"Calibration failed for device {device.name}",
                error_code=hb.error,
                procedure_result=hb.result,
            )

        if hb.state != AxisState.CLOSED_LOOP_CONTROL:
            raise NotArmedException(
                node_id=node_id,
                message=f"Failed to arm device {device.name}",
                error_code=hb.error,
                procedure_result=hb.result,
            )

        console.print(f"[green]Armed device with id{node_id}[/green]")

        return hb

    def set_position(self, node_id: int, position: float):
        device = self.devices.get(node_id)

        if not device:
            console.print(f"[red]Device with node_id {node_id} not found[/red]")
            return
        # check if device is armed
        if device.is_calibrated and device.is_armed and device.control_mode == ControlMode.POSITION_CONTROL:
            device.set_position(position)
            return 
        
        if not device.is_calibrated:
            console.print(f"[red]Device with node_id {node_id} not calibrated[/red]")
            device.calibrate()
            device.set_controller_mode(ControlMode.POSITION_CONTROL, InputMode.POS_FILTER)
            device.arm()
            device.set_position(position)
            return 


    def calibrate_all(self) -> None:
        """
        Calibrate all devices sequentially, grouped by joint type
        """
        # Get all the node_ids
        node_ids = list(self.devices.keys())
        # Group by joint type (Hip Abduction/Adduction, Hip Flexion/Extension, Knee Flexion/Extension)
        hfe_ids = [x for x in node_ids if x % 3 == 1]
        kfe_ids = [x for x in node_ids if x % 3 == 2]

        # Calibrate each group sequentially
        joint_groups = [
            ("kfe", kfe_ids),
            ("hfe", hfe_ids),
        ]

        for group_name, ids in joint_groups:
            console.print(
                f"[blue]Starting calibration of {group_name} joints...[/blue]"
            )

            not_calib = []
            # check if they are calibrated before

            for node_id in ids:
                device = self.devices[node_id]

                # Clear errors before calibration
                device.clear_errors()

                hb = device.arm()

                if hb.is_armed():
                    console.print(f"[green]Device {device.name} is calibrated [/green]")
                    device.disarm()
                else:
                    console.print(
                        f"[red]Device {device.name} is not calibrated adding it to queue[/red]"
                    )
                    not_calib.append(node_id)

            self.calibrate_group(not_calib)

            for node_id in ids:
                device = self.devices[node_id]

                console.print(
                    f"[blue]{device.name} (Node ID: {node_id}) is calibrated, proceding to arming...[/blue]"
                )

                # Clear errors before calibration
                device.clear_errors()

                hb = device.request_heartbeat()

                if not hb.is_perfect():
                    console.print(
                        f"[red]Motor calibration failed for {device.name}, stopping...[/red]"
                    )
                    self.stop()

                    raise CalibrationFailedException(
                        node_id=node_id,
                        message=f"Motor calibration failed for {device.name}",
                        error_code=hb.error,
                        procedure_result=hb.result,
                    )

                if hb.is_idle():
                    device.set_controller_mode(
                        ControlMode.POSITION_CONTROL,
                        InputMode.POS_FILTER,
                    )

                    time.sleep(0.1)

                    if group_name == "kfe":
                        self.arm_one(node_id)
                        self.set_position(node_id, -3.14)
                    elif group_name == "hfe":
                        self.arm_one(node_id)

                        if node_id == 7 or node_id == 10:
                            self.set_position(node_id, -1.57)
                        else:
                            self.set_position(node_id, 1.57)
                    elif group_name == "haa":
                        self.arm_one(node_id)
                        self.set_position(node_id, 0.0)

                    console.print(
                        f"Node ID: {node_id} at position {device.get_position()}"
                    )
                    console.print(
                        f"[green]Motor calibration completed for {device.name}[/green]"
                    )

                    # Wait a moment between calibration steps
                    time.sleep(0.5)

            console.print(f"[blue]Completed calibration of {group_name} joints[/blue]")

        console.print(f"[green]All device calibration completed[/green]")
        self._devices_calibrated = True

    def get_torques(self) -> List[float]:
        torques = []
        for i in range(11):
            if i in self.devices:
                torques.append(self.devices[i].get_torque())
            else:
                torques.append(0.0)

        return torques

    # funtion to calibrate some of the motors
    def calibrate_group(self, node_ids: List[int], timeout: int = 60) -> None:
        """Calibrate a list of motors at the same time"""
        # clear errors
        for node_id in node_ids:
            self.devices[node_id].clear_errors()

        cmd_id = OdriveCANCommands.SET_AXIS_STATE
        for node_id in node_ids:
            arb_id = node_id << Arbitration.NODE_ID_SIZE | cmd_id
            data = struct.pack("<I", AxisState.FULL_CALIBRATION_SEQUENCE)
            self.send_can_frame(arb_id, data)

        # poll all devices until they are a done or timeout
        start_time = time.time()
        pendings_nodes = set(node_ids)
        errored_nodes: Dict[int, Heartbeat] = {}

        while len(pendings_nodes) > 0 and (time.time() - start_time) < timeout:
            nodes_to_checks = pendings_nodes.copy()  # avoid mutation while iterating

            for node_id in nodes_to_checks:
                hb = self.devices[node_id].request_heartbeat()

                if hb.result == ODriveProcedureResult.BUSY:
                    console.print(f"[blue]Calibrating motor {node_id}...[/blue]")
                elif hb.result == ODriveProcedureResult.SUCCESS:
                    console.print(
                        f"[green]Calibration of motor {node_id} completed[/green]"
                    )
                    pendings_nodes.remove(node_id)
                else:
                    console.print(
                        f"[red]Calibration of motor {node_id} is {hb.result}[/red]"
                    )
                    pendings_nodes.remove(node_id)
                    errored_nodes[node_id] = hb

            time.sleep(0.5)

        # print the timeouts
        for node_id, hb in errored_nodes.items():
            console.print(f"[red]Calibration of motor {node_id} timed out, {hb}[/red]")

        if len(errored_nodes) > 0:
            self.stop()
        # update the calibration status
        for node_id in node_ids:
            if node_id not in errored_nodes:
                self.devices[node_id].is_calibrated = True
                
        return
    
    def clear_errors(self) -> None:
        """
        Clear errors on all devices
        """
        for node_id, device in self.devices.items():
            hb = device.request_heartbeat()
            if hb.error != ODriveErrorCode.NO_ERROR :
                device.clear_errors()
                console.print(f"[blue]Cleared errors for device {node_id}[/blue]")        
                
    def get_calibrated_devices(self)-> List[int]:
        # get the heartbeats of all the devices
        not_calibrated = []
        for node_id, device in self.devices.items():
            hb = device.arm()
            if hb.not_calibrated():
                not_calibrated.append(node_id)
                console.print(f"Device {node_id} heartbeat not calibrate: {hb}")
            else:
                device.is_calibrated = True
                
                console.print(f"Device {node_id} heartbeat calibrated: {hb}")
        return not_calibrated
           
         
        
    def set_controller_mode_all(self, control_mode: ControlMode, input_mode: InputMode):
        for node_id, device in self.devices.items():
            console.print(f"Setting controller mode {control_mode.name} for device {node_id}")
            device.disarm()
            device.set_controller_mode(control_mode, input_mode) 
            

    def set_controller_mode_group(self, node_ids: List[int], control_mode: ControlMode, input_mode: InputMode):
        for node_id in node_ids:
            device = self.devices[node_id]
            console.print(f"Setting controller mode {control_mode.name} for device {node_id}")
            device.disarm()
            device.set_controller_mode(control_mode, input_mode) 