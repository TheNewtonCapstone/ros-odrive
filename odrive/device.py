import math
import struct
from enum import IntEnum, unique
from typing import Callable, Optional, Tuple
import time
from .can_interface import Arbitration
from rich.console import Console
from .error import ODriveErrorCode, ODriveProcedureResult
from .exception import ODriveException, NotArmedException, CalibrationFailedException


@unique
class AxisState(IntEnum):
    """
    Enum class for the current statues of an ODrive axis, comes from the ODrive docs.
    See https://docs.odriverobotics.com/v/latest/manual/can-protocol.html for more information.
    """

    UNDEFINED = 0
    IDLE = 1
    STARTUP_SEQUENCE = 2
    FULL_CALIBRATION_SEQUENCE = 3
    MOTOR_CALIBRATION = 4
    ENCODER_INDEX_SEARCH = 6
    ENCODER_OFFSET_CALIBRATION = 7
    CLOSED_LOOP_CONTROL = 8
    LOCKIN_SPIN = 9
    ENCODER_DIR_FIND = 10
    HOMING = 11
    ENCODER_HALL_PHASE_receive_thread = 12
    ENCODER_HALL_POLARITY_CALIBRATION = 13


@unique
class OdriveCANCommands(IntEnum):
    """
    Enum class that contains the ODrive CAN commands.
    See https://docs.odriverobotics.com/v/latest/manual/can-protocol.html
    for more information.
    """

    GET_VERSION = 0x00
    GET_HEARTBEAT = 0x01
    ESTOP = 0x02
    GET_ERROR = 0x03
    RXS_DO = 0x04
    TXS_DO = 0x05
    GET_ADDRESS = 0x06
    SET_AXIS_STATE = 0x07
    GET_ENCODER_ESTIMATES = 0x09

    SET_CONTROLLER_MODE = 0x0B
    SET_INPUT_POS = 0x0C
    SET_INPUT_VEL = 0x0D
    SET_INPUT_TORQUE = 0x0E
    SET_LIMITS = 0x0F
    SET_ABSOLUTE_POSITION = 0x19

    SET_TRAJ_VEL_LIMIT = 0x11
    SET_TRAJ_ACCEL_LIMIT = 0x12
    SET_TRAJ_INERTIA = 0x13

    GET_IQ = 0x14
    GET_TEMPERATURE = 0x15

    REBOOT = 0x16
    GET_BUS_VOLTAGE = 0x17
    CLEAR_ERRORS = 0x18
    SET_POS_GAINS = 0x1A
    SET_VEL_GAINS = 0x1B
    GET_TORQUE = 0x1C
    GET_POWERS = 0x1D
    ENTER_DFU_MODE = 0x1F


@unique
class ControlMode(IntEnum):
    """
    Enum class that contains the control modes of the ODrive.
    """

    VOLTAGE_CONTROL = 0
    TORQUE_CONTROL = 1
    VELOCITY_CONTROL = 2
    POSITION_CONTROL = 3


@unique
class InputMode(IntEnum):
    """
    Enum class that contains the input modes of the ODrive.
    """

    INACTIVE = 0
    PASSTHROUGH = 1
    VEL_RAMP = 2
    POS_FILTER = 3
    MIX_CHANNELS = 4
    TRAP_TRAJ = 5


class ODriveDevice:
    """
    This class represents a single ODrive device on the CAN bus.
    """

    def __init__(
        self,
        node_id: int,
        name: str,
        direction: float,
        position_limit: float,
        gear_ratio: float,
        send_can_frame: Callable[[int, bytes], bool],
        request: Callable[[int, int, bytes, float], Optional[bytes]] = None,
    ):
        """
        Initialize ODrive device.

        Args:
            node_id: CAN node ID of the ODrive device
            send_can_frame: Function to send CAN frames (takes arbitration_id and data)
        """
        self.name = name
        self.node_id = node_id
        self.min_position = -position_limit
        self.max_position = position_limit
        self.direction = direction  # direction of the motor, this used to keep the all movements uniform
        self.gear_ratio = gear_ratio  # gear ratio of the motor
        self.send_can_frame = send_can_frame
        self.request = request

        # rad to motor turns conversion
        self.TURNS_TO_RAD = (2 * math.pi) / self.gear_ratio
        self.RAD_TO_TURNS = self.gear_ratio / (2 * math.pi)

        self.offset = 0.0  # offset in turns

        # State tracking
        self.position = 0.0
        self.velocity = 0.0
        self.torque_target = 0.0
        self.torque_estimate = 0.0
        self.axis_error = 0
        self.axis_state = AxisState.UNDEFINED
        self.procedure_result = 0
        self.trajectory_done = False

        # Message handlers
        self.message_handlers = {
            OdriveCANCommands.GET_ENCODER_ESTIMATES: self.handle_encoder_estimates,
            OdriveCANCommands.GET_HEARTBEAT: self.handle_heartbeat,
            OdriveCANCommands.GET_TORQUE: self.handle_torque,
        }

        # Debug info
        self.last_send_time = 0
        self.last_receive_time = 0
        self.console = Console()

    #######################################################################
    # Actuator methods (high-level)
    #######################################################################

    def set_position(
        self,
        pos: float,
        velocity_ff: float = 0.0,
        torque_ff: float = 0.0,
    ):
        """
        Set the position of the axis.

        Args:
            pos: Target position in radians
            velocity_ff: Velocity feed-forward in turns/s
            torque_ff: Torque feed-forward in Nm

        Returns:
            bool: True if message was sent successfully
        """

        err, state, procedure_result, traj_done = self.request_heartbeat()
        if state != AxisState.CLOSED_LOOP_CONTROL:
            raise NotArmedException(
                self.node_id,
                message="Axis is not in closed loop control mode",
                error_code=ODriveErrorCode(err),
                procedure_result=ODriveProcedureResult(procedure_result),
            )

        turns = self.rad_to_turns(pos)
        self.console.print(f"Setting position to {pos} turns: {turns}")

        cmd_id = OdriveCANCommands.SET_INPUT_POS
        arb_id = self.make_arbitration_id(self.node_id, cmd_id)

        # Convert to appropriate format
        # vel_ff_int = int(self.ravelocity_ff * 1000)  # Scale by 1000 per ODrive protocol
        # torque_ff_int = int(torque_ff * 1000)  # Scale by 1000 per ODrive protocol

        # # Clamp to int16 range
        # vel_ff_int = max(min(self.rad_to_turns(vel_ff_int), 32767), -32768)
        # torque_ff_int = max(min(torque_ff_int, 32767), -32768)

        # f is for float, h is for int16
        data = struct.pack("<fhh", turns, 0, 0)
        success = self.send_can_frame(arb_id, data)

        if success:
            self.last_send_time = time.time()

    def set_velocity(self, vel: float, torque_ff: float = 0.0):
        """
        Set the velocity of the axis.

        Args:
            vel: Target velocity in turns/s
            torque_ff: Torque feed-forward in Nm

        Returns:
            bool: True if message was sent successfully
        """
        err, state, procedure_result, traj_done = self.request_heartbeat()
        if state != AxisState.CLOSED_LOOP_CONTROL:
            raise NotArmedException(
                self.node_id,
                message="Axis is not in closed loop control mode",
                error_code=ODriveErrorCode(err),
                procedure_result=ODriveProcedureResult(procedure_result),
            )

        cmd_id = OdriveCANCommands.SET_INPUT_VEL
        arb_id = self.make_arbitration_id(self.node_id, cmd_id)
        data = struct.pack("<ff", self.rad_to_turns(vel), torque_ff)
        success = self.send_can_frame(arb_id, data)

        if success:
            self.last_send_time = time.time()

    def set_torque(self, torque: float):
        """
        Set the torque of the axis.

        Args:
            torque: Target torque in Nm

        Returns:
            bool: True if message was sent successfully
        """
        err, state, procedure_result, traj_done = self.request_heartbeat()
        if state != AxisState.CLOSED_LOOP_CONTROL:
            raise NotArmedException(
                self.node_id,
                message="Axis is not in closed loop control mode",
                error_code=ODriveErrorCode(err),
                procedure_result=ODriveProcedureResult(procedure_result),
            )

        cmd_id = OdriveCANCommands.SET_INPUT_TORQUE
        arb_id = self.make_arbitration_id(self.node_id, cmd_id)
        data = struct.pack("<f", torque)
        success = self.send_can_frame(arb_id, data)

        if success:
            self.last_send_time = time.time()

    def request_encoder_estimates(self) -> bool:
        """
        Request the encoder estimates.

        Returns:
            bool: True if request was sent successfully
        """
        cmd_id = OdriveCANCommands.GET_ENCODER_ESTIMATES
        arb_id = self.make_arbitration_id(self.node_id, cmd_id)
        # Send an RTR frame (Remote Transmission Request) see https://github.com/odriverobotics/ODrive/blob/master/docs/can-protocol.rst
        # Using empty data with RTR bit would be better, but our interface doesn't expose RTR
        # so we'll send a regular frame with empty data
        success = self.send_can_frame(arb_id, b"")

        if success:
            self.last_send_time = time.time()

        return success

    #######################################################################
    # CAN methods
    #######################################################################

    def make_arbitration_id(self, node_id: int, cmd_id: int) -> int:
        """
        Create the arbitration ID for a CAN message.

        Args:
            node_id: Node ID of the target ODrive
            cmd_id: Command ID of the message

        Returns:
            int: Arbitration ID
        """
        return node_id << Arbitration.NODE_ID_SIZE | cmd_id

    def set_axis_state(self, state: AxisState, timeout: float = 3.0) -> AxisState:
        """
        Set the state of the axis.

        Args:
            state: Target axis state

        Returns:
            bool: True if message was sent successfully
        """

        cmd_id = OdriveCANCommands.SET_AXIS_STATE

        # get initial state
        err, curr_state, procedure_result, traj_done = self.request_heartbeat(
            timeout=0.1
        )

        if procedure_result == ODriveProcedureResult.CANCELLED:
            time.sleep(0.1)
            self.clear_errors()

        # request a to check if the axis is a state we can use
        data = struct.pack("<I", int(state))
        self.send_can_frame(self.make_arbitration_id(self.node_id, cmd_id), data)

        time.sleep(0.5)
        return self.poll_heartbeat(timeout=timeout)

    def poll_heartbeat(self, timeout: float = 3.0) -> AxisState:
        """Periodically poll the heartbeat to check if the axis is in the desired state and get to the desired state
        Args:
            end_state: The desired state of the axis
            timeout: Maximum time to wait for the axis to get to the desired
        Returns:
            bool: True if the axis is in the desired state and transition state and false if the request timeout
        """
        start_time = time.time()
        while time.time() - start_time < timeout:
            err, curr_state, procedure_result, traj_done = self.request_heartbeat()
            if err != 0:
                raise ODriveException(
                    self.node_id,
                    ODriveErrorCode(err),
                    ODriveProcedureResult(procedure_result),
                    f"Error setting axis state for node {self.name}",
                )

            if procedure_result == ODriveProcedureResult.BUSY:
                time.sleep(0.1)
                continue

            if procedure_result == ODriveProcedureResult.SUCCESS:
                self._print_heartbeat(err, curr_state, procedure_result, traj_done)
                break

        return self.request_heartbeat()

    def set_controller_mode(
        self,
        control_mode: ControlMode,
        input_mode: InputMode,
    ) -> bool:
        """
        Set the controller mode of the axis.

        Args:
            control_mode: Control mode to set
            input_mode: Input mode to set

        Returns:
            bool: True if message was sent successfully
        """
        cmd_id = OdriveCANCommands.SET_CONTROLLER_MODE
        arb_id = self.make_arbitration_id(self.node_id, cmd_id)
        # TODO: ADD SAFETY CHECKS CONTROL MODE SHOULD BE IDLE
        
        data = struct.pack("<II", int(control_mode), int(input_mode))
        success = self.send_can_frame(arb_id, data)

        return self.poll_heartbeat()

    def _print_heartbeat(
        self,
        error: int,
        state: int,
        procedure_result: int,
        trajectory_done: int,
    ):
        self.console.print(
            f"Node: {self.name} Error: {ODriveErrorCode(error).name}, State: {AxisState(state).name}, Procedure Result: {ODriveProcedureResult(procedure_result).name}, Trajectory Done: {trajectory_done}"
        )

    def clear_errors(self, identify: bool = False) -> bool:
        """
        Clear the errors on the ODrive.

        Args:
            identify: If True, blink the LED to identify the ODrive

        Returns:
            bool: True if message was sent successfully
        """
        cmd_id = OdriveCANCommands.CLEAR_ERRORS
        arb_id = self.make_arbitration_id(self.node_id, cmd_id)
        data = struct.pack("<B", 1 if identify else 0)
        success = self.send_can_frame(arb_id, data)

        if success:
            self.last_send_time = time.time()

        # make sure the error is cleared
        err, procedure_result, res_state, _ = self.request_heartbeat()
        if err != 0:
            raise ODriveException(
                self.node_id,
                f"Error clearing errors for node {self.name}",
                ODriveErrorCode(err),
                ODriveProcedureResult(procedure_result),
            )

        if res_state != AxisState.IDLE:
            self.set_axis_state(AxisState.IDLE)

        return success

    def handle_encoder_estimates(self, data: bytes) -> None:
        """
        Process encoder estimates message

        Args:
            data: Data received from the ODrive
        """
        if len(data) >= 8:
            pos, vel = struct.unpack("<ff", data[:8])
            self.position = pos
            self.velocity = vel
            self.last_receive_time = time.time()

    def handle_heartbeat(self, data: bytes) -> None:
        """
        Process heartbeat message

        Args:
            data: Data received from the ODrive
        """
        if len(data) >= 7:
            error, state, procedure_result, trajectory_done = struct.unpack(
                "<IBBB", data[:7]
            )
            self.axis_error = error
            self.axis_state = (
                AxisState(state)
                if state in [e.value for e in AxisState]
                else AxisState.UNDEFINED
            )
            self.procedure_result = procedure_result
            self.trajectory_done = trajectory_done == 1
            self.last_receive_time = time.time()

    def handle_torque(self, data: bytes) -> None:
        """
        Process torque message

        Args:
            data: Data received from the ODrive
        """
        if len(data) >= 8:
            torque_target, torque_estimate = struct.unpack("<ff", data[:8])
            self.torque_target = torque_target
            self.torque_estimate = torque_estimate
            self.last_receive_time = time.time()

    def process_can_message(self, cmd_id: int, data: bytes) -> None:
        """
            Process CAN message from the ODrive

        Args:
            cmd_id: Command ID of the message
            data: Data received from the ODrive
        """
        handler = self.message_handlers.get(cmd_id)

        if handler:
            handler(data)

    def set_absolute_position(self, position: float):
        """
        Sets the encoder position to a specified value

        Args:
            position: Desired position in turns

        Returns:
            bool: True if message was sent successfully
        """
        cmd_id = OdriveCANCommands.SET_ABSOLUTE_POSITION
        arb_id = self.make_arbitration_id(self.node_id, cmd_id)
        data = struct.pack("<f", position)
        success = self.send_can_frame(arb_id, data)

        if success:
            self.last_send_time = time.time()

        # TODO: ADD CHECKING TO SEE IF ENCODER

    def estop(self) -> bool:
        """
        Emergency stop (disarm the ODrive)

        Returns:
            bool: True if message was sent successfully
        """
        cmd_id = OdriveCANCommands.ESTOP
        arb_id = self.make_arbitration_id(self.node_id, cmd_id)
        success = self.send_can_frame(arb_id, b"")

        if success:
            self.last_send_time = time.time()

        # TODO: ADD CHECKING TO see if the state is IDLE
        return success

    def set_pos_gain(self, pos_gain: float) -> bool:
        """
        Set the position gain (P gain)

        Args:
            pos_gain: Position gain in (turns/s)/turns

        Returns:
            bool: True if message was sent successfully
        """
        cmd_id = OdriveCANCommands.SET_POS_GAINS
        arb_id = self.make_arbitration_id(self.node_id, cmd_id)
        data = struct.pack("<f", pos_gain)
        success = self.send_can_frame(arb_id, data)

        if success:
            self.last_send_time = time.time()

        return success

    def set_vel_gains(self, vel_gain: float, vel_integrator_gain: float) -> bool:
        """
        Set the velocity gains (P and I gains)

        Args:
            vel_gain: Velocity gain in Nm/(turns/s)
            vel_integrator_gain: Velocity integrator gain in Nm/turns

        Returns:
            bool: True if message was sent successfully
        """
        cmd_id = OdriveCANCommands.SET_VEL_GAINS
        arb_id = self.make_arbitration_id(self.node_id, cmd_id)
        data = struct.pack("<ff", vel_gain, vel_integrator_gain)
        success = self.send_can_frame(arb_id, data)

        if success:
            self.last_send_time = time.time()

        return success

    def set_traj_vel_limit(self, vel_limit: float) -> bool:
        """
        Set the trajectory velocity limit

        Args:
            vel_limit: Velocity limit in turns/s

        Returns:
            bool: True if message was sent successfully
        """
        cmd_id = OdriveCANCommands.SET_TRAJ_VEL_LIMIT
        arb_id = self.make_arbitration_id(self.node_id, cmd_id)
        data = struct.pack("<f", vel_limit)
        success = self.send_can_frame(arb_id, data)

        if success:
            self.last_send_time = time.time()

        return success

    def set_traj_accel_limits(self, accel_limit: float, decel_limit: float) -> bool:
        """
        Set the trajectory acceleration and deceleration limits

        Args:
            accel_limit: Acceleration limit in turns/s²
            decel_limit: Deceleration limit in turns/s²

        Returns:
            bool: True if message was sent successfully
        """
        cmd_id = OdriveCANCommands.SET_TRAJ_ACCEL_LIMIT
        arb_id = self.make_arbitration_id(self.node_id, cmd_id)
        data = struct.pack("<ff", accel_limit, decel_limit)
        success = self.send_can_frame(arb_id, data)

        if success:
            self.last_send_time = time.time()

        return success

    def set_traj_inertia(self, inertia: float) -> bool:
        """
        Set the trajectory inertia

        Args:
            inertia: Inertia in Nm/(turns/s²)

        Returns:
            bool: True if message was sent successfully
        """
        cmd_id = OdriveCANCommands.SET_TRAJ_INERTIA
        arb_id = self.make_arbitration_id(self.node_id, cmd_id)
        data = struct.pack("<f", inertia)
        success = self.send_can_frame(arb_id, data)

        if success:
            self.last_send_time = time.time()

        return success

    def request_heartbeat(self, timeout: float = 0.1) -> Tuple[int, int, int, int]:
        """
        Request a heartbeat and wait for the response

        Args:
            timeout: Maximum time to wait for response:%y


        Returns:
            bool: True if heartbeat received and processed, False otherwise
        """

        data = self.request(
            self.node_id,
            OdriveCANCommands.GET_HEARTBEAT,
            b"",
            timeout=timeout,
        )

        error, state, procedure_result, trajectory_done = struct.unpack(
            "<IBBB",
            data[:7],
        )

        return error, state, procedure_result, trajectory_done

    def calibrate(self, timeout: float = 20.0):
        """
        Run motor calibration and wait for completion

        Args:
            timeout: Maximum time to wait for calibration

        Returns:
            bool: True if calibration successful, False otherwise
        """
        # Start calibration
        self.set_axis_state(AxisState.FULL_CALIBRATION_SEQUENCE, timeout=timeout)

        return self.request_heartbeat(timeout=timeout)

    def get_name(self):
        return self.name

    def get_position(self):
        return self.turns_to_rad(self.position)

    def get_velocity(self):
        return self.turns_to_rad(self.velocity)

    def set_limits(self, velocity_limit: float, current_limit: float) -> bool:
        """
        Set velocity and current limits

        Args:
            velocity_limit: Maximum velocity in turns/s
            current_limit: Maximum current in Amps

        Returns:
            bool: True if message was sent successfully
        """
        cmd_id = OdriveCANCommands.SET_LIMITS
        arb_id = self.make_arbitration_id(self.node_id, cmd_id)
        data = struct.pack("<ff", velocity_limit, current_limit)
        success = self.send_can_frame(arb_id, data)

        if success:
            self.last_send_time = time.time()

        return success

    def reboot(self, save_config: bool = False) -> bool:
        """
        Reboot the ODrive

        Args:
            save_config: If True, save configuration before rebooting

        Returns:
            bool: True if message was sent successfully
        """
        cmd_id = OdriveCANCommands.REBOOT
        arb_id = self.make_arbitration_id(self.node_id, cmd_id)
        action = 1 if save_config else 0  # 0: reboot, 1: save_configuration()
        data = struct.pack("<B", action)
        success = self.send_can_frame(arb_id, data)

        if success:
            self.last_send_time = time.time()

        return success

    def normalize_angle(self, angle: float) -> float:
        """Normalize the position to the range [-pi, pi]"""
        angle = angle % (2 * math.pi)
        return angle if angle <= math.pi else (angle - 2 * math.pi)

    def shortest_path(self, target_position: float) -> float:
        """Find the shortest path to the target position. we do this to avoid taking the long path when the motor is at the other end of the range"""
        target_position = self.normalize_angle(target_position)
        current_position = self.normalize_angle(self.position)
        diff = target_position - current_position

        return diff if abs(diff) <= math.pi else -diff

    def rad_to_turns(self, rad: float) -> float:
        """Convert radians to turns"""
        normalized_rad = self.normalize_angle(rad)
        direction_rad = normalized_rad * self.direction
        offset_rad = self.offset * self.direction
        self.console.print(f"Normalized rad: {normalized_rad}, direction rad: {direction_rad}, offset rad: {offset_rad}")
        rad_with_direction = self.normalize_angle(rad) * self.direction
        return (rad_with_direction * self.RAD_TO_TURNS) + self.offset

    def turns_to_rad(self, turns: float) -> float:
        """Convert turns to radians"""
        return self.normalize_angle((turns - self.offset) * self.TURNS_TO_RAD * self.direction)

    def is_within_limits(self, pos: float) -> bool:
        """Check if the position (rad) is within the limits"""
        return self.min_position <= pos <= self.max_position

    def clamp_to_limits(self, pos: float) -> float:
        """Clamp the position to the limits"""
        return max(self.min_position, min(self.max_position, pos))
