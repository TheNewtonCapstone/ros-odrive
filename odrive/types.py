from enum import IntEnum, unique
from typing import Callable, Optional, Tuple


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
class ODriveErrorCode(IntEnum):
    # see https://docs.odriverobotics.com/v/latest/fibre_types/com_odriverobotics_ODrive.html#ODrive.Error for more details

    NO_ERROR = 0
    INITILIZATION_ERROR = 1
    SYSTEM_LEVEL = 2
    TIMING_ERROR = 4
    MISSING_ESTIMATE = 8
    BAD_CONFIG = 16
    DRV_FAULT = 32
    MISSING_INPUT = 64
    DC_BUS_OVER_VOLTAGE = 256
    DC_BUS_UNDER_VOLTAGE = 512
    DC_BUS_OVER_CURRENT = 1024  # either
    DC_BUS_OVER_REGEN_CURRENT = 2048
    CURRENT_LIMITATION = 4096
    MOTOR_OVER_TEMP = 8192
    INVERTER_OVER_TEMP = 16384
    VELOCITY_LIMIT_VIOLATION = 32768
    POSITION_LIMIT_VIOLATION = 65536
    WATCHDOG_TIMER_EXPIRED = 131072
    ESTOP_REQUESTED = 33554432
    SPINOUT_DETECTED = 67108864
    BRAKE_RESISTOR_DISARMED = 134217728
    THERMISTOR_DISCONNECTED = 268435456
    CALIBRATION_ERROR = 1073741824


@unique
class ODriveProcedureResult(IntEnum):
    # https://docs.odriverobotics.com/v/latest/fibre_types/com_odriverobotics_ODrive.html#ODrive.ProcedureResult
    SUCCESS = 0
    BUSY = 1
    CANCELLED = 2
    DISARMED = 3
    NO_RESPONSE = 4
    POLE_PAIR_CPR_MISMATCH = 5
    PHASE_RESISTANCE_OUT_OF_RANGE = 6
    PHASE_INDUCTANCE_OUT_OF_RANGE = 7
    UNBALANCED_PHASES = 8
    INVALID_MOTOR_TYPE = 9
    ILLEGAL_HALL_STATE = 10
    TIMEOUT = 11
    HOMING_WITHOUT_ENDSTOP = 12
    NOT_CALIBRATED = 14
    NOT_CONVERGING = 15  #
    """  
    The calibration did not converge.

    The measurements made during a calibration task did not reach sufficient statistical significance.

    For instance during AxisState.ENCODER_OFFSET_CALIBRATION the encoder did not move sufficiently to determine the direction between encoder and motor.

    Try changing the calibration parameters."""


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


class Heartbeat:
    def __init__(
        self,
        error,
        state,
        result,
        done,
    ):

        self.error = ODriveErrorCode(error)
        self.state = AxisState(state)
        self.result = ODriveProcedureResult(result)
        self.done = done

    @property
    def _error(self):
        return self.error

    @property
    def _state_enum(self):
        return self.state

    @property
    def _result(self):
        return self.result

    @property
    def _done(self):
        return self.done

    def is_armed(self):
        return self.state == AxisState.CLOSED_LOOP_CONTROL

    def is_idle(self):
        return self.state == AxisState.IDLE

    def has_error(self):
        return self.error != ODriveErrorCode.NO_ERROR

    def is_perfect(self):
        return (
            self.error == ODriveErrorCode.NO_ERROR
            and self.result == ODriveProcedureResult.SUCCESS
        )

    def __str__(self):
        return f"error:{self.error.name},state:{self.state.name},result:{self.result.name},done:{self.done}"
