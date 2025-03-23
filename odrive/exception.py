import logging
from enum import IntEnum
from .types import ODriveErrorCode, ODriveProcedureResult
from typing import Optional
import logging

# from .logger import configure_logger

# logger = configure_logger()
logger = logging.getLogger("odrive")


class TimeoutException(Exception):
    pass


class CanException(Exception):
    def __init__(self, *args):
        super().__init__(*args)

    pass


class MessageNotSentException(CanException):
    pass


class NotConnectedException(CanException):
    pass


class ODriveException(Exception):
    def __init__(
        self,
        node_id: int,
        message: str,
        error_code: Optional[ODriveErrorCode] = None,
        procedure_result: Optional[ODriveProcedureResult] = None,
    ):

        self.node_id = (node_id,)
        self.error_code = error_code
        self.procedure_result = procedure_result
        self.message = message
        error_msg = f"Device {node_id}: {message}"
        if error_code is not None:
            error_msg += f" Error code: {error_code}: {error_code.name}"
        if procedure_result is not None:
            error_msg += (
                f" Procedure result: {procedure_result}: {procedure_result}"
            )
        super().__init__(error_msg)

    def _log_exceptions(self):
        if logger is None:
            logging.getLogger("odrive")
        logging.error("An exception: {}".format(self))


class NotArmedException(ODriveException):
    """Raised when attempting to access a device that is not armed."""

    pass


class DeviceNotFoundException(ODriveException):
    """Raised when attempting to access a device that doesn't exist."""

    pass


class CommandFailedException(ODriveException):
    """Raised when a command to the device fails."""

    pass


class StateTransitionException(ODriveException):
    """Raised when a state transition fails."""

    pass


class CalibrationFailedException(ODriveException):
    """Raised when calibration fails."""

    pass


class NoHeartbeatException(ODriveException):
    """Raised when no heartbeat is received from a device."""

    pass


class PositionLimitException(ODriveException):
    """Raised when attempting to set a position outside of limits."""

    pass


class NoHearBeatException(ODriveException):
    pass
