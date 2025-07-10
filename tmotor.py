import os
import time
import traceback
import warnings
import logging
from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum
from typing import Any, Optional, cast, Callable, NamedTuple

import can
import numpy as np

from SoftRealtimeLoop import SoftRealtimeLoop

class MotorConstants(NamedTuple):
    MOTOR_COUNT_PER_REV: int
    NM_PER_AMP: float
    NM_PER_RAD_TO_K: float
    NM_S_PER_RAD_TO_B: float
    MAX_CASE_TEMPERATURE: float
    MAX_WINDING_TEMPERATURE: float

class ControlModeConfig(NamedTuple):
    entry_callback: Callable
    exit_callback: Callable
    has_gains: bool
    max_gains: Optional[Any]

class ControlModeConfigs(NamedTuple):
    POSITION: ControlModeConfig
    CURRENT: ControlModeConfig
    VELOCITY: ControlModeConfig
    VOLTAGE: ControlModeConfig
    IDLE: ControlModeConfig

class ControlModes(Enum):
    POSITION = "POSITION"
    CURRENT = "CURRENT"
    VELOCITY = "VELOCITY"
    VOLTAGE = "VOLTAGE"
    IDLE = "IDLE"

CONTROL_MODES = ControlModes

def check_actuator_connection(func):
    def wrapper(self, *args, **kwargs):
        if self.is_offline:
            return func(self, *args, **kwargs)
        if not hasattr(self, '_canman') or self._canman is None:
            raise RuntimeError("Actuator not connected")
        return func(self, *args, **kwargs)
    return wrapper

def check_actuator_open(func):
    def wrapper(self, *args, **kwargs):
        if not self._is_open:
            raise RuntimeError("Actuator not opened")
        return func(self, *args, **kwargs)
    return wrapper

def check_actuator_stream(func):
    def wrapper(self, *args, **kwargs):
        if not self._is_streaming:
            raise RuntimeError("Actuator not streaming")
        return func(self, *args, **kwargs)
    return wrapper

# Simplified thermal model
class SimpleThermalModel:
    def __init__(self, temp_limit_windings, soft_border_C_windings, temp_limit_case, soft_border_C_case):
        self.temp_limit_windings = temp_limit_windings
        self.temp_limit_case = temp_limit_case
        self.T_w = temp_limit_case  # Simplified: assume winding temp equals case temp

# TMotor servo mode parameters configuration
SERVO_PARAMS: dict[str, Any] = {
    "ERROR_CODES": {
        0: "No Error",
        1: "Over voltage fault",
        2: "Under voltage fault",
        3: "DRV fault",
        4: "Absolute over current fault",
        5: "Over temp FET fault",
        6: "Over temp motor fault",
        7: "Gate driver over voltage fault",
        8: "Gate driver under voltage fault",
        9: "MCU under voltage fault",
        10: "Booting from watchdog reset fault",
        11: "Encoder SPI fault",
        12: "Encoder sincos below min amplitude fault",
        13: "Encoder sincos above max amplitude fault",
        14: "Flash corruption fault",
        15: "High offset current sensor 1 fault",
        16: "High offset current sensor 2 fault",
        17: "High offset current sensor 3 fault",
        18: "Unbalanced currents fault",
    },
    "AK10-9": {
        "P_min": -32000,  # -3200 deg
        "P_max": 32000,  # 3200 deg
        "V_min": -100000,  # ERPM
        "V_max": 100000,  # ERPM
        "Curr_min": -60000,  # -60A (protocol units)
        "Curr_max": 60000,  # 60A (protocol units)
        "Kt_actual": 0.206,  # Nm/A
        "GEAR_RATIO": 9.0,
        "NUM_POLE_PAIRS": 21,
    },
    "AK80-9": {
        "P_min": -32000,  # -3200 deg
        "P_max": 32000,  # 3200 deg
        "V_min": -32000,  # ERPM
        "V_max": 32000,  # ERPM
        "Curr_min": -60000,  # -60A (protocol units)
        "Curr_max": 60000,  # 60A (protocol units)
        "Kt_actual": 0.115,  # Nm/A
        "GEAR_RATIO": 9.0,
        "NUM_POLE_PAIRS": 21,
    },
    "CAN_PACKET_ID": {
        "CAN_PACKET_SET_DUTY": 0,
        "CAN_PACKET_SET_CURRENT": 1,
        "CAN_PACKET_SET_CURRENT_BRAKE": 2,
        "CAN_PACKET_SET_RPM": 3,
        "CAN_PACKET_SET_POS": 4,
        "CAN_PACKET_SET_ORIGIN_HERE": 5,
        "CAN_PACKET_SET_POS_SPD": 6,
    },
}

# TMotor servo mode constants
TMOTOR_SERVO_CONSTANTS = MotorConstants(
    MOTOR_COUNT_PER_REV=3600,  # 360 degrees * 10
    NM_PER_AMP=0.115,  # AK80-9 approximate
    NM_PER_RAD_TO_K=0.0,  # servo mode not needed
    NM_S_PER_RAD_TO_B=0.0,  # servo mode not needed
    MAX_CASE_TEMPERATURE=80.0,
    MAX_WINDING_TEMPERATURE=110.0,
)


@dataclass
class ServoMotorState:
    """Motor state data structure"""

    position: float = 0.0  # degrees
    velocity: float = 0.0  # ERPM
    current: float = 0.0  # amps
    temperature: float = 0.0  # celsius
    error: int = 0
    acceleration: float = 0.0  # rad/s²


class ServoControlMode(Enum):
    """TMotor servo control modes"""

    POSITION = 4
    VELOCITY = 3
    CURRENT = 1
    IDLE = 7


class CANManagerServo:
    """TMotor servo mode CAN communication manager"""

    _instance: Optional["CANManagerServo"] = None
    debug: bool = False
    _initialized: bool = False

    def __new__(cls) -> "CANManagerServo":
        if not cls._instance:
            cls._instance = super().__new__(cls)
            cls._instance._initialized = False
        return cls._instance

    def __init__(self) -> None:
        if hasattr(self, "_initialized") and self._initialized:
            return

        logging.info("Initializing CAN Manager for TMotor Servo Mode")
        try:
            # Configure CAN interface
            os.system("sudo /sbin/ip link set can0 down")  # noqa: S605, S607
            # Configure CAN interface with bitrate
            os.system("sudo /sbin/ip link set can0 up type can bitrate 1000000")  # noqa: S605, S607
            os.system("sudo ifconfig can0 txqueuelen 1000")  # noqa: S605, S607

            self.bus = can.interface.Bus(channel="can0", bustype="socketcan")
            self.notifier = can.Notifier(bus=self.bus, listeners=[])

            logging.info(f"CAN bus connected: {self.bus}")
            self._initialized = True

        except Exception as e:
            logging.error(f"CAN bus initialization failed: {e}")
            raise RuntimeError("CAN bus initialization failed") from e

    def __del__(self) -> None:
        try:
            os.system("sudo /sbin/ip link set can0 down")  # noqa: S605, S607
        except Exception as e:
            logging.warning(f"Error shutting down CAN interface: {e}")

    def send_message(self, motor_id: int, data: list, data_len: int) -> None:
        """Send CAN message"""
        if data_len > 8:
            raise ValueError(f"Data too long in message for motor {motor_id}")

        if self.debug:
            logging.info(f'ID: {hex(motor_id)} Data: [{", ".join(hex(d) for d in data)}]')

        message = can.Message(arbitration_id=motor_id, data=data, is_extended_id=True)

        try:
            self.bus.send(message)
        except can.CanError as e:
            logging.error(f"Failed to send CAN message: {e}")

    def power_on(self, motor_id: int) -> None:
        """Send power on command"""
        self.send_message(motor_id, [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC], 8)

    def power_off(self, motor_id: int) -> None:
        """Send power off command"""
        self.send_message(motor_id, [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD], 8)

    def set_current(self, controller_id: int, current: float) -> None:
        """Send current control command"""
        current_protocol = int(current * 1000.0)  # Convert to protocol units
        buffer = self._pack_int32(current_protocol)
        message_id = (SERVO_PARAMS["CAN_PACKET_ID"]["CAN_PACKET_SET_CURRENT"] << 8) | controller_id
        self.send_message(message_id, buffer, len(buffer))

    def set_velocity(self, controller_id: int, velocity: float) -> None:
        """Send velocity control command"""
        buffer = self._pack_int32(int(velocity))
        message_id = (SERVO_PARAMS["CAN_PACKET_ID"]["CAN_PACKET_SET_RPM"] << 8) | controller_id
        self.send_message(message_id, buffer, len(buffer))

    def set_position(self, controller_id: int, position: float) -> None:
        """Send position control command"""
        # NOTE(IMPORTANT): TMotor spec uses 1,000,000 scale (0.000001° resolution) per documentation
        # Current implementation uses 10 scale (0.1° resolution) for simplicity
        # To enable high-precision position control, change to: int(position * 1000000.0)
        buffer = self._pack_int32(int(position * 10.0))  # 0.1 degree resolution
        message_id = (SERVO_PARAMS["CAN_PACKET_ID"]["CAN_PACKET_SET_POS"] << 8) | controller_id
        self.send_message(message_id, buffer, len(buffer))

    def set_origin(self, controller_id: int, mode: int = 1) -> None:
        """Set motor origin"""
        buffer = [mode]
        message_id = (SERVO_PARAMS["CAN_PACKET_ID"]["CAN_PACKET_SET_ORIGIN_HERE"] << 8) | controller_id
        self.send_message(message_id, buffer, len(buffer))

    def set_control_mode(self, controller_id: int, mode: int) -> None:
        """Send control mode switch command"""
        buffer = [mode]
        message_id = (7 << 8) | controller_id  # Assume packet_id=7 for mode switching
        self.send_message(message_id, buffer, len(buffer))

    @staticmethod
    def _pack_int32(number: int) -> list:
        """Pack 32-bit integer to byte list (big-endian, MSB first)"""
        if number < 0:
            number = (1 << 32) + number
        # TMotor expects big-endian (MSB first) per documentation
        return [(number >> 24) & 0xFF, (number >> 16) & 0xFF, (number >> 8) & 0xFF, number & 0xFF]

    def parse_servo_message(self, data: bytes) -> ServoMotorState:
        """Parse servo message to motor state"""
        if len(data) < 8:
            raise ValueError(f"Invalid message length: {len(data)}")

        # Fix endian issue: TMotor packs as big-endian (Data[n]<<8 | Data[n+1])
        pos_int = int.from_bytes(data[0:2], byteorder="big", signed=True)
        spd_int = int.from_bytes(data[2:4], byteorder="big", signed=True)
        cur_int = int.from_bytes(data[4:6], byteorder="big", signed=True)

        motor_pos = float(pos_int * 0.1)  # position (degrees)
        motor_spd = float(spd_int * 10.0)  # velocity (ERPM)
        motor_cur = float(cur_int / 1000.0)  # current (amps)
        motor_temp = float(data[6])  # temperature (celsius)
        motor_error = int(data[7])  # error code

        return ServoMotorState(motor_pos, motor_spd, motor_cur, motor_temp, motor_error, 0.0)

    def add_motor_listener(self, motor: "TMotorServoActuator") -> None:
        """Add motor listener"""
        listener = MotorListener(self, motor)
        self.notifier.add_listener(listener)

    def enable_debug(self, enable: bool = True) -> None:
        """Enable/disable debug mode"""
        self.debug = enable
        if enable:
            logging.basicConfig(level=logging.DEBUG)
            logging.info("CAN debug mode enabled")

    def get_last_message_info(self) -> str:
        """Get information about last sent message"""
        return f"Last message sent at {time.time()}"


class MotorListener(can.Listener):
    """CAN message listener"""

    def __init__(self, canman: CANManagerServo, motor: "TMotorServoActuator") -> None:
        self.canman = canman
        self.motor = motor

    def on_message_received(self, msg) -> None:
        """Handle received CAN message"""
        data = bytes(msg.data)
        motor_id = msg.arbitration_id & 0x00000FF
        if motor_id == self.motor.motor_id:
            self.motor._update_state_async(self.canman.parse_servo_message(data))


# Simplified unit conversion functions
import math

def degrees_to_radians(degrees: float) -> float:
    """Convert degrees to radians"""
    return degrees * math.pi / 180.0


def radians_to_degrees(radians: float) -> float:
    """Convert radians to degrees"""
    return radians * 180.0 / math.pi


def erpm_to_rad_per_sec(erpm: float, num_pole_pairs: int) -> float:
    """Convert ERPM to rad/s"""
    return erpm * 2 * math.pi / (60 * num_pole_pairs)


def rad_per_sec_to_erpm(rad_per_sec: float, num_pole_pairs: int) -> float:
    """Convert rad/s to ERPM"""
    return rad_per_sec * 60 * num_pole_pairs / (2 * math.pi)


# Control mode configuration
def _servo_position_mode_entry(actuator: "TMotorServoActuator") -> None:
    actuator._servo_mode = ServoControlMode.POSITION
    # Send actual mode switch command to motor
    if not actuator.is_offline and actuator._canman:
        actuator._canman.set_control_mode(actuator.motor_id, ServoControlMode.POSITION.value)
        time.sleep(0.1)  # Wait for mode switch to complete


def _servo_position_mode_exit(actuator: "TMotorServoActuator") -> None:
    pass  # servo mode handles automatically


def _servo_current_mode_entry(actuator: "TMotorServoActuator") -> None:
    actuator._servo_mode = ServoControlMode.CURRENT
    # Send actual mode switch command to motor
    if not actuator.is_offline and actuator._canman:
        actuator._canman.set_control_mode(actuator.motor_id, ServoControlMode.CURRENT.value)
        time.sleep(0.1)  # Wait for mode switch to complete


def _servo_current_mode_exit(actuator: "TMotorServoActuator") -> None:
    if not actuator.is_offline and actuator._canman:
        actuator._canman.set_current(actuator.motor_id, 0.0)


def _servo_velocity_mode_entry(actuator: "TMotorServoActuator") -> None:
    actuator._servo_mode = ServoControlMode.VELOCITY
    # Send actual mode switch command to motor
    if not actuator.is_offline and actuator._canman:
        actuator._canman.set_control_mode(actuator.motor_id, ServoControlMode.VELOCITY.value)
        time.sleep(0.1)  # Wait for mode switch to complete


def _servo_velocity_mode_exit(actuator: "TMotorServoActuator") -> None:
    if not actuator.is_offline and actuator._canman:
        actuator._canman.set_velocity(actuator.motor_id, 0.0)


def _servo_idle_mode_entry(actuator: "TMotorServoActuator") -> None:
    actuator._servo_mode = ServoControlMode.IDLE
    # Send actual mode switch command to motor
    if not actuator.is_offline and actuator._canman:
        actuator._canman.set_control_mode(actuator.motor_id, ServoControlMode.IDLE.value)
        time.sleep(0.1)  # Wait for mode switch to complete


def _servo_idle_mode_exit(actuator: "TMotorServoActuator") -> None:
    pass


TMOTOR_SERVO_CONTROL_MODE_CONFIGS = ControlModeConfigs(
    POSITION=ControlModeConfig(
        entry_callback=_servo_position_mode_entry,
        exit_callback=_servo_position_mode_exit,
        has_gains=False,  # servo mode handles internally
        max_gains=None,
    ),
    CURRENT=ControlModeConfig(
        entry_callback=_servo_current_mode_entry,
        exit_callback=_servo_current_mode_exit,
        has_gains=False,  # servo mode handles internally
        max_gains=None,
    ),
    VELOCITY=ControlModeConfig(
        entry_callback=_servo_velocity_mode_entry,
        exit_callback=_servo_velocity_mode_exit,
        has_gains=False,  # servo mode handles internally
        max_gains=None,
    ),
    VOLTAGE=ControlModeConfig(
        entry_callback=lambda actuator: setattr(actuator, "_servo_mode", ServoControlMode.IDLE),
        exit_callback=lambda actuator: None,
        has_gains=False,
        max_gains=None,
    ),
    IDLE=ControlModeConfig(
        entry_callback=_servo_idle_mode_entry,
        exit_callback=_servo_idle_mode_exit,
        has_gains=False,
        max_gains=None,
    ),
)


class TMotorServoActuator(ABC):
    """

    Example:
        >>> with TMotorServoActuator(motor_type="AK80-9", motor_id=1) as motor:
        ...     motor.set_control_mode(CONTROL_MODES.CURRENT)
        ...     motor.set_output_torque(2.5)
        ...     motor.update()
    """

    def __init__(
        self,
        tag: str = "TMotorServoActuator",
        motor_type: str = "AK80-9",
        motor_id: int = 1,
        gear_ratio: float = 1.0,
        frequency: int = 500,
        offline: bool = False,
        max_temperature: float = 80.0,
        **kwargs: Any,
    ) -> None:
        """
        Initialize TMotor servo actuator

        Args:
            tag: actuator identifier
            motor_type: motor model (AK80-9, AK10-9)
            motor_id: CAN ID
            gear_ratio: gear ratio
            frequency: control frequency Hz
            offline: offline mode
            max_temperature: maximum temperature
        """
        # Validate motor type
        if motor_type not in SERVO_PARAMS:
            raise ValueError(f"Unsupported motor type: {motor_type}")

        # Base class attributes
        self.tag = tag
        self.gear_ratio = gear_ratio
        self.motor_constants = TMOTOR_SERVO_CONSTANTS
        self.frequency = frequency
        self.is_offline = offline
        self._is_open = False
        self._is_streaming = False
        self._is_homed = False
        self._control_mode = None

        # Motor configuration
        self.motor_type = motor_type
        self.motor_id = motor_id
        self.max_temperature = max_temperature
        self._motor_params = SERVO_PARAMS[motor_type]

        # CAN communication
        self._canman: Optional[CANManagerServo] = None
        if not self.is_offline:
            self._canman = CANManagerServo()
            self._canman.add_motor_listener(self)

        # State management
        self._motor_state = ServoMotorState()
        self._motor_state_async = ServoMotorState()
        self._servo_mode = ServoControlMode.IDLE

        # Error handling
        self._error_code: Optional[int] = None
        self._error_message: Optional[str] = None

        # Time management
        self._start_time = time.time()
        self._last_update_time = self._start_time
        self._last_command_time: Optional[float] = None

        # Thermal management
        self._thermal_model = SimpleThermalModel(
            temp_limit_windings=self.motor_constants.MAX_WINDING_TEMPERATURE,
            soft_border_C_windings=10.0,
            temp_limit_case=self.motor_constants.MAX_CASE_TEMPERATURE,
            soft_border_C_case=10.0,
        )

        logging.info(f"Initialized TMotor servo: {self.motor_type} ID:{self.motor_id}")

    @property
    def _CONTROL_MODE_CONFIGS(self) -> ControlModeConfigs:
        return TMOTOR_SERVO_CONTROL_MODE_CONFIGS

    def device_info_string(self) -> str:
        return f"{self.motor_type} ID:{self.motor_id}"

    @check_actuator_connection
    def start(self) -> None:
        """Start motor"""
        logging.info(f"Starting {self.device_info_string()}")

        if not self.is_offline and self._canman:
            self._canman.power_on(self.motor_id)
            time.sleep(0.5)  # Wait for motor startup

            # Set initial control mode to IDLE
            self._canman.set_control_mode(self.motor_id, ServoControlMode.IDLE.value)
            time.sleep(0.1)

            # Verify motor status
            self.update()  # Read status once
            if self._motor_state.error != 0:
                raise RuntimeError(f"Motor startup failed with error: {self._motor_state.error}")

        self._is_open = True
        self._is_streaming = True

    @check_actuator_stream
    @check_actuator_open
    def stop(self) -> None:
        """Stop motor"""
        logging.info(f"Stopping {self.device_info_string()}")

        if not self.is_offline and self._canman:
            self._canman.power_off(self.motor_id)

        self._is_open = False
        self._is_streaming = False

    def update(self) -> None:
        """Update motor state"""
        # Temperature check
        if self.case_temperature > self.max_case_temperature:
            raise RuntimeError(f"Temperature {self.case_temperature}°C exceeds limit")

        # Update state
        self._motor_state.position = self._motor_state_async.position
        self._motor_state.velocity = self._motor_state_async.velocity
        self._motor_state.current = self._motor_state_async.current
        self._motor_state.temperature = self._motor_state_async.temperature
        self._motor_state.error = self._motor_state_async.error

        # Communication timeout check
        now = time.time()
        if (
            self._last_command_time is not None
            and (now - self._last_command_time) < 0.25
            and (now - self._last_update_time) > 0.1
        ):
            warnings.warn(f"No data from motor: {self.device_info_string()}", RuntimeWarning, stacklevel=2)

        self._last_update_time = now

    def _update_state_async(self, servo_state: ServoMotorState) -> None:
        """Asynchronously update state"""
        # More detailed error handling
        if servo_state.error != 0:
            error_codes = cast(dict[int, str], SERVO_PARAMS["ERROR_CODES"])
            error_msg = error_codes.get(servo_state.error, f"Unknown error code: {servo_state.error}")
            self._error_code = servo_state.error
            self._error_message = error_msg

            # Take different actions based on error type
            if servo_state.error in [1, 2]:  # Voltage errors
                logging.error(f"Voltage error {servo_state.error}: {error_msg}")
            elif servo_state.error in [4, 5, 6]:  # Overcurrent or overtemperature
                logging.critical(f"Critical error {servo_state.error}: {error_msg}")
                # Auto-stop motor
                if self._canman:
                    self._canman.set_current(self.motor_id, 0.0)
            else:
                logging.warning(f"Motor error {servo_state.error}: {error_msg}")
        else:
            self._error_code = None
            self._error_message = None

        # Calculate acceleration
        now = time.time()
        dt = now - self._last_update_time
        if dt > 0:
            motor_params = cast(dict[str, Any], self._motor_params)
            old_vel_rad_s = erpm_to_rad_per_sec(self._motor_state_async.velocity, motor_params["NUM_POLE_PAIRS"])
            new_vel_rad_s = erpm_to_rad_per_sec(servo_state.velocity, motor_params["NUM_POLE_PAIRS"])
            servo_state.acceleration = (new_vel_rad_s - old_vel_rad_s) / dt

        self._motor_state_async = servo_state

    @property
    def error_info(self) -> Optional[tuple[int, str]]:
        """Get error information"""
        if self._error_code is not None and self._error_message is not None:
            return (self._error_code, self._error_message)
        return None

    def home(
        self,
        homing_voltage: int = 0,
        homing_frequency: Optional[int] = None,
        homing_direction: int = 0,
        output_position_offset: float = 0.0,
        current_threshold: int = 0,
        velocity_threshold: float = 0.0,
    ) -> None:
        """Home motor"""
        if not self.is_offline and self._canman:
            self._canman.set_origin(self.motor_id, 1)
            time.sleep(0.1)
        self._is_homed = True
        logging.info(f"Homed {self.device_info_string()}")

    # ============ Control Interface ============

    def set_motor_voltage(self, value: float) -> None:
        """Set motor voltage (not directly supported in servo mode)"""
        logging.warning("Voltage control not supported in servo mode")

    def set_motor_current(self, value: float) -> None:
        """Set motor current"""
        if not self.is_offline and self._canman:
            self._canman.set_current(self.motor_id, value)
            self._last_command_time = time.time()

    def set_motor_position(self, value: float) -> None:
        """Set motor position (radians)"""
        position_deg = radians_to_degrees(value)
        if not self.is_offline and self._canman:
            self._canman.set_position(self.motor_id, position_deg)
            self._last_command_time = time.time()

    def set_motor_torque(self, value: float) -> None:
        """Set motor torque (Nm) - core functionality as requested by user"""
        # Torque to current: T = I * Kt
        current = value / self._motor_params["Kt_actual"]
        self.set_motor_current(current)

    def set_output_torque(self, value: float) -> None:
        """Set output torque (Nm) - core functionality as requested by user"""
        # Output torque to motor torque: T_motor = T_output * gear_ratio
        motor_torque = value * self.gear_ratio
        self.set_motor_torque(motor_torque)

    def set_motor_velocity(self, value: float) -> None:
        """Set motor velocity (rad/s)"""
        motor_params = cast(dict[str, Any], self._motor_params)
        velocity_erpm = rad_per_sec_to_erpm(value, motor_params["NUM_POLE_PAIRS"])
        if not self.is_offline and self._canman:
            self._canman.set_velocity(self.motor_id, velocity_erpm)
            self._last_command_time = time.time()

    def set_output_velocity(self, value: float) -> None:
        """Set output velocity (rad/s)"""
        motor_velocity = value * self.gear_ratio
        self.set_motor_velocity(motor_velocity)

    # ============ Unsupported PID Functions - TMotor Servo Mode Handles All Control Loops Internally ============

    def set_current_gains(self, kp: float, ki: float, kd: float, ff: float) -> None:
        """TMotor servo mode does not support external current PID gains - motor handles current control internally"""
        raise NotImplementedError(
            "TMotor servo mode does not support external current PID gains. "
            "The motor handles current control internally."
        )

    def set_position_gains(self, kp: float, ki: float, kd: float, ff: float) -> None:
        """TMotor servo mode does not support external position PID gains - motor handles position control internally"""
        raise NotImplementedError(
            "TMotor servo mode does not support external position PID gains. "
            "The motor handles position control internally."
        )

    def set_impedance_gains(self, kp: float, ki: float, kd: float, k: float, b: float, ff: float) -> None:
        """TMotor servo mode does not support impedance control"""
        raise NotImplementedError(
            "TMotor servo mode does not support impedance control. "
            "Use position, velocity, or current control modes instead."
        )

    # ============ State Properties ============

    @property
    def motor_position(self) -> float:
        """Motor position (radians)"""
        return degrees_to_radians(self._motor_state.position)

    @property
    def output_position(self) -> float:
        """Output position (radians)"""
        return self.motor_position / self.gear_ratio

    @property
    def motor_velocity(self) -> float:
        """Motor velocity (rad/s)"""
        motor_params = cast(dict[str, Any], self._motor_params)
        return erpm_to_rad_per_sec(self._motor_state.velocity, motor_params["NUM_POLE_PAIRS"])

    @property
    def output_velocity(self) -> float:
        """Output velocity (rad/s)"""
        return self.motor_velocity / self.gear_ratio

    @property
    def motor_voltage(self) -> float:
        """Motor voltage (estimated)"""
        return 24.0  # Cannot get directly in servo mode

    @property
    def motor_current(self) -> float:
        """Motor current (A)"""
        return self._motor_state.current

    @property
    def motor_torque(self) -> float:
        """Motor torque (Nm)"""
        motor_params = cast(dict[str, Any], self._motor_params)
        return self.motor_current * cast(float, motor_params["Kt_actual"])

    @property
    def output_torque(self) -> float:
        """Output torque (Nm)"""
        return self.motor_torque / self.gear_ratio

    @property
    def case_temperature(self) -> float:
        """Case temperature (°C)"""
        return self._motor_state.temperature

    @property
    def winding_temperature(self) -> float:
        """Winding temperature (°C)"""
        return cast(float, getattr(self._thermal_model, "T_w", self.case_temperature))

    def __str__(self) -> str:
        """State string"""
        return (
            f"{self.device_info_string()} | "
            f"Pos: {self.output_position:.3f}rad | "
            f"Vel: {self.output_velocity:.3f}rad/s | "
            f"Torque: {self.output_torque:.3f}Nm | "
            f"Current: {self.motor_current:.3f}A | "
            f"Temp: {self.case_temperature:.1f}°C"
        )
    
    @property
    def max_case_temperature(self):
        return self.motor_constants.MAX_CASE_TEMPERATURE

    @property
    def max_winding_temperature(self):
        return self.motor_constants.MAX_WINDING_TEMPERATURE

    def set_control_mode(self, mode):
        """Set control mode"""
        if hasattr(self._CONTROL_MODE_CONFIGS, mode.value):
            config = getattr(self._CONTROL_MODE_CONFIGS, mode.value)
            if self._control_mode:
                old_config = getattr(self._CONTROL_MODE_CONFIGS, self._control_mode.value)
                old_config.exit_callback(self)
            config.entry_callback(self)
            self._control_mode = mode
        else:
            raise ValueError(f"Unsupported control mode: {mode}")

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()

    def verify_control_mode(self) -> bool:
        """Verify current control mode matches expected mode"""
        if self._servo_mode == ServoControlMode.CURRENT:
            # Send small current test command
            test_current = 0.1  # 100mA test
            self.set_motor_current(test_current)
            time.sleep(0.05)
            self.update()

            # Check if motor responds to current command
            if abs(self.motor_current - test_current) < 0.5:  # 500mA tolerance
                return True
            else:
                logging.warning(f"Control mode verification failed. Expected: {test_current}A, Got: {self.motor_current}A")
                return False
        return True

    def get_detailed_status(self) -> dict:
        """Get detailed motor status for debugging"""
        return {
            "motor_id": self.motor_id,
            "control_mode": self._servo_mode.name if self._servo_mode else "UNKNOWN",
            "is_open": self._is_open,
            "is_streaming": self._is_streaming,
            "is_homed": self._is_homed,
            "position": self.motor_position,
            "velocity": self.motor_velocity,
            "current": self.motor_current,
            "temperature": self.case_temperature,
            "error_code": self._error_code,
            "error_message": self._error_message,
            "last_command_time": self._last_command_time,
            "last_update_time": self._last_update_time,
        }


if __name__ == "__main__":
    print("TMotor Servo Mode Current Control Test")

    # Set up debug mode
    logging.basicConfig(level=logging.INFO)

    try:
        actuator = TMotorServoActuator(motor_type="AK80-9", motor_id=1, offline=False)

        # Enable debug mode
        if actuator._canman and hasattr(actuator._canman, 'enable_debug'):
            actuator._canman.enable_debug(True)

        with actuator as motor:
            motor_actuator = cast(TMotorServoActuator, motor)

            print(f"Motor initialized: {motor_actuator.device_info_string()}")
            print(f"Initial status: {motor_actuator.get_detailed_status()}")

            # Home motor
            motor_actuator.home()
            print("Motor homed")

            # Set current control mode
            motor_actuator.set_control_mode(CONTROL_MODES.CURRENT)
            print("Current control mode set")

            # Verify control mode
            if motor_actuator.verify_control_mode():
                print("✅ Control mode verification passed")
            else:
                print("❌ Control mode verification failed")
                raise RuntimeError("Control mode verification failed")

            # Send current command
            target_current = 2.0  # 2A test current
            motor_actuator.set_motor_current(target_current)
            print(f"Target current set to: {target_current}A")

            # Monitor loop
            loop = SoftRealtimeLoop(dt=0.1, report=False, fade=0)

            for t in loop:
                if t > 3.0:  # 3 second test
                    break

                motor_actuator.update()

                if int(t * 10) % 5 == 0:  # Print every 0.5 seconds
                    status = motor_actuator.get_detailed_status()
                    print(f"Time: {t:.1f}s | Current: {status['current']:.3f}A | "
                          f"Mode: {status['control_mode']} | Error: {status['error_code']}")

                    if status['error_code'] is not None:
                        print(f"❌ Error detected: {status['error_message']}")
                        break

            # Safe stop
            motor_actuator.set_motor_current(0.0)
            print("Motor current set to 0A")

    except Exception as e:
        print(f"❌ Test failed: {e}")
        import traceback
        traceback.print_exc()

    print("Current control test completed!")
