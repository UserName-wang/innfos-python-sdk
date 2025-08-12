"""
Actuator class for controlling individual INNFOS smart actuators
"""

import struct
from typing import Union
from .protocol import GluonProtocol
from .constants import Command, ControlMode
from .exceptions import ActuatorError

class Actuator:
    """
    Represents a single INNFOS smart actuator
    """
    
    def __init__(self, protocol: GluonProtocol, actuator_id: int):
        """
        Initialize an actuator
        
        Args:
            protocol: GluonProtocol instance for communication
            actuator_id: ID of the actuator
        """
        self.protocol = protocol
        self.id = actuator_id
        self.enabled = False
    
    def enable(self) -> bool:
        """
        Enable the actuator
        
        Returns:
            True if successful, False otherwise
        """
        try:
            data = struct.pack('B', 0x01)  # 0x01 = enable
            response = self.protocol.send_command(self.id, Command.ENABLE_ACTUATOR, data)
            if response and len(response) >= 1 and response[0] == 0x01:
                self.enabled = True
                return True
            return False
        except Exception as e:
            raise ActuatorError(f"Failed to enable actuator {self.id}: {e}")
    
    def disable(self) -> bool:
        """
        Disable the actuator
        
        Returns:
            True if successful, False otherwise
        """
        try:
            data = struct.pack('B', 0x00)  # 0x00 = disable
            response = self.protocol.send_command(self.id, Command.ENABLE_ACTUATOR, data)
            if response and len(response) >= 1 and response[0] == 0x01:
                self.enabled = False
                return True
            return False
        except Exception as e:
            raise ActuatorError(f"Failed to disable actuator {self.id}: {e}")
    
    def set_mode(self, mode: int) -> bool:
        """
        Set the control mode of the actuator
        
        Args:
            mode: Control mode from ControlMode class
            
        Returns:
            True if successful, False otherwise
        """
        try:
            data = struct.pack('B', mode)
            response = self.protocol.send_command(self.id, Command.SET_MODE, data)
            if response and len(response) >= 1 and response[0] == 0x01:
                return True
            return False
        except Exception as e:
            raise ActuatorError(f"Failed to set mode for actuator {self.id}: {e}")
    
    def set_position(self, position: float) -> bool:
        """
        Set the target position of the actuator
        
        Args:
            position: Target position in rotations (-128.0 to 127.999999940)
            
        Returns:
            True if command sent successfully
        """
        try:
            # Convert position to IQ24 format
            iq24_position = int(position * (2**24))
            # Pack as 4-byte big-endian signed integer
            data = struct.pack('>i', iq24_position)
            self.protocol.send_command(self.id, Command.SET_POSITION, data)
            return True
        except Exception as e:
            raise ActuatorError(f"Failed to set position for actuator {self.id}: {e}")
    
    def set_speed(self, speed: float) -> bool:
        """
        Set the target speed of the actuator
        
        Args:
            speed: Target speed as fraction of max speed (-1.0 to 1.0)
                  Actual speed = speed * 6000 RPM
            
        Returns:
            True if command sent successfully
        """
        try:
            # Convert speed to IQ24 format
            iq24_speed = int(speed * (2**24))
            # Pack as 4-byte big-endian signed integer
            data = struct.pack('>i', iq24_speed)
            self.protocol.send_command(self.id, Command.SET_SPEED, data)
            return True
        except Exception as e:
            raise ActuatorError(f"Failed to set speed for actuator {self.id}: {e}")
    
    def set_current(self, current: float) -> bool:
        """
        Set the target current of the actuator
        
        Args:
            current: Target current as fraction of max current (-1.0 to 1.0)
                     Actual current = current * max_current (depends on model)
            
        Returns:
            True if command sent successfully
        """
        try:
            # Convert current to IQ24 format
            iq24_current = int(current * (2**24))
            # Pack as 4-byte big-endian signed integer
            data = struct.pack('>i', iq24_current)
            self.protocol.send_command(self.id, Command.SET_CURRENT, data)
            return True
        except Exception as e:
            raise ActuatorError(f"Failed to set current for actuator {self.id}: {e}")
    
    def get_position(self) -> Union[float, None]:
        """
        Get the current position of the actuator
        
        Returns:
            Current position in rotations, or None if failed
        """
        try:
            response = self.protocol.send_command(self.id, Command.READ_POSITION)
            if response and len(response) == 4:
                # Unpack IQ24 format position
                iq24_position = struct.unpack('>i', response)[0]
                position = iq24_position / (2**24)
                return position
            return None
        except Exception as e:
            raise ActuatorError(f"Failed to read position for actuator {self.id}: {e}")
    
    def get_speed(self) -> Union[float, None]:
        """
        Get the current speed of the actuator
        
        Returns:
            Current speed as fraction of max speed, or None if failed
        """
        try:
            response = self.protocol.send_command(self.id, Command.READ_SPEED)
            if response and len(response) == 4:
                # Unpack IQ24 format speed
                iq24_speed = struct.unpack('>i', response)[0]
                speed = iq24_speed / (2**24)
                return speed
            return None
        except Exception as e:
            raise ActuatorError(f"Failed to read speed for actuator {self.id}: {e}")
    
    def get_current(self) -> Union[float, None]:
        """
        Get the current current (electrical current) of the actuator
        
        Returns:
            Current current as fraction of max current, or None if failed
        """
        try:
            response = self.protocol.send_command(self.id, Command.READ_CURRENT)
            if response and len(response) == 4:
                # Unpack IQ24 format current
                iq24_current = struct.unpack('>i', response)[0]
                current = iq24_current / (2**24)
                return current
            return None
        except Exception as e:
            raise ActuatorError(f"Failed to read current for actuator {self.id}: {e}")
    
    def get_voltage(self) -> Union[float, None]:
        """
        Get the current voltage of the actuator
        
        Returns:
            Current voltage in volts, or None if failed
        """
        try:
            response = self.protocol.send_command(self.id, Command.READ_VOLTAGE)
            if response and len(response) == 2:
                # Unpack IQ10 format voltage (value is real value * 2^10)
                iq10_voltage = struct.unpack('>H', response)[0]
                voltage = iq10_voltage / (2**10)
                return voltage
            return None
        except Exception as e:
            raise ActuatorError(f"Failed to read voltage for actuator {self.id}: {e}")
    
    def get_temperature(self) -> Union[float, None]:
        """
        Get the current temperature of the actuator motor
        
        Returns:
            Current temperature in Celsius, or None if failed
        """
        try:
            response = self.protocol.send_command(self.id, Command.READ_TEMPERATURE)
            if response and len(response) == 2:
                # Unpack IQ8 format temperature (value is real value * 2^8)
                iq8_temperature = struct.unpack('>H', response)[0]
                temperature = iq8_temperature / (2**8)
                return temperature
            return None
        except Exception as e:
            raise ActuatorError(f"Failed to read temperature for actuator {self.id}: {e}")
    
    def store_parameters(self) -> bool:
        """
        Store current parameters to EEPROM
        
        Returns:
            True if successful, False otherwise
        """
        try:
            response = self.protocol.send_command(self.id, Command.STORE_PARAMETERS)
            if response and len(response) >= 1 and response[0] == 0x01:
                return True
            return False
        except Exception as e:
            raise ActuatorError(f"Failed to store parameters for actuator {self.id}: {e}")