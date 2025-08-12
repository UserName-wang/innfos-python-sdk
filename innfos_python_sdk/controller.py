"""
Main controller for INNFOS Gluon robotic arm
"""

from typing import List, Tuple, Optional
from .protocol import GluonProtocol
from .actuator import Actuator
from .constants import PROTOCOL, ErrorsDefine
from .exceptions import GluonError

class ActuatorController:
    """
    Main controller class for INNFOS Gluon robotic arm
    Manages communication with all actuators and provides high-level control interface
    """
    
    _instance = None
    
    def __init__(self, ip: str = PROTOCOL['DEFAULT_IP'], port: int = PROTOCOL['DEFAULT_PORT']):
        """
        Initialize the controller
        
        Args:
            ip: IP address of the ECB (Ethernet to CAN Bridge)
            port: UDP port for communication (default 2000)
        """
        self.protocol = GluonProtocol(ip, port)
        self.actuators = {}  # Dict of actuator ID -> Actuator object
        self.connected = False
    
    @classmethod
    def init_controller(cls, ip: str = PROTOCOL['DEFAULT_IP'], port: int = PROTOCOL['DEFAULT_PORT']):
        """
        Initialize and return the singleton controller instance
        
        Args:
            ip: IP address of the ECB
            port: UDP port for communication
            
        Returns:
            ActuatorController instance
        """
        if cls._instance is None:
            cls._instance = cls(ip, port)
        return cls._instance
    
    @classmethod
    def get_instance(cls):
        """
        Get the singleton controller instance
        
        Returns:
            ActuatorController instance
        """
        if cls._instance is None:
            cls._instance = cls()
        return cls._instance
    
    def connect(self) -> bool:
        """
        Connect to the ECB and initialize communication
        
        Returns:
            True if connection successful, False otherwise
        """
        try:
            if self.protocol.connect():
                # Perform handshake
                if self.protocol.handshake():
                    self.connected = True
                    return True
            return False
        except Exception as e:
            self.connected = False
            raise GluonError(f"Failed to connect: {e}")
    
    def disconnect(self):
        """
        Disconnect from the ECB
        """
        # Disable all actuators
        for actuator in self.actuators.values():
            try:
                actuator.disable()
            except:
                pass  # Ignore errors during disconnect
        
        self.protocol.disconnect()
        self.connected = False
    
    def lookup_actuators(self, error_code: Optional[ErrorsDefine] = None) -> List[Tuple[int, int]]:
        """
        Search for and identify all connected actuators
        
        Args:
            error_code: Optional error code reference to be set if an error occurs
            
        Returns:
            List of tuples (actuator_id, serial_number)
        """
        if not self.connected:
            if error_code is not None:
                error_code = ErrorsDefine.ERR_CAN_DISCONNECTION
            return []
        
        try:
            actuators = self.protocol.query_actuators()
            
            # Create Actuator objects for each found actuator
            self.actuators = {}
            for actuator_id, serial_number in actuators:
                self.actuators[actuator_id] = Actuator(self.protocol, actuator_id)
            
            if error_code is not None:
                error_code = ErrorsDefine.ERR_NONE
                
            return actuators
            
        except Exception as e:
            if error_code is not None:
                error_code = ErrorsDefine.ERR_CAN_COMMUNICATION
            raise GluonError(f"Failed to lookup actuators: {e}")
    
    def has_available_actuator(self) -> bool:
        """
        Check if any actuators are currently connected and available
        
        Returns:
            True if at least one actuator is connected, False otherwise
        """
        return len(self.actuators) > 0
    
    def get_actuator_id_array(self) -> List[int]:
        """
        Get a list of IDs for all currently connected actuators
        
        Returns:
            List of actuator IDs
        """
        return list(self.actuators.keys())
    
    def get_actuator_unified_id_array(self) -> List[Tuple[int, str]]:
        """
        Get a list of UnifiedID objects for all currently connected actuators
        UnifiedID consists of actuator ID and IP address
        
        Returns:
            List of tuples (actuator_id, ip_address)
        """
        return [(actuator_id, self.protocol.ip) for actuator_id in self.actuators.keys()]
    
    def get_unified_id_group(self, ip_address: str) -> List[int]:
        """
        Get a list of actuator IDs that belong to a specific IP address group
        
        Args:
            ip_address: The IP address to query for connected actuators
            
        Returns:
            List of actuator IDs connected to the specified IP address
        """
        if ip_address == self.protocol.ip:
            return list(self.actuators.keys())
        else:
            return []
    
    def get_actuator(self, actuator_id: int) -> Optional[Actuator]:
        """
        Get an actuator by its ID
        
        Args:
            actuator_id: ID of the actuator to retrieve
            
        Returns:
            Actuator object or None if not found
        """
        return self.actuators.get(actuator_id)
    
    def process_events(self):
        """
        Process controller events
        This method should be called regularly to handle communication with the actuators
        """
        # In a more complex implementation, this would handle asynchronous events
        # For now, it's a placeholder
        pass
    
    def enable_all_actuators(self) -> bool:
        """
        Enable all connected actuators
        
        Returns:
            True if all actuators enabled successfully, False otherwise
        """
        success = True
        for actuator in self.actuators.values():
            if not actuator.enable():
                success = False
        return success
    
    def disable_all_actuators(self) -> bool:
        """
        Disable all connected actuators
        
        Returns:
            True if all actuators disabled successfully, False otherwise
        """
        success = True
        for actuator in self.actuators.values():
            if not actuator.disable():
                success = False
        return success
    
    def set_all_positions(self, positions: List[float]) -> bool:
        """
        Set positions for all actuators simultaneously
        
        Args:
            positions: List of target positions for each actuator
            
        Returns:
            True if commands sent successfully
        """
        if len(positions) != len(self.actuators):
            raise GluonError("Number of positions must match number of actuators")
        
        actuator_ids = sorted(self.actuators.keys())
        for i, actuator_id in enumerate(actuator_ids):
            actuator = self.actuators[actuator_id]
            actuator.set_position(positions[i])
        
        return True
    
    def get_all_positions(self) -> List[float]:
        """
        Get positions for all actuators
        
        Returns:
            List of current positions for each actuator
        """
        positions = []
        actuator_ids = sorted(self.actuators.keys())
        for actuator_id in actuator_ids:
            actuator = self.actuators[actuator_id]
            position = actuator.get_position()
            positions.append(position if position is not None else 0.0)
        
        return positions