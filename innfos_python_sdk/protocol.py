"""
Protocol implementation for INNFOS Gluon robotic arm
Based on MINTASCA communication protocol over UDP
"""

import socket
import struct
import time
from typing import Tuple, Optional, List
from .constants import PROTOCOL, Command
from .exceptions import ProtocolError, CommunicationError

class CRC16:
    """CRC16 calculation helper class"""
    
    # CRC high byte table
    CRCH_TABLE = [
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
        0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
        0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40
    ]
    
    # CRC low byte table
    CRCL_TABLE = [
        0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7,
        0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E,
        0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9,
        0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC,
        0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
        0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32,
        0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D,
        0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38,
        0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF,
        0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
        0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1,
        0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4,
        0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB,
        0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA,
        0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
        0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0,
        0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97,
        0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E,
        0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89,
        0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
        0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83,
        0x41, 0x81, 0x80, 0x40
    ]
    
    @staticmethod
    def calculate(data: bytes) -> int:
        """
        Calculate CRC16 checksum for the given data
        
        Args:
            data: Bytes to calculate CRC for
            
        Returns:
            16-bit CRC checksum
        """
        crch = 0xFF
        crcl = 0xFF
        
        for byte in data:
            index = crch ^ byte
            crch = crcl ^ CRC16.CRCH_TABLE[index]
            crcl = CRC16.CRCL_TABLE[index]
            
        return (crch << 8) | crcl


class GluonProtocol:
    """
    Implements the MINTASCA communication protocol for INNFOS actuators
    """
    
    def __init__(self, ip: str = PROTOCOL['DEFAULT_IP'], port: int = PROTOCOL['DEFAULT_PORT']):
        """
        Initialize the protocol handler
        
        Args:
            ip: IP address of the ECB (Ethernet to CAN Bridge)
            port: UDP port for communication (default 2000)
        """
        self.ip = ip
        self.port = port
        self.socket = None
        self.timeout = 1.0  # 1 second timeout
        
    def connect(self) -> bool:
        """
        Establish UDP connection to the ECB
        
        Returns:
            True if connection successful, False otherwise
        """
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket.settimeout(self.timeout)
            return True
        except Exception as e:
            raise CommunicationError(f"Failed to create UDP socket: {e}")
    
    def disconnect(self):
        """Close the UDP connection"""
        if self.socket:
            self.socket.close()
            self.socket = None
    
    def _build_packet(self, actuator_id: int, command: int, data: bytes = b'') -> bytes:
        """
        Build a protocol packet
        
        Packet format:
        [HEADER][ID][CMD][DATA_LEN][DATA][CRC][TAIL]
        
        Args:
            actuator_id: ID of the actuator (0x00 for broadcast)
            command: Command code
            data: Data payload (optional)
            
        Returns:
            Complete packet as bytes
        """
        # Build the core packet (without CRC)
        packet = bytearray()
        packet.append(PROTOCOL['HEADER'])  # Header
        packet.append(actuator_id)         # Actuator ID
        packet.append(command)             # Command
        packet.extend(struct.pack('>H', len(data)))  # Data length (big endian)
        packet.extend(data)                # Data payload
        
        # Calculate and append CRC
        crc_data = packet[1:]  # CRC is calculated on everything except header
        crc = CRC16.calculate(crc_data)
        packet.extend(struct.pack('>H', crc))  # CRC (big endian)
        
        packet.append(PROTOCOL['TAIL'])    # Tail
        return bytes(packet)
    
    def _parse_packet(self, packet: bytes) -> Tuple[int, int, int, bytes]:
        """
        Parse a response packet
        
        Args:
            packet: Raw packet bytes
            
        Returns:
            Tuple of (actuator_id, command, data_length, data)
            
        Raises:
            ProtocolError: If packet format is invalid
        """
        if len(packet) < 6:  # Minimum packet size
            raise ProtocolError("Packet too short")
            
        if packet[0] != PROTOCOL['HEADER']:
            raise ProtocolError("Invalid packet header")
            
        if packet[-1] != PROTOCOL['TAIL']:
            raise ProtocolError("Invalid packet tail")
            
        actuator_id = packet[1]
        command = packet[2]
        data_length = struct.unpack('>H', packet[3:5])[0]
        
        # Extract data and CRC
        if len(packet) < 6 + data_length + 2:  # Header(1) + ID(1) + CMD(1) + LEN(2) + DATA + CRC(2) + TAIL(1)
            raise ProtocolError("Packet length mismatch")
            
        data = packet[5:5+data_length]
        received_crc = struct.unpack('>H', packet[5+data_length:7+data_length])[0]
        
        # Verify CRC
        crc_data = packet[1:5+data_length]  # Everything except header and CRC+tail
        calculated_crc = CRC16.calculate(crc_data)
        
        if received_crc != calculated_crc:
            raise ProtocolError(f"CRC mismatch: expected {calculated_crc:04X}, got {received_crc:04X}")
            
        return actuator_id, command, data_length, data
    
    def send_command(self, actuator_id: int, command: int, data: bytes = b'') -> Optional[bytes]:
        """
        Send a command to an actuator and receive response
        
        Args:
            actuator_id: ID of the actuator
            command: Command code
            data: Data payload (optional)
            
        Returns:
            Response data bytes, or None if no response expected
        """
        if not self.socket:
            raise CommunicationError("Not connected. Call connect() first.")
            
        # Build and send packet
        packet = self._build_packet(actuator_id, command, data)
        
        try:
            self.socket.sendto(packet, (self.ip, self.port))
            
            # For commands that don't expect a response
            if command in [Command.SET_CURRENT, Command.SET_SPEED, Command.SET_POSITION]:
                return None
                
            # Wait for response
            response, addr = self.socket.recvfrom(1024)
            actuator_id, resp_command, data_length, data = self._parse_packet(response)
            
            # Verify response is for the correct command
            if resp_command != command:
                raise ProtocolError(f"Command mismatch: expected {command:02X}, got {resp_command:02X}")
                
            return data
            
        except socket.timeout:
            raise CommunicationError("Timeout waiting for response")
        except Exception as e:
            raise CommunicationError(f"Communication error: {e}")
    
    def handshake(self) -> bool:
        """
        Perform handshake with ECB
        
        Returns:
            True if handshake successful
        """
        try:
            response = self.send_command(PROTOCOL['BROADCAST_ID'], Command.HANDSHAKE)
            if response and len(response) >= 1 and response[0] == 0x01:
                return True
            return False
        except:
            return False
    
    def query_actuators(self) -> List[Tuple[int, int]]:
        """
        Query connected actuators
        
        Returns:
            List of tuples (actuator_id, serial_number)
        """
        actuators = []
        try:
            # Send query command
            packet = self._build_packet(PROTOCOL['BROADCAST_ID'], Command.QUERY_ACTUATORS)
            self.socket.sendto(packet, (self.ip, self.port))
            
            # Wait for responses (there might be multiple)
            start_time = time.time()
            while time.time() - start_time < 1.0:  # Wait up to 1 second
                try:
                    response, addr = self.socket.recvfrom(1024)
                    actuator_id, command, data_length, data = self._parse_packet(response)
                    
                    if command == Command.QUERY_ACTUATORS and data_length >= 5:
                        # Extract actuator ID and serial number
                        actuator_id = response[1]
                        serial_number = struct.unpack('>I', data[0:4])[0]  # 4 bytes serial number
                        actuators.append((actuator_id, serial_number))
                        
                except socket.timeout:
                    break  # No more responses
                except ProtocolError:
                    continue  # Ignore invalid packets
        except Exception as e:
            raise CommunicationError(f"Failed to query actuators: {e}")
            
        return actuators