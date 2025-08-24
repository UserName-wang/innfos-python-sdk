#!/usr/bin/env python3
"""
Packet analysis script to examine the handshake packet format
"""

from innfos_python_sdk import GluonProtocol, Command
from innfos_python_sdk.protocol import CRC16
import struct

def analyze_handshake_packet():
    """Analyze the handshake packet format"""
    print("Analyzing handshake packet format")
    print("=" * 40)
    
    protocol = GluonProtocol()
    
    # Build handshake packet
    packet = protocol._build_packet(0x00, Command.HANDSHAKE, b'')
    print(f"Handshake packet: {packet.hex()}")
    print(f"Packet length: {len(packet)} bytes")
    print()
    
    # Break down the packet
    print("Packet breakdown:")
    print(f"  Header: 0x{packet[0]:02X} (should be 0xEE)")
    print(f"  Actuator ID: 0x{packet[1]:02X} (0x00 for broadcast)")
    print(f"  Command: 0x{packet[2]:02X} (0x44 for HANDSHAKE)")
    print(f"  Data length: 0x{packet[3]:02X}{packet[4]:02X} ({int.from_bytes(packet[3:5], 'big')} bytes)")
    print(f"  Data: {packet[5:5].hex() if len(packet) > 5 else 'None'}")
    print(f"  CRC: 0x{packet[-3]:02X}{packet[-2]:02X}")
    print(f"  Tail: 0x{packet[-1]:02X} (should be 0xED)")
    print()
    
    # Verify CRC calculation
    print("CRC verification:")
    crc_data = packet[1:-3]  # Everything except header and CRC+tail
    calculated_crc = CRC16.calculate(crc_data)
    packet_crc = struct.unpack('>H', packet[-3:-1])[0]
    print(f"  CRC data: {crc_data.hex()}")
    print(f"  Calculated CRC: 0x{calculated_crc:04X}")
    print(f"  Packet CRC: 0x{packet_crc:04X}")
    print(f"  CRC match: {calculated_crc == packet_crc}")
    print()
    
    # Compare with expected values from documentation
    print("Expected packet format based on documentation:")
    print("  Header: 0xEE")
    print("  ID: 0x00 (broadcast)")
    print("  Command: 0x44 (HANDSHAKE)")
    print("  Length: 0x0000 (no data)")
    print("  CRC: Should match calculated value")
    print("  Tail: 0xED")

def analyze_query_packet():
    """Analyze the query actuators packet format"""
    print("\nAnalyzing query actuators packet format")
    print("=" * 40)
    
    protocol = GluonProtocol()
    
    # Build query actuators packet
    packet = protocol._build_packet(0x00, Command.QUERY_ACTUATORS, b'')
    print(f"Query packet: {packet.hex()}")
    print(f"Packet length: {len(packet)} bytes")
    print()
    
    # Break down the packet
    print("Packet breakdown:")
    print(f"  Header: 0x{packet[0]:02X} (should be 0xEE)")
    print(f"  Actuator ID: 0x{packet[1]:02X} (0x00 for broadcast)")
    print(f"  Command: 0x{packet[2]:02X} (0x02 for QUERY_ACTUATORS)")
    print(f"  Data length: 0x{packet[3]:02X}{packet[4]:02X} ({int.from_bytes(packet[3:5], 'big')} bytes)")
    print(f"  Data: {packet[5:5].hex() if len(packet) > 5 else 'None'}")
    print(f"  CRC: 0x{packet[-3]:02X}{packet[-2]:02X}")
    print(f"  Tail: 0x{packet[-1]:02X} (should be 0xED)")

def main():
    analyze_handshake_packet()
    analyze_query_packet()
    
    print("\n" + "=" * 50)
    print("NEXT STEPS:")
    print("1. Use Wireshark to capture actual network traffic")
    print("2. Compare our packet format with known working implementations")
    print("3. Check if there are any differences in protocol implementation")

if __name__ == "__main__":
    main()