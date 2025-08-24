#!/usr/bin/env python3
"""
Diagnostic script to troubleshoot connection issues with INNFOS Python SDK
"""

import socket
import time
from innfos_python_sdk import ActuatorController, GluonProtocol, Command

def test_basic_udp_connection(ip, port):
    """Test basic UDP connectivity to the device"""
    print(f"1. Testing basic UDP connectivity to {ip}:{port}")
    try:
        # Create a UDP socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(2.0)
        
        # Send a simple message
        message = b"ping"
        sock.sendto(message, (ip, port))
        print("   ✓ UDP socket created and message sent")
        
        # Try to receive a response (this might timeout, which is OK)
        try:
            response, addr = sock.recvfrom(1024)
            print(f"   ✓ Response received from {addr}: {response}")
        except socket.timeout:
            print("   → No response received (this might be normal)")
        
        sock.close()
        return True
    except Exception as e:
        print(f"   ✗ UDP connection test failed: {e}")
        return False

def test_handshake_process():
    """Test the handshake process step by step"""
    print("\n2. Testing handshake process")
    try:
        protocol = GluonProtocol()
        if not protocol.connect():
            print("   ✗ Failed to create socket")
            return False
            
        print("   ✓ Socket created successfully")
        
        # Build handshake packet manually
        packet = protocol._build_packet(0x00, Command.HANDSHAKE, b'')
        print(f"   ✓ Handshake packet built: {packet.hex()}")
        
        # Send handshake
        protocol.socket.sendto(packet, (protocol.ip, protocol.port))
        print("   ✓ Handshake packet sent")
        
        # Wait for response
        try:
            response, addr = protocol.socket.recvfrom(1024)
            print(f"   ✓ Response received: {response.hex()}")
            
            # Try to parse the response
            try:
                actuator_id, command, data_length, data = protocol._parse_packet(response)
                print(f"     → Parsed response: ID={actuator_id}, CMD={command:02X}, LEN={data_length}, DATA={data.hex()}")
                if data and data[0] == 0x01:
                    print("     → Handshake successful!")
                    return True
                else:
                    print("     → Unexpected handshake response")
            except Exception as e:
                print(f"     ✗ Failed to parse response: {e}")
                
        except socket.timeout:
            print("   → Handshake timed out - no response from device")
            
    except Exception as e:
        print(f"   ✗ Handshake test failed: {e}")
    
    return False

def test_with_different_timeouts():
    """Test connection with different timeout values"""
    print("\n3. Testing with different timeouts")
    
    for timeout in [1.0, 3.0, 5.0]:
        print(f"   Testing with {timeout}s timeout:")
        try:
            protocol = GluonProtocol()
            protocol.timeout = timeout
            if protocol.connect():
                protocol.socket.settimeout(timeout)
                response = protocol.send_command(0x00, Command.HANDSHAKE)
                if response and len(response) >= 1 and response[0] == 0x01:
                    print(f"     ✓ Handshake successful with {timeout}s timeout")
                    return True
                else:
                    print(f"     → Handshake failed with {timeout}s timeout")
            protocol.disconnect()
        except Exception as e:
            print(f"     → Error with {timeout}s timeout: {e}")
    
    return False

def test_controller_connection():
    """Test using the full controller connection process"""
    print("\n4. Testing controller connection process")
    try:
        controller = ActuatorController.init_controller()
        print("   ✓ Controller initialized")
        
        result = controller.connect()
        if result:
            print("   ✓ Controller connected successfully")
            controller.disconnect()
            return True
        else:
            print("   → Controller connection returned False")
            
    except Exception as e:
        print(f"   ✗ Controller connection failed: {e}")
    
    return False

def main():
    print("INNFOS Python SDK Connection Diagnostic")
    print("=" * 50)
    
    # Default IP and port
    ip = "192.168.1.30"
    port = 2000
    
    print(f"Testing connection to: {ip}:{port}")
    print(f"You confirmed that ping to {ip} works, so network connectivity is OK")
    print()
    
    # Run all tests
    test1 = test_basic_udp_connection(ip, port)
    test2 = test_handshake_process()
    test3 = test_with_different_timeouts()
    test4 = test_controller_connection()
    
    print("\n" + "=" * 50)
    print("DIAGNOSTIC SUMMARY:")
    print(f"  Basic UDP connectivity: {'PASS' if test1 else 'FAIL'}")
    print(f"  Handshake process: {'PASS' if test2 else 'FAIL'}")
    print(f"  Different timeouts: {'PASS' if test3 else 'FAIL'}")
    print(f"  Controller connection: {'PASS' if test4 else 'FAIL'}")
    
    print("\nTROUBLESHOOTING TIPS:")
    if not test1:
        print("  - UDP connectivity issue - check firewall settings")
    if not test2:
        print("  - Handshake failing - check that ECB is properly powered")
        print("  - Make sure you're using the right IP address for your ECB")
    if not test3:
        print("  - Try increasing timeout values in protocol.py")
    if not test4:
        print("  - Full connection process failing")
    
    print("\nIf all tests fail, verify:")
    print("  1. The ECB is powered on")
    print("  2. Network cable is properly connected")
    print("  3. You're using the correct IP address")
    print("  4. No firewall is blocking UDP port 2000")
    print("  5. You're on the same network subnet as the ECB")

if __name__ == "__main__":
    main()