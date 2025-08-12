#!/usr/bin/env python3
"""
Simulation example showing how to test the SDK without physical hardware
"""

import time
import struct
from unittest.mock import Mock, patch
from innfos_python_sdk import ActuatorController, Actuator

def simulate_actuator_behavior():
    """
    Demonstrate how to simulate actuator behavior for testing
    """
    print("INNFOS Python SDK Simulation Example")
    print("=" * 40)
    
    # Create a mock protocol to simulate communication
    mock_protocol = Mock()
    
    # Simulate successful handshake
    mock_protocol.handshake.return_value = True
    
    # Simulate actuator discovery
    mock_protocol.query_actuators.return_value = [
        (1, 123456789),  # Actuator ID 1 with serial number
        (2, 987654321),  # Actuator ID 2 with serial number
    ]
    
    # Create controller with mocked protocol
    controller = ActuatorController()
    controller.protocol = mock_protocol
    controller.connected = True
    
    # Simulate finding actuators
    actuators = controller.lookup_actuators()
    print(f"Found {len(actuators)} simulated actuators:")
    for actuator_id, serial in actuators:
        print(f"  ID: {actuator_id}, Serial: {serial}")
    
    # Create mock actuators with simulated behavior
    actuator1 = Actuator(mock_protocol, 1)
    actuator2 = Actuator(mock_protocol, 2)
    
    # Simulate successful operations
    mock_protocol.send_command.return_value = b'\x01'  # Success response
    
    # For position reading, return a simulated position value
    # IQ24 format: 10.5 rotations = 10.5 * 2^24 = 176160768
    iq24_value = int(10.5 * (2**24))
    position_response = struct.pack('>i', iq24_value)
    
    # Set up side effects for different calls
    call_count = 0
    def side_effect(*args, **kwargs):
        nonlocal call_count
        call_count += 1
        if call_count == 1:
            return b'\x01'  # Enable response
        elif call_count == 2:
            return None     # Set position response (no response)
        elif call_count == 3:
            return position_response  # Get position response
        elif call_count == 4:
            return b'\x01'  # Disable response
        return b'\x01'
    
    mock_protocol.send_command.side_effect = side_effect
    
    print(f"\nControlling simulated actuator {actuator1.id}")
    
    # Enable actuator
    if actuator1.enable():
        print("  ✓ Actuator enabled")
    
    # Set position
    actuator1.set_position(10.5)
    print("  ✓ Position set to 10.5 rotations")
    
    # Get position
    position = actuator1.get_position()
    print(f"  ✓ Current position: {position} rotations")
    
    # Disable actuator
    if actuator1.disable():
        print("  ✓ Actuator disabled")
    
    print("\nSimulation completed successfully!")
    print("\nWhen you connect to real hardware, simply remove the mocking:")
    print("  # Instead of:")
    print("  controller = ActuatorController()")
    print("  controller.protocol = mock_protocol  # Remove this line")
    print("  ")
    print("  # Use:")
    print("  controller = ActuatorController.init_controller()")
    print("  controller.connect()")

def show_offline_api_usage():
    """
    Show how to use the API without connecting to hardware
    """
    print("\n\nAPI Usage Without Hardware Connection")
    print("=" * 40)
    
    # Show what you can do without hardware
    print("1. Import and instantiate classes:")
    controller = ActuatorController()
    print("   ✓ Controller created (not connected)")
    
    # Show available methods
    print("\n2. Available methods (can be called without hardware):")
    methods = [method for method in dir(controller) if not method.startswith('_')]
    for method in sorted(methods):
        print(f"   - {method}")
    
    print("\n3. Constants and error codes available:")
    from innfos_python_sdk import ErrorsDefine, ControlMode
    print(f"   - Error codes: ERR_NONE={ErrorsDefine.ERR_NONE}, ERR_ACTUATOR_LOCKED_ROTOR={ErrorsDefine.ERR_ACTUATOR_LOCKED_ROTOR}")
    print(f"   - Control modes: POSITION_MODE={ControlMode.TRAPEZOIDAL_POSITION_MODE}")
    
    print("\n4. Data format handling (can be tested without hardware):")
    from innfos_python_sdk.actuator import Actuator
    from innfos_python_sdk.protocol import CRC16
    
    # Test IQ format conversions
    position = 5.25  # 5.25 rotations
    iq24_value = int(position * (2**24))
    print(f"   - Position {position} rotations = IQ24 value {iq24_value}")
    print(f"   - Back to position: {iq24_value / (2**24)}")
    
    # Test CRC calculation
    test_data = b'\x01\x06\x00\x04\x00\x00\x00\x00'
    crc = CRC16.calculate(test_data)
    print(f"   - CRC16 of test data: 0x{crc:04X}")

if __name__ == "__main__":
    simulate_actuator_behavior()
    show_offline_api_usage()
    
    print("\n" + "=" * 50)
    print("CONCLUSION:")
    print("You can develop and test most of your code logic without")
    print("the physical robot arm. Just connect to real hardware")
    print("when you're ready for integration testing.")