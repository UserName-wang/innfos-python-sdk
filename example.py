#!/usr/bin/env python3
"""
Example script demonstrating usage of the INNFOS Python SDK
"""

import time
from innfos_python_sdk import ActuatorController, ControlMode

def main():
    print("INNFOS Gluon Python SDK Example")
    print("=" * 40)
    
    # Initialize the controller
    print("1. Initializing controller...")
    controller = ActuatorController.init_controller()
    
    # Connect to the ECB
    print("2. Connecting to ECB...")
    if not controller.connect():
        print("Failed to connect to ECB")
        return
    
    print("   Connected successfully")
    
    # Find connected actuators
    print("3. Looking for actuators...")
    try:
        actuators = controller.lookup_actuators()
        if actuators:
            print(f"   Found {len(actuators)} actuators:")
            for actuator_id, serial in actuators:
                print(f"     ID: {actuator_id}, Serial: {serial}")
        else:
            print("   No actuators found")
            return
    except Exception as e:
        print(f"   Error looking for actuators: {e}")
        return
    
    # Get actuator IDs
    actuator_ids = controller.get_actuator_id_array()
    if not actuator_ids:
        print("   No actuators available")
        return
    
    print(f"4. Working with actuators: {actuator_ids}")
    
    # Enable all actuators
    print("5. Enabling all actuators...")
    if controller.enable_all_actuators():
        print("   All actuators enabled")
    else:
        print("   Failed to enable all actuators")
        return
    
    try:
        # Demonstrate control of the first actuator
        first_actuator_id = actuator_ids[0]
        actuator = controller.get_actuator(first_actuator_id)
        
        print(f"6. Controlling actuator {first_actuator_id}")
        
        # Set position mode
        print("   Setting position mode...")
        if actuator.set_mode(ControlMode.TRAPEZOIDAL_POSITION_MODE):
            print("   Position mode set")
        else:
            print("   Failed to set position mode")
        
        # Get initial position
        initial_position = actuator.get_position()
        print(f"   Initial position: {initial_position}")
        
        # Move to a new position
        target_position = 5.0  # 5 full rotations
        print(f"   Moving to position {target_position}...")
        actuator.set_position(target_position)
        
        # Wait a bit and check position
        time.sleep(2)
        current_position = actuator.get_position()
        print(f"   Current position: {current_position}")
        
        # Move back to initial position
        print(f"   Moving back to position {initial_position}...")
        actuator.set_position(initial_position or 0.0)
        
        # Wait and check final position
        time.sleep(2)
        final_position = actuator.get_position()
        print(f"   Final position: {final_position}")
        
    except Exception as e:
        print(f"   Error controlling actuator: {e}")
    
    finally:
        # Disable all actuators
        print("7. Disabling all actuators...")
        if controller.disable_all_actuators():
            print("   All actuators disabled")
        else:
            print("   Failed to disable all actuators")
    
    # Disconnect
    print("8. Disconnecting...")
    controller.disconnect()
    print("   Disconnected")
    
    print("\nExample completed successfully!")

if __name__ == "__main__":
    main()