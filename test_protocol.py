#!/usr/bin/env python3

from innfos_python_sdk import ActuatorController

def test_protocol():
    print("Testing INNFOS Python SDK Protocol")
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
    except Exception as e:
        print(f"   Error looking for actuators: {e}")

if __name__ == "__main__":
    test_protocol()