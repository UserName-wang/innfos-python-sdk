#!/usr/bin/env python3
"""
Simple test script to verify the package can be imported
"""

def test_imports():
    try:
        import innfos_python_sdk
        print("✓ innfos_python_sdk imported successfully")
        
        # Test importing individual modules
        from innfos_python_sdk import ActuatorController
        print("✓ ActuatorController imported successfully")
        
        from innfos_python_sdk import Actuator
        print("✓ Actuator imported successfully")
        
        from innfos_python_sdk import GluonProtocol
        print("✓ GluonProtocol imported successfully")
        
        from innfos_python_sdk import ErrorsDefine, ControlMode, Command
        print("✓ Constants imported successfully")
        
        from innfos_python_sdk import GluonError, ProtocolError, ActuatorError
        print("✓ Exceptions imported successfully")
        
        # Test creating instances
        controller = ActuatorController()
        print("✓ ActuatorController instance created successfully")
        
        protocol = GluonProtocol()
        print("✓ GluonProtocol instance created successfully")
        
        print("\nAll tests passed! Package is ready to use.")
        return True
        
    except Exception as e:
        print(f"✗ Import test failed: {e}")
        return False

if __name__ == "__main__":
    test_imports()