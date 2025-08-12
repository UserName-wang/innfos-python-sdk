# INNFOS Python SDK

Pure Python implementation of the SDK for INNFOS Gluon robotic arm, based on the MINTASCA communication protocol.

## Overview

This is a pure Python SDK for controlling INNFOS smart actuators used in the Gluon robotic arm. Unlike the C++ SDK with Python bindings, this implementation is written entirely in Python, making it:

- Cross-platform compatible (x86, ARM, etc.)
- Easy to install and use
- No compilation required
- Perfect for integration with ROS 2 and other Python-based robotics frameworks

## Features

- Full implementation of the MINTASCA UDP communication protocol
- Control individual actuators or the entire robotic arm
- Position, speed, and current control modes
- Real-time monitoring of actuator status (position, speed, current, voltage, temperature)
- Automatic CRC16 checksum calculation and verification
- Error handling and exception management
- Compatible with Jetson and other ARM-based platforms
- Works without physical hardware for development and testing

## Installation

```bash
# Clone the repository
git clone https://github.com/innfos/gluon-python-sdk.git

# Navigate to the directory
cd innfos-python-sdk

# Install the package
pip install .
```

Or install in development mode:
```bash
pip install -e .
```

## Usage

### Basic Setup

```python
from innfos_python_sdk import ActuatorController

# Initialize the controller
controller = ActuatorController.init_controller()

# Connect to the ECB (Ethernet to CAN Bridge)
if controller.connect():
    print("Connected to ECB")
else:
    print("Failed to connect to ECB")
    exit(1)

# Find connected actuators
error_code = None
actuators = controller.lookup_actuators(error_code)

if actuators:
    print(f"Found {len(actuators)} actuators")
    for actuator_id, serial in actuators:
        print(f"  Actuator ID: {actuator_id}, Serial: {serial}")
else:
    print("No actuators found")
```

### Controlling Individual Actuators

```python
# Get the first actuator
actuator_ids = controller.get_actuator_id_array()
if actuator_ids:
    actuator_id = actuator_ids[0]
    actuator = controller.get_actuator(actuator_id)
    
    # Enable the actuator
    if actuator.enable():
        print(f"Actuator {actuator_id} enabled")
    
    # Set position (in rotations)
    actuator.set_position(10.0)  # Rotate 10 full turns
    
    # Get current position
    position = actuator.get_position()
    print(f"Current position: {position} rotations")
    
    # Disable the actuator
    actuator.disable()
```

### Controlling Multiple Actuators

```python
# Enable all actuators
controller.enable_all_actuators()

# Set positions for all actuators
positions = [0.0, 0.5, 1.0, 1.5, 2.0, 2.5]  # Example positions
controller.set_all_positions(positions)

# Get positions for all actuators
current_positions = controller.get_all_positions()
print(f"Current positions: {current_positions}")

# Disable all actuators
controller.disable_all_actuators()
```

## Development Without Hardware

You can develop and test your code without connecting to physical hardware using Python's mocking capabilities:

```python
from unittest.mock import Mock
from innfos_python_sdk import ActuatorController, Actuator

# Create a mock protocol
mock_protocol = Mock()
mock_protocol.handshake.return_value = True
mock_protocol.query_actuators.return_value = [(1, 123456789)]

# Use with controller
controller = ActuatorController()
controller.protocol = mock_protocol
controller.connected = True

# Test your code logic
actuators = controller.lookup_actuators()
# ... rest of your code
```

See [simulation_example.py](file:///home/panda/study/ros/ros_gluon/innfos-python-sdk/simulation_example.py) for a complete example of how to simulate actuator behavior for testing.

## API Reference

### ActuatorController

Main controller class for managing the robotic arm.

- `init_controller(ip, port)` - Initialize the controller
- `get_instance()` - Get the singleton instance
- `connect()` - Connect to the ECB
- `disconnect()` - Disconnect from the ECB
- `lookup_actuators()` - Find connected actuators
- `has_available_actuator()` - Check if any actuators are connected
- `get_actuator_id_array()` - Get list of actuator IDs
- `get_actuator_unified_id_array()` - Get list of actuator IDs with IP addresses
- `get_unified_id_group(ip)` - Get actuators connected to specific IP
- `get_actuator(id)` - Get actuator by ID
- `process_events()` - Process controller events
- `enable_all_actuators()` - Enable all actuators
- `disable_all_actuators()` - Disable all actuators
- `set_all_positions(positions)` - Set positions for all actuators
- `get_all_positions()` - Get positions for all actuators

### Actuator

Class representing an individual actuator.

- `enable()` - Enable the actuator
- `disable()` - Disable the actuator
- `set_mode(mode)` - Set control mode
- `set_position(position)` - Set target position
- `set_speed(speed)` - Set target speed
- `set_current(current)` - Set target current
- `get_position()` - Get current position
- `get_speed()` - Get current speed
- `get_current()` - Get current current
- `get_voltage()` - Get current voltage
- `get_temperature()` - Get motor temperature
- `store_parameters()` - Store parameters to EEPROM

## Project Structure

```
innfos-python-sdk/
├── innfos_python_sdk/           # Main Python package
│   ├── __init__.py             # Package initialization and exports
│   ├── constants.py            # Error codes, commands, and constants
│   ├── exceptions.py           # Custom exception classes
│   ├── protocol.py             # Low-level protocol implementation
│   ├── actuator.py             # Individual actuator control
│   └── controller.py           # Main controller class
├── setup.py                    # Package installation configuration
├── README.md                   # This file
├── PROJECT_SUMMARY.md          # Technical summary
├── example.py                  # Usage example
├── simulation_example.py        # Simulation example for testing without hardware
├── comprehensive_test.py       # Comprehensive test script
└── test_import.py              # Import verification script
```

## Requirements

- Python 3.7 or higher
- Network access to the ECB (Ethernet to CAN Bridge) for real hardware
- INNFOS Gluon robotic arm or compatible actuators (for real hardware testing)

## Platform Support

This pure Python implementation works on:

- x86/x64 platforms (Linux, Windows, macOS)
- ARM platforms (Jetson, Raspberry Pi, etc.)
- Any platform that supports Python 3.7+

## Integration with ROS 2

This SDK is designed for easy integration with ROS 2:

1. Create a ROS 2 package for your robot control
2. Import and use the SDK in your nodes
3. Handle the lifecycle (connection, control, disconnection)
4. Publish actuator states as ROS topics
5. Subscribe to control commands from ROS services or actions

Example node structure:
```python
import rclpy
from rclpy.node import Node
from innfos_python_sdk import ActuatorController

class GluonControllerNode(Node):
    def __init__(self):
        super().__init__('gluon_controller')
        self.controller = ActuatorController.init_controller()
        # ... ROS 2 integration code
        
def main(args=None):
    rclpy.init(args=args)
    node = GluonControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

## Documentation

- [PROJECT_SUMMARY.md](file:///home/panda/study/ros/ros_gluon/innfos-python-sdk/PROJECT_SUMMARY.md) - Complete technical summary of the implementation
- [example.py](file:///home/panda/study/ros/ros_gluon/innfos-python-sdk/example.py) - Basic usage example
- [simulation_example.py](file:///home/panda/study/ros/ros_gluon/innfos-python-sdk/simulation_example.py) - Example of testing without hardware
- This README - General usage instructions

## Testing

Run the import test:
```bash
python3 test_import.py
```

Run the comprehensive test:
```bash
python3 comprehensive_test.py
```

Run the simulation example:
```bash
python3 simulation_example.py
```

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.