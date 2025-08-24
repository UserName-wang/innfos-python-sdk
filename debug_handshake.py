from innfos_python_sdk.protocol import GluonProtocol
from innfos_python_sdk.constants import Command

# Create protocol instance
protocol = GluonProtocol()

# Build handshake packet
handshake_packet = protocol._build_packet(0x00, Command.HANDSHAKE, b'')

print('New handshake packet:', handshake_packet.hex().upper())
print('User says correct:   ', 'EE00440000ED')

# Check if they match
if handshake_packet.hex().upper() == 'EE00440000ED':
    print('✓ Packets match!')
else:
    print('✗ Packets do not match')
    
# Also test a different command to make sure we didn't break CRC calculation for other commands
query_packet = protocol._build_packet(0x00, Command.QUERY_ACTUATORS, b'')
print('Query actuators packet:', query_packet.hex().upper())