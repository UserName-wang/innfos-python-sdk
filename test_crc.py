#!/usr/bin/env python3

import sys
sys.path.append('/home/panda/study/ros/ros_gluon/innfos-python-sdk')

from innfos_python_sdk.protocol import CRC16

# Test data from the response: EE0202000402BC3A2B92C7ED
# CRC should be calculated on: [02 02 00 04 02 BC 3A 2B]
crc_data = bytearray([0x02, 0x02, 0x00, 0x04, 0x02, 0xBC, 0x3A, 0x2B])
calculated_crc = CRC16.calculate(crc_data)
received_crc = 0x92C7

print(f"CRC data: {[hex(b) for b in crc_data]}")
print(f"Calculated CRC: {hex(calculated_crc)}")
print(f"Received CRC: {hex(received_crc)}")
print(f"CRC match: {calculated_crc == received_crc}")

# Test another response: EE0102000602BCFD280000E186ED
# CRC should be calculated on: [01 02 00 06 02 BC FD 28]
crc_data2 = bytearray([0x01, 0x02, 0x00, 0x06, 0x02, 0xBC, 0xFD, 0x28])
calculated_crc2 = CRC16.calculate(crc_data2)
received_crc2 = 0xE186

print(f"\nCRC data 2: {[hex(b) for b in crc_data2]}")
print(f"Calculated CRC 2: {hex(calculated_crc2)}")
print(f"Received CRC 2: {hex(received_crc2)}")
print(f"CRC match 2: {calculated_crc2 == received_crc2}")

# Test with the example from specification: EE 06 02 00 04 01 64 5A DF 3B 3F ED
# CRC should be calculated on: [06 02 00 04 01 64 5A DF]
crc_data3 = bytearray([0x06, 0x02, 0x00, 0x04, 0x01, 0x64, 0x5A, 0xDF])
calculated_crc3 = CRC16.calculate(crc_data3)
received_crc3 = 0x3B3F

print(f"\nCRC data 3 (from spec): {[hex(b) for b in crc_data3]}")
print(f"Calculated CRC 3: {hex(calculated_crc3)}")
print(f"Received CRC 3: {hex(received_crc3)}")
print(f"CRC match 3: {calculated_crc3 == received_crc3}")

# Let's try a different approach - maybe the CRC algorithm is different
# Let's check if it's calculated in a different order or with different initial values