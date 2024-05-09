import sys
import struct

def float_to_hex(f):
    return hex(struct.unpack('<I', struct.pack('<f', f))[0])

def double_to_hex(d):
    return hex(struct.unpack('<Q', struct.pack('<d', d))[0])

def float_from_hex(hex_str):
    return struct.unpack('<f', struct.pack('<I', int(hex_str, 16)))[0]

def double_from_hex(hex_str):
    return struct.unpack('<d', struct.pack('<Q', int(hex_str, 16)))[0]

if len(sys.argv) != 2:
    print("Usage: python floattest.py num")
    sys.exit(1)

num = float(sys.argv[1])

float_hex = float_to_hex(num)
double_hex = double_to_hex(num)

float_value = float_from_hex(float_hex)
double_value = double_from_hex(double_hex)

print(f"Original: {num}")
print(f"Float: {float_value} {float_hex}")
print(f"Double: {double_value} {double_hex}")
