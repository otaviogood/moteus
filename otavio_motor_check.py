#!/usr/bin/python3 -B

"""
This program performs health checks on a moteus controller:
- Reads quaternion values and verifies they are non-zero
- Reads the encoder value and verifies it's non-zero
- Checks the FET temperature, ensures it's between 20.0 and 40.0
- Checks the motor temperature, ensures it's between 20.0 and 40.0
"""

import asyncio
import math
import moteus
import struct

# Import the resolution types from the multiplex module
from moteus.multiplex import INT16, F32

# Helper function to convert the 16-bit values into float16 format
def float16_to_float32(value):
    # Extract sign, exponent, and mantissa from the 16-bit value
    sign = (value & 0x8000) >> 15
    exponent = (value & 0x7C00) >> 10
    mantissa = value & 0x03FF

    # Handle special cases
    if exponent == 0:
        if mantissa == 0:
            return -0.0 if sign else 0.0
        else:
            # Denormalized number
            return (-1.0 if sign else 1.0) * (mantissa / 1024.0) * (2.0 ** -14)
    elif exponent == 31:
        if mantissa == 0:
            return float('-inf') if sign else float('inf')
        else:
            return float('nan')

    # Normalized number
    result = (-1.0 if sign else 1.0) * (1.0 + mantissa / 1024.0) * (2.0 ** (exponent - 15))
    return result

async def main():
    # Parse command line arguments to get target
    import argparse
    parser = argparse.ArgumentParser(description='Health check for moteus controller')
    parser.add_argument('--target', type=int, default=1, help='ID of the target controller')
    parser.add_argument('--nomotor', action='store_true', help='Skip motor related checks for sensor boards with no motor driver')
    moteus.make_transport_args(parser)
    args = parser.parse_args()

    # Create a query resolution that specifies which registers to read
    qr = moteus.QueryResolution()
    qr._extra = {
        # Quaternion values
        0x072: INT16,  # Register.AUX2_QUATERNIONX
        0x073: INT16,  # Register.AUX2_QUATERNIONY
        0x074: INT16,  # Register.AUX2_QUATERNIONZ

        # Encoder position (position source 0)
        0x050: F32,    # Register.ENCODER_0_POSITION

        # Temperatures
        0x00e: F32,    # Register.TEMPERATURE (FET/board temperature)
        0x00a: F32,    # Register.MOTOR_TEMPERATURE
    }

    # Create a controller with our specific query resolution and target ID
    transport = moteus.get_singleton_transport(args)
    controller = moteus.Controller(id=args.target, query_resolution=qr, transport=transport)

    print(f"Performing health check on moteus controller (ID: {args.target})...")

    # Query the controller for the registers
    result = await controller.query()

    all_passed = True

    # Check quaternion values
    quat_x = result.values.get(0x072, 0)
    quat_y = result.values.get(0x073, 0)
    quat_z = result.values.get(0x074, 0)

    # Convert from int16 to float16
    x = float16_to_float32(quat_x)
    y = float16_to_float32(quat_y)
    z = float16_to_float32(quat_z)

    # Calculate the magnitude of the quaternion components
    quat_magnitude = math.sqrt(x*x + y*y + z*z)

    print(f"Quaternion X: {x:.6f}")
    print(f"Quaternion Y: {y:.6f}")
    print(f"Quaternion Z: {z:.6f}")
    print(f"Quaternion magnitude: {quat_magnitude:.6f}")

    if quat_magnitude == 0:
        print("ERROR: Quaternion values are all zero!")
        all_passed = False
    else:
        print("PASS: Quaternion values are non-zero.")

    # Check encoder value
    encoder_pos = result.values.get(0x050, float('nan'))
    print(f"Encoder position: {encoder_pos:.6f} revolutions")

    if math.isnan(encoder_pos):
        print("ERROR: Encoder position reading not available!")
        all_passed = False
    elif encoder_pos == 0.0:
        print("ERROR: Encoder position is zero! (maybe just unlucky?)")
        all_passed = False
    else:
        print("PASS: Encoder position is non-zero.")

    if not args.nomotor:
        # Check FET temperature
        fet_temp = result.values.get(0x00e, float('nan'))

        if math.isnan(fet_temp):
            print("ERROR: FET temperature reading not available!")
            all_passed = False
        else:
            print(f"FET temperature: {fet_temp:.1f}°C")
            if 18.0 <= fet_temp <= 34.0:
                print("PASS: FET temperature is within normal range (18-34°C).")
            else:
                print(f"ERROR: FET temperature {fet_temp:.1f}°C is outside normal range (18-34°C)!")
                all_passed = False

        # Check motor temperature
        motor_temp = result.values.get(0x00a, float('nan'))

        if math.isnan(motor_temp):
            print("ERROR: Motor temperature reading not available! Make sure thermistor is connected and configured.")
            all_passed = False
        else:
            print(f"Motor temperature: {motor_temp:.1f}°C")
            if 18.0 <= motor_temp <= 34.0:
                print("PASS: Motor temperature is within normal range (18-34°C).")
            else:
                print(f"ERROR: Motor temperature {motor_temp:.1f}°C is outside normal range (18-34°C)!")
                all_passed = False

    # Overall status
    print("\nOverall health check:")
    if all_passed:
        print("SUCCESS: All health checks passed!")
    else:
        print("FAILURE: One or more health checks failed!")

if __name__ == '__main__':
    asyncio.run(main()) 