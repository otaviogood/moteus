#!/usr/bin/python3 -B

"""
This example reads the quaternion values from a LSM6DSV16X IMU connected
to the Aux2 port of a moteus controller with ID #1.
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
    parser = argparse.ArgumentParser(description='Query quaternion data from moteus controller')
    parser.add_argument('--target', type=int, default=1, help='ID of the target controller')
    parser.add_argument('--component', choices=['all', 'x', 'y', 'z'], default='all',
                      help='Which quaternion component to query (to diagnose I2C issues)')
    moteus.make_transport_args(parser)
    args = parser.parse_args()

    # Create a query resolution that specifies which quaternion registers to read
    qr = moteus.QueryResolution()
    qr._extra = {}

    # Configure which components to query based on command-line options
    if args.component == 'x':
        qr._extra[0x072] = INT16  # Register.AUX2_QUATERNIONX
    elif args.component == 'y':
        qr._extra[0x073] = INT16  # Register.AUX2_QUATERNIONY
    elif args.component == 'z':
        qr._extra[0x074] = INT16  # Register.AUX2_QUATERNIONZ
    else:  # Default to 'all'
        qr._extra = {
            0x072: INT16,  # Register.AUX2_QUATERNIONX
            0x073: INT16,  # Register.AUX2_QUATERNIONY
            0x074: INT16,  # Register.AUX2_QUATERNIONZ
        }
    # Also query the absolute position from the Aux1 SPI encoder
    qr._extra[0x006] = F32  # Register.ABS_POSITION

    # Create a controller with our specific query resolution and target ID
    transport = moteus.get_singleton_transport(args)
    controller = moteus.Controller(id=args.target, query_resolution=qr, transport=transport)

    print(f"Reading quaternion values from LSM6DSV16X IMU on Aux2 (controller ID: {args.target}). Press Ctrl+C to exit.")
    print()

    try:
        while True:
            # Query the controller for the custom registers
            result = await controller.query()

            # Extract the quaternion values, defaulting to 0 if not queried
            quat_x = result.values.get(0x072, 0) if args.component in ['all', 'x'] else 0
            quat_y = result.values.get(0x073, 0) if args.component in ['all', 'y'] else 0
            quat_z = result.values.get(0x074, 0) if args.component in ['all', 'z'] else 0

            # Extract the absolute position value
            abs_pos = result.values.get(0x006, 0.0)

            # Print raw values for debugging
            if args.component in ['all', 'x']:
                print(f"Raw X: {quat_x & 0xFFFF} (0x{quat_x & 0xFFFF:04x})")
            if args.component in ['all', 'y']:
                print(f"Raw Y: {quat_y & 0xFFFF} (0x{quat_y & 0xFFFF:04x})")
            if args.component in ['all', 'z']:
                print(f"Raw Z: {quat_z & 0xFFFF} (0x{quat_z & 0xFFFF:04x})")

            # Convert from int16 to float16 (IEEE 754 half-precision format)
            x = float16_to_float32(quat_x) if args.component in ['all', 'x'] else 0.0
            y = float16_to_float32(quat_y) if args.component in ['all', 'y'] else 0.0
            z = float16_to_float32(quat_z) if args.component in ['all', 'z'] else 0.0

            # Print the individual components
            if args.component in ['all', 'x']:
                print(f"Converted X: {x:.6f}")
            if args.component in ['all', 'y']:
                print(f"Converted Y: {y:.6f}")
            if args.component in ['all', 'z']:
                print(f"Converted Z: {z:.6f}")

            # Only do the quaternion calculations if we're querying all components
            if args.component == 'all':
                # Check if the quaternion is valid (w² + x² + y² + z² = 1)
                square_sum = x*x + y*y + z*z
                print(f"Sum of squares (x² + y² + z²): {square_sum:.6f}")

                # Make sure the value inside sqrt is not negative
                w_squared = 1.0 - square_sum
                if w_squared < 0:
                    print(f"Warning: Invalid quaternion (sum of squares > 1): {square_sum:.6f}")
                    w = 0.0
                else:
                    # Calculate w component: In a proper quaternion, x² + y² + z² + w² = 1
                    # So w = sqrt(1 - (x² + y² + z²))
                    w = math.sqrt(w_squared)

                # The LSM6DSV16X "Game Rotation Vector" outputs quaternion in the order (x,y,z)
                # with w calculated as above. For consistency with typical quaternion notation,
                # print in the order (w,x,y,z).
                print(f"Quaternion: [{w:.4f}, {x:.4f}, {y:.4f}, {z:.4f}]")
                print("")

            # Print the absolute position
            print(f"Absolute Position (Aux1 SPI): {abs_pos:.4f} revolutions")
            print("")

            # Wait before the next reading
            await asyncio.sleep(0.1)

    except KeyboardInterrupt:
        print("\nExiting...")

if __name__ == '__main__':
    asyncio.run(main()) 