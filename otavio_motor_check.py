#!/usr/bin/python3 -B

"""
This program performs health checks on a moteus controller:
- Reads the Aux2 IMU orientation and verifies it is valid (not the
  "no data" sentinel)
- Reads the encoder value and verifies it's non-zero
- Checks the FET temperature, ensures it's between 20.0 and 40.0
- Checks the motor temperature, ensures it's between 20.0 and 40.0
"""

import asyncio
import math
import moteus

# Import the resolution types from the multiplex module
from moteus.multiplex import INT16, F32

def decode_quat48(w0, w1, w2):
    """Registers 0x06d-0x06f -> (w, x, y, z), or None for the "no data"
    sentinel (docs/protocol/registers.md)."""
    v = ((w0 & 0xffff) | ((w1 & 0xffff) << 16) | ((w2 & 0xffff) << 32)) & ((1 << 47) - 1)
    if v == 0:
        return None
    omitted = (v >> 45) & 0x3
    others = [(((v >> (15 * j)) & 0x7fff) / 32767 * 2.0 - 1.0) / math.sqrt(2.0)
              for j in range(3)]
    largest = math.sqrt(max(0.0, 1.0 - sum(c * c for c in others)))
    return others[:omitted] + [largest] + others[omitted:]

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
        moteus.Register.AUX2_QUATERNIONX: INT16,
        moteus.Register.AUX2_QUATERNIONY: INT16,
        moteus.Register.AUX2_QUATERNIONZ: INT16,

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

    # Query the controller for the registers.  Right after a boot the IMU
    # sends the "no data" sentinel for up to ~1 s, so retry briefly.
    for _ in range(20):
        result = await controller.query()
        q = decode_quat48(
            result.values.get(moteus.Register.AUX2_QUATERNIONX, 0),
            result.values.get(moteus.Register.AUX2_QUATERNIONY, 0),
            result.values.get(moteus.Register.AUX2_QUATERNIONZ, 0))
        if q is not None:
            break
        await asyncio.sleep(0.1)

    all_passed = True

    if q is None:
        print("ERROR: IMU orientation not available (no data for 2 s)!")
        all_passed = False
    else:
        print("Orientation wxyz: [{:+.4f}, {:+.4f}, {:+.4f}, {:+.4f}]".format(*q))
        print("PASS: IMU orientation is valid.")

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
            if 18.0 <= fet_temp <= 38.0:
                print("PASS: FET temperature is within normal range (18-38°C).")
            else:
                print(f"ERROR: FET temperature {fet_temp:.1f}°C is outside normal range (18-38°C)!")
                all_passed = False

        # Check motor temperature
        motor_temp = result.values.get(0x00a, float('nan'))

        if math.isnan(motor_temp):
            print("ERROR: Motor temperature reading not available! Make sure thermistor is connected and configured.")
            all_passed = False
        else:
            print(f"Motor temperature: {motor_temp:.1f}°C")
            if 18.0 <= motor_temp <= 38.0:
                print("PASS: Motor temperature is within normal range (18-38°C).")
            else:
                print(f"ERROR: Motor temperature {motor_temp:.1f}°C is outside normal range (18-38°C)!")
                all_passed = False

    # Overall status
    print("\nOverall health check:")
    if all_passed:
        print("SUCCESS: All health checks passed!")
    else:
        print("FAILURE: One or more health checks failed!")

if __name__ == '__main__':
    asyncio.run(main())