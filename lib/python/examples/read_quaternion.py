#!/usr/bin/python3 -B

"""
This example reads the orientation of a LSM6DSV16X IMU connected to the
Aux2 port of a moteus controller (aux2.i2c.devices.0.type 3).  Registers
0x06d-0x06f carry a 48-bit "smallest three" quaternion; see
docs/protocol/registers.md.
"""

import argparse
import asyncio
import math

import moteus
from moteus.multiplex import INT16

QUAT48_LEVELS = 32767


def decode_quat48(w0, w1, w2):
    """Three register words -> (w, x, y, z), or None for the "no data"
    sentinel.  Also returns the freshness toggle (bit 47)."""
    v = (w0 & 0xffff) | ((w1 & 0xffff) << 16) | ((w2 & 0xffff) << 32)
    toggle = (v >> 47) & 1
    v &= (1 << 47) - 1
    if v == 0:
        return None, toggle
    omitted = (v >> 45) & 0x3
    others = [((v >> (15 * j)) & 0x7fff) / QUAT48_LEVELS * 2.0 - 1.0
              for j in range(3)]
    others = [c / math.sqrt(2.0) for c in others]
    largest = math.sqrt(max(0.0, 1.0 - sum(c * c for c in others)))
    q = others[:omitted] + [largest] + others[omitted:]
    return q, toggle


async def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--target', type=int, default=1,
                        help='ID of the target controller')
    moteus.make_transport_args(parser)
    args = parser.parse_args()

    qr = moteus.QueryResolution()
    qr._extra = {
        moteus.Register.AUX2_QUATERNIONX: INT16,
        moteus.Register.AUX2_QUATERNIONY: INT16,
        moteus.Register.AUX2_QUATERNIONZ: INT16,
    }
    transport = moteus.get_singleton_transport(args)
    controller = moteus.Controller(
        id=args.target, query_resolution=qr, transport=transport)

    print(f"Reading the Aux2 IMU orientation of controller {args.target}. "
          "Press Ctrl+C to exit.")
    while True:
        result = await controller.query()
        q, toggle = decode_quat48(
            result.values[moteus.Register.AUX2_QUATERNIONX],
            result.values[moteus.Register.AUX2_QUATERNIONY],
            result.values[moteus.Register.AUX2_QUATERNIONZ])
        if q is None:
            print("no data (IMU not configured or filter warming up)")
        else:
            w, x, y, z = q
            print(f"wxyz = [{w:+.4f}, {x:+.4f}, {y:+.4f}, {z:+.4f}]  "
                  f"toggle {toggle}")
        await asyncio.sleep(0.1)


if __name__ == '__main__':
    asyncio.run(main())
