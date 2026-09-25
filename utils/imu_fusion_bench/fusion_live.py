"""Live orientation readout for the hand-rotation axis check (docs §7.3).  READ-ONLY.

Usage: PYTHONPATH=lib/python python3 fusion_live.py <id> [seconds]
Prints, at 5 Hz: heading/pitch/roll (deg, ZYX from the board->world quaternion),
the world-up vector in the body frame, and the angular rate from telemetry.
Expected for a right-handed body frame: rotating the board about its +z
(counterclockwise seen from above) increases heading; tilting +x down makes
up_body.x positive... compare against the silkscreen axes of the LSM6DSV16X.
"""
import asyncio, math, sys, time
import moteus
sys.path.insert(0, 'docs')
from imu_orientation_quantization import decode_quat48

def zyx_deg(q):
    w, x, y, z = q
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    s = max(-1.0, min(1.0, 2 * (w * y - z * x)))
    pitch = math.asin(s)
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    return math.degrees(yaw), math.degrees(pitch), math.degrees(roll)

async def main(dev_id, seconds):
    qr = moteus.QueryResolution()
    qr._extra = {0x06d: moteus.INT16, 0x06e: moteus.INT16, 0x06f: moteus.INT16}
    cq = moteus.Controller(id=dev_id, query_resolution=qr)
    c = moteus.Controller(id=dev_id); s = moteus.Stream(c)
    await asyncio.wait_for(s.flush_read(), 2.0)
    await asyncio.wait_for(s.command(b'tel stop'), 5.0)
    await asyncio.wait_for(s.flush_read(), 2.0)
    t_end = time.monotonic() + seconds
    print('    t   heading   pitch    roll |  up_body x     y     z |  omega x     y     z (rad/s)')
    while time.monotonic() < t_end:
        r = await asyncio.wait_for(cq.query(), 1.0)
        words = tuple(int(r.values[reg]) & 0xffff for reg in (0x06d, 0x06e, 0x06f))
        q = decode_quat48(words)
        fu = await asyncio.wait_for(s.read_data('aux2_fusion'), 8.0)
        if q is None:
            print(f'{time.monotonic() % 1000:7.2f}  sentinel')
        else:
            w, x, y, z = q
            up = (2 * (x * z - w * y), 2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
            h, p, rl = zyx_deg(q)
            om = fu.omega
            print(f'{time.monotonic() % 1000:7.2f} {h:8.1f} {p:7.1f} {rl:7.1f} | {up[0]:9.3f} {up[1]:6.3f} {up[2]:6.3f} | {om[0]:8.2f} {om[1]:6.2f} {om[2]:6.2f}')
        await asyncio.sleep(0.2)

asyncio.run(main(int(sys.argv[1]), float(sys.argv[2]) if len(sys.argv) > 2 else 20.0))
