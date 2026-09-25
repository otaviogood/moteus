"""Cold-start timing of the IMU fusion after a board reset (no motion).

Usage: PYTHONPATH=lib/python python3 utils/imu_fusion_bench/cold_start.py <id> [hz] [flips]

Sends `d reset` on the diagnostic channel (an MCU reset; the servo comes
back stopped), then polls registers 0x000 (mode) and 0x06d-0x06f at the
given rate and reports, relative to the reset:

  * the first reply (board back on CAN),
  * the first valid quat48 reply (not the sentinel),
  * the first reply that completes `flips` consecutive valid replies with
    an alternating toggle bit (the host's startup check).

The mode register is printed on every change; anything but 0 is flagged.
"""
import asyncio, sys, time
import moteus
sys.path.insert(0, 'docs')
from imu_orientation_quantization import decode_quat48, toggle_quat48


async def main(dev_id, hz, flips):
    c = moteus.Controller(id=dev_id)
    s = moteus.Stream(c)
    await asyncio.wait_for(s.flush_read(), 2.0)

    qr = moteus.QueryResolution()
    qr.mode = moteus.INT8
    qr._extra = {0x06d: moteus.INT16, 0x06e: moteus.INT16, 0x06f: moteus.INT16}
    c_q = moteus.Controller(id=dev_id, query_resolution=qr)

    s.write(b'd reset\n')
    await s.drain()
    t0 = time.monotonic()

    period = 1.0 / hz
    first_reply = first_valid = check_done = None
    last_bit = None
    run = 0
    last_mode = None
    sentinels = 0
    while time.monotonic() - t0 < 6.0:
        t_req = time.monotonic()
        try:
            r = await asyncio.wait_for(c_q.query(), 0.02)
        except (asyncio.TimeoutError, RuntimeError):
            r = None
        if r is not None and r.values:
            t = t_req - t0
            if first_reply is None:
                first_reply = t
            mode = int(r.values.get(moteus.Register.MODE, -1))
            if mode != last_mode:
                flag = '' if mode == 0 else '  <-- NONZERO MODE'
                print(f'  {t * 1e3:7.1f} ms  mode {mode}{flag}')
                last_mode = mode
            words = tuple(int(r.values[reg]) & 0xffff
                          for reg in (0x06d, 0x06e, 0x06f))
            if decode_quat48(words) is None:
                sentinels += 1
                run = 0
                last_bit = None
            else:
                if first_valid is None:
                    first_valid = t
                bit = toggle_quat48(words)
                run = run + 1 if (last_bit is None or bit != last_bit) else 1
                last_bit = bit
                if check_done is None and run >= flips:
                    check_done = t
            if check_done is not None and t > check_done + 0.2:
                break
        dt = period - (time.monotonic() - t_req)
        if dt > 0:
            await asyncio.sleep(dt)

    fmt = lambda v: 'never' if v is None else f'{v * 1e3:.0f} ms'
    print(f'reset -> first reply {fmt(first_reply)}, first valid quaternion '
          f'{fmt(first_valid)}, {flips} alternating toggles {fmt(check_done)} '
          f'({sentinels} sentinel replies, polled at {hz} Hz)')


if __name__ == '__main__':
    dev = int(sys.argv[1])
    hz = float(sys.argv[2]) if len(sys.argv) > 2 else 200.0
    flips = int(sys.argv[3]) if len(sys.argv) > 3 else 4
    asyncio.run(main(dev, hz, flips))
