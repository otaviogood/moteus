"""Trace tilt, bias and the stationary flag at rest (READ-ONLY): prints 1 s summaries.
Usage: PYTHONPATH=lib/python python3 static_trace.py <id> [seconds]"""
import asyncio, math, sys, time
import moteus
def up_body(q):
    w, x, y, z = q; return (2*(x*z - w*y), 2*(w*x + y*z), w*w - x*x - y*y + z*z)
async def main(dev_id, seconds):
    c = moteus.Controller(id=dev_id); s = moteus.Stream(c)
    await asyncio.wait_for(s.flush_read(), 2.0)
    await asyncio.wait_for(s.command(b'tel stop'), 5.0)
    await asyncio.wait_for(s.flush_read(), 2.0)
    t0 = time.monotonic(); rows = []
    print('   t   tilt_err  bias_x   bias_y   bias_z (dps)  stat  |omega| dps  words')
    ref = None
    while time.monotonic() - t0 < seconds:
        f = await asyncio.wait_for(s.read_data('aux2_fusion'), 8.0)
        up = up_body(tuple(f.q))
        if ref is None: ref = up
        d = sum(a*b for a, b in zip(up, ref))
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, d))))
        om = math.degrees(math.sqrt(sum(v*v for v in f.omega)))
        rows.append((time.monotonic() - t0, tilt, [math.degrees(b) for b in f.bias], f.stationary, om, f.stationary_words))
        await asyncio.sleep(0.04)
    last = -1
    for r in rows:
        if int(r[0]) != last:
            last = int(r[0])
            print(f'{r[0]:5.1f}  {r[1]:7.3f}  {r[2][0]:7.4f} {r[2][1]:8.4f} {r[2][2]:8.4f}   {int(r[3])}   {r[4]:7.3f}    {r[5]}')
    tilts = [r[1] for r in rows]
    print(f'tilt vs first sample: max {max(tilts):.3f} deg; samples {len(rows)}')
asyncio.run(main(int(sys.argv[1]), float(sys.argv[2]) if len(sys.argv) > 2 else 40.0))
