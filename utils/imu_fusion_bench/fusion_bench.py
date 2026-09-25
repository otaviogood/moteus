"""READ-ONLY bench monitor for the IMU fusion firmware (no motion commands).

Usage: PYTHONPATH=lib/python python3 fusion_bench.py <id> [seconds] [hz]

Polls registers 0x06d-0x06f (int16, one short-opcode read) at the given
rate, decodes with the reference decoder, tracks the toggle bit and
sentinel, and prints the aux2 fusion telemetry before/after.
"""
import asyncio, math, sys, time
import moteus
sys.path.insert(0, 'docs')
from imu_orientation_quantization import decode_quat48, toggle_quat48

def fields(obj, names):
    return {n: getattr(obj, n) for n in names if hasattr(obj, n)}

FUSION_FIELDS = ['init_state', 'initialized', 'converged', 'timing_degraded', 'toggle',
                 'storage_owner', 'last_reinit_reason', 'freq_fine', 'odr_actual_hz',
                 'gyro_words', 'accel_words', 'gyro_gaps', 'gap_slots',
                 'mailbox_overflow', 'mailbox_max_depth', 'fifo_overruns',
                 'resyncs', 'reinits', 'i2c_errors', 'stall_passes', 'arrival_unknown',
                 'sentinel_replies', 'valid_replies', 'phase_unc_us',
                 'q', 'bias', 'omega']

def gravity_from_q(q):
    w, x, y, z = q
    return (2*(x*z - w*y), 2*(w*x + y*z), 1 - 2*(x*x + y*y))

async def main(dev_id, seconds, hz):
    c = moteus.Controller(id=dev_id)
    s = moteus.Stream(c)
    await asyncio.wait_for(s.flush_read(), 2.0)
    await asyncio.wait_for(s.command(b'tel stop'), 5.0)
    await asyncio.wait_for(s.flush_read(), 2.0)

    async def fusion_status():
        fu = await asyncio.wait_for(s.read_data('aux2_fusion'), 8.0)
        a2 = await asyncio.wait_for(s.read_data('aux2'), 8.0)
        return fields(fu, FUSION_FIELDS), a2.i2c.devices[0]

    ss = await asyncio.wait_for(s.read_data('servo_stats'), 8.0)
    print(f"servo mode={ss.mode} fault={ss.fault} isr_max_cycles={getattr(ss, 'isr_max_cycles', None)}")
    f0, d0 = await fusion_status()
    print('aux2 i2c dev0: active=%s error_count=%s' % (d0.active, d0.error_count))
    for k, v in f0.items():
        print(f'  {k} = {v}')

    # Register polling: 3 x int16 at 0x06d via the query protocol.
    qr = moteus.QueryResolution()
    qr._extra = {0x06d: moteus.INT16, 0x06e: moteus.INT16, 0x06f: moteus.INT16}
    c_q = moteus.Controller(id=dev_id, query_resolution=qr)
    n = 0; sentinels = 0; repeats = 0; last_bit = None; run = 0; max_run = 0
    t_end = time.monotonic() + seconds
    period = 1.0 / hz
    last_q = None; max_step_deg = 0.0
    grav = None
    while time.monotonic() < t_end:
        t0 = time.monotonic()
        r = await asyncio.wait_for(c_q.query(), 1.0)
        n += 1
        words = tuple(int(r.values[reg]) & 0xffff for reg in (0x06d, 0x06e, 0x06f))
        q = decode_quat48(words)
        if q is None:
            sentinels += 1; last_bit = None; run = 0
        else:
            bit = toggle_quat48(words)
            if last_bit is not None and bit == last_bit:
                repeats += 1; run += 1; max_run = max(max_run, run)
            else:
                run = 0
            last_bit = bit
            if last_q is not None:
                d = abs(sum(a*b for a, b in zip(q, last_q)))
                max_step_deg = max(max_step_deg, 2*math.degrees(math.acos(min(1.0, d))))
            last_q = q
            grav = gravity_from_q(q)
        dt = period - (time.monotonic() - t0)
        if dt > 0:
            await asyncio.sleep(dt)
    print(f'\npolled {n} replies at {hz} Hz over {seconds} s: sentinels={sentinels} '
          f'toggle_repeats={repeats} longest_repeat_run={max_run} max_step={max_step_deg:.3f} deg')
    if grav is not None:
        print('last gravity (body frame) = (%.4f, %.4f, %.4f)' % grav)
        print('last words = %s' % (' '.join('0x%04x' % w for w in words)))
    f1, d1 = await fusion_status()
    ss = await asyncio.wait_for(s.read_data('servo_stats'), 8.0)
    print(f"servo mode={ss.mode} fault={ss.fault} isr_max_cycles={getattr(ss, 'isr_max_cycles', None)}")
    print('deltas over the run:')
    for k in ('gyro_words', 'accel_words', 'gyro_gaps', 'gap_slots', 'mailbox_overflow',
              'fifo_overruns', 'resyncs', 'reinits', 'i2c_errors', 'stall_passes',
              'arrival_unknown', 'sentinel_replies', 'valid_replies'):
        if k in f0 and k in f1:
            print(f'  {k}: {f1[k] - f0[k]:+} (now {f1[k]})')
    for k in ('init_state', 'initialized', 'converged', 'odr_actual_hz', 'freq_fine', 'phase_unc_us',
              'mailbox_max_depth', 'toggle', 'q', 'bias', 'omega'):
        if k in f1:
            print(f'  {k} = {f1[k]}')

asyncio.run(main(int(sys.argv[1]) if len(sys.argv) > 1 else 2,
                 float(sys.argv[2]) if len(sys.argv) > 2 else 5.0,
                 float(sys.argv[3]) if len(sys.argv) > 3 else 120.0))
