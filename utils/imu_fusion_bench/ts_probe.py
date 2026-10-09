"""Transfer-delay probe readout (READ-ONLY, no motion commands).

The fusion firmware measures, for one gyro sample every 32, the time from
the sample's FIFO write to the arrival stamp the fusion uses for it
(fw/imu_fusion.h EvaluateProbe: the chip's own timestamp word gives the
FIFO-write time of its slot in the chip's clock, a live read of the
chip's TIMESTAMP0-3 right after it ties that clock to TIM3, and the
measured sample is two slots later, a slot holding nothing but a gyro
word with the bus quiet again: the kind the fusion's arrival-floor
tracker follows).  This prints
those statistics from the aux2_fusion telemetry.

Usage: PYTHONPATH=lib/python python3 utils/imu_fusion_bench/ts_probe.py <id> [seconds]

Once a second: pairs so far, the 16-pair running means of the transfer
(arrival - FIFO write) and of the fusion clock's offset (history stamp -
FIFO write), and the transfer's min / max since the board's fusion
started (min only); at the end the mean and spread of the running means
over the run.  The transfer is the I2C path (status poll, word reads), so it
should be the same still or moving; rejects > 0 means corrupted live
reads (a counter byte rolling over mid-read) or a layout problem.
Expected: a few hundred us, a few tens of us of spread; the clock offset
near the transfer's minimum (the arrival-floor tracker).  Systematic
uncertainty ~ +-50 us (where in its read the chip latches the counter).
Needs a board flashed with a MOTEUS_TS_PROBE build (fw/imu_fusion.h).
"""
import asyncio
import statistics
import sys
import time

import moteus

FIELDS = ('ts_pairs', 'ts_rejects', 'ts_delay_min_us', 'ts_delay_mean_us', 'ts_model_mean_us')


async def main(dev_id, seconds):
    c = moteus.Controller(id=dev_id)
    s = moteus.Stream(c)
    await asyncio.wait_for(s.flush_read(), 2.0)
    await asyncio.wait_for(s.command(b'tel stop'), 5.0)
    await asyncio.wait_for(s.flush_read(), 2.0)

    async def status():
        fu = await asyncio.wait_for(s.read_data('aux2_fusion'), 8.0)
        missing = [f for f in FIELDS if not hasattr(fu, f)]
        if missing:
            sys.exit(f'aux2_fusion has no {missing}: the board is not running a MOTEUS_TS_PROBE build')
        return fu

    first = await status()
    print(f'board {dev_id}: converged {first.converged}, i2c_errors {first.i2c_errors}, '
          f'resyncs {first.resyncs}, odr {first.odr_actual_hz:.1f} Hz, freq_fine {first.freq_fine}')
    print(f'probe so far: {first.ts_pairs} pairs, {first.ts_rejects} rejects, '
          f'transfer min {first.ts_delay_min_us} us')
    print('    t   pairs  +/s  rejects  xfer_us  clock_us   min_us')
    means, models = [], []
    t0 = time.monotonic()
    last_pairs = first.ts_pairs
    last_t = t0
    while time.monotonic() - t0 < seconds:
        await asyncio.sleep(0.5)
        fu = await status()
        now = time.monotonic()
        rate = (fu.ts_pairs - last_pairs) / max(1e-6, now - last_t)
        last_pairs, last_t = fu.ts_pairs, now
        means.append(float(fu.ts_delay_mean_us))
        models.append(float(fu.ts_model_mean_us))
        if int(now - t0) != int(now - t0 - 0.5):
            print(f'{now - t0:5.1f}  {fu.ts_pairs:6d} {rate:5.1f} {fu.ts_rejects:8d} '
                  f'{fu.ts_delay_mean_us:8.1f} {fu.ts_model_mean_us:9.1f} {fu.ts_delay_min_us:8d}',
                  flush=True)
    fu = await status()
    print(f'\n{fu.ts_pairs - first.ts_pairs} pairs in {seconds:.0f} s '
          f'({(fu.ts_pairs - first.ts_pairs) / seconds:.1f}/s; the chip writes a timestamp word '
          f'every 32 samples, ~30/s), {fu.ts_rejects - first.ts_rejects} new rejects')
    if len(means) > 2:
        print(f'transfer (arrival - FIFO write), running mean over the run: {statistics.mean(means):.1f} us '
              f'(sd {statistics.pstdev(means):.1f}, range {min(means):.1f}..{max(means):.1f})')
        print(f'fusion clock offset (history stamp - FIFO write), running mean over the run: '
              f'{statistics.mean(models):.1f} us (sd {statistics.pstdev(models):.1f}, '
              f'range {min(models):.1f}..{max(models):.1f})')
    print(f'transfer since the fusion started ({fu.ts_pairs} pairs): min {fu.ts_delay_min_us} us')
    print(f'i2c_errors {fu.i2c_errors - first.i2c_errors:+d}, resyncs {fu.resyncs - first.resyncs:+d} during the run')


if __name__ == '__main__':
    asyncio.run(main(int(sys.argv[1]), float(sys.argv[2]) if len(sys.argv) > 2 else 30.0))
