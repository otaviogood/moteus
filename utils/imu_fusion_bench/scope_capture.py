"""Logic-analyzer capture sequence for the MOTEUS_SCOPE_MARKERS build
(fw/scope_markers.h) on the BENCH motor.  Zero-torque only.

Usage: PYTHONPATH=lib/python python3 utils/imu_fusion_bench/scope_capture.py <id> [--conf-write]

Start the capture first; the script waits 3 s, then runs phases separated
by 1 s of idle, printing each phase's start time (relative to the "t=0"
line) and the servo's isr_max_cycles after it.  DBG2 (`d mark`) is high
for exactly the span of each phase (for the position phases, while the
mode is active) and low for the 1 s between phases:

  A  idle            servo stopped, IMU fusion streaming (baseline)
  B  query load      quaternion reads as fast as the adapter allows
  C  position hold   `d pos nan 0 0 t8` (max torque 0) plus the query load
  D  moving setpoint `d pos <here + 3 rev> 0 0 v1 a2 t8` (max torque 0):
                     the trajectory planner accelerating and cruising, the
                     heaviest control path, plus the query load
  E  conf write      only with --conf-write: a flash save while stopped
                     (the ISR stalls on flash-resident code for ~86 us)

Max torque is 0 in every phase, so the motor never produces torque; the
position phases report while still in position mode, then `d stop`
(the 8 s watchdog never expires).  Always sends `d stop` at the end.
"""
import asyncio
import sys
import time

import moteus

PHASE_S = 3.0
GAP_S = 1.0
CYCLES_PER_US = 170.0


async def main(dev_id, conf_write):
    c = moteus.Controller(id=dev_id)
    s = moteus.Stream(c)
    await asyncio.wait_for(s.flush_read(), 2.0)
    await asyncio.wait_for(s.command(b'tel stop'), 5.0)
    await asyncio.wait_for(s.flush_read(), 2.0)

    qr = moteus.QueryResolution()
    qr._extra = {0x06d: moteus.INT16, 0x06e: moteus.INT16, 0x06f: moteus.INT16}
    c_q = moteus.Controller(id=dev_id, query_resolution=qr)

    async def isr_max():
        ss = await asyncio.wait_for(s.read_data('servo_stats'), 8.0)
        return ss.mode, ss.fault, ss.isr_max_cycles

    async def queries(seconds):
        n = 0
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            await asyncio.wait_for(c_q.query(), 1.0)
            n += 1
        return n / seconds

    async def mark(on):
        await asyncio.wait_for(s.command(b'd mark 1' if on else b'd mark 0'), 5.0)

    await mark(False)
    print('start the capture now; t=0 in 3 s', flush=True)
    await asyncio.sleep(3.0)
    t0 = time.monotonic()
    print('t=0', flush=True)

    def banner(name):
        print(f'{time.monotonic() - t0:6.2f} s  phase {name}', flush=True)

    async def report(name):
        # isr_max_cycles is the previous second's maximum.
        await asyncio.sleep(1.1)
        mode, fault, cyc = await isr_max()
        print(f'          {name}: isr_max {cyc} cycles ({cyc / CYCLES_PER_US:.1f} us '
              f'of 33.3), mode {mode}, fault {fault}', flush=True)

    try:
        banner('A idle')
        await mark(True)
        await asyncio.sleep(PHASE_S)
        await report('A')
        await mark(False)
        await asyncio.sleep(GAP_S)

        banner('B query load')
        await mark(True)
        rate = await queries(PHASE_S)
        print(f'          B: {rate:.0f} queries/s', flush=True)
        await report('B')
        await mark(False)
        await asyncio.sleep(GAP_S)

        banner('C position hold (max torque 0) + query load')
        await asyncio.wait_for(s.command(b'd pos nan 0 0 t8'), 5.0)
        await mark(True)
        rate = await queries(PHASE_S)
        print(f'          C: {rate:.0f} queries/s', flush=True)
        await report('C')
        await mark(False)
        await asyncio.wait_for(s.command(b'd stop'), 5.0)
        await asyncio.sleep(GAP_S)

        banner('D moving setpoint (max torque 0) + query load')
        here = (await asyncio.wait_for(c.query(), 1.0)).values[moteus.Register.POSITION]
        target = here + 3.0
        await asyncio.wait_for(
            s.command(f'd pos {target:.4f} 0 0 v1 a2 t8'.encode()), 5.0)
        await mark(True)
        rate = await queries(PHASE_S)
        print(f'          D: {rate:.0f} queries/s (setpoint {here:.3f} -> {target:.3f} rev)',
              flush=True)
        await report('D')
        await mark(False)
        await asyncio.wait_for(s.command(b'd stop'), 5.0)
        await asyncio.sleep(GAP_S)

        if conf_write:
            banner('E conf write')
            await mark(True)
            await asyncio.wait_for(s.command(b'conf write'), 5.0)
            await mark(False)
            await report('E')
            await asyncio.sleep(GAP_S)
    finally:
        await asyncio.wait_for(s.command(b'd stop'), 5.0)
        await mark(False)

    fu = await asyncio.wait_for(s.read_data('aux2_fusion'), 8.0)
    print(f'{time.monotonic() - t0:6.2f} s  done; fusion i2c_errors {fu.i2c_errors} '
          f'resyncs {fu.resyncs} gaps {fu.gyro_gaps} mailbox_max_depth {fu.mailbox_max_depth}',
          flush=True)


if __name__ == '__main__':
    asyncio.run(main(int(sys.argv[1]), '--conf-write' in sys.argv[2:]))
