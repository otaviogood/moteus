"""Logic-analyzer capture with robot-like CAN traffic, for the
MOTEUS_SCOPE_MARKERS build on the BENCH motor.  Phase 4 SPINS THE MOTOR,
gently: the rotor must be free.

Usage: PYTHONPATH=lib/python python3 utils/imu_fusion_bench/scope_can_capture.py <id>

Probes: CAN TX and RX (logic side of the transceiver), DBG1 (control
interrupt) and DBG2, which this script switches to the CAN frame marker
(`d mark can`: high from the start of a frame's processing until its
command reaches the servo, or the end of the frame if it carries none,
notched low while the frame's IMU fusion work runs).

Phases (3 s unless noted), separated by 1 s with no bus traffic at all:

  1  robot pattern   120 Hz ticks, each: a position command to the board
                     (max torque 0, no reply), then the robot's broadcast
                     telemetry request (0x7F, 20 bytes, reply)
  2  + bus fill      the same, plus dummy commands to absent boards before
                     it and dummy 48-byte "replies" from absent boards after
                     it (as many as the adapter can send, ~770 frames/s)
  3  request stress  back-to-back telemetry queries to the board
  4  spin            the robot pattern, commanding 0.5 rev/s (accel 1
                     rev/s^2, max torque 0.2 N m, watchdog 0.1 s) for 4 s,
                     then 0 rev/s for 1 s: real phase current, the
                     heaviest control path
  5  enable cycles   5 x (zero-torque command 0.3 s, stop 0.2 s): the gate
                     driver's enable, config write and calibration each time

Phases 1-3 and 5 command zero torque.  Each phase ends with a stop
command and a report: isr_max (its last full second), the mode after the
stop, and the gate driver's status reads and register readback (fsr1/fsr2
must stay 0 and fault_config 0; a corrupted SPI read would show up as
stray bits).  The dummy frames are addressed so that the board's hardware filter drops
them; they only occupy the bus.
"""
import asyncio
import math
import sys
import time

import moteus
from moteus.transport_device import Frame

TICK_S = 1.0 / 120.0
SPIN_REV_S = 0.5
SPIN_ACCEL = 1.0         # rev/s^2
SPIN_TORQUE_NM = 0.2
SPIN_S = 4.0
SPIN_DOWN_S = 1.0
PHASE_S = 3.0
GAP_S = 1.0
CYCLES_PER_US = 170.0
BROADCAST_ID = 0x7F
# orin/can_backend.request_joint_telemetry: the robot's per-tick request.
TELEMETRY_REQUEST = bytes([
    0x15, 0x50, 0x17, 0x6D, 0x1D, 0x01, 0x1D, 0x02, 0x16, 0x0D,
    0x11, 0x0A, 0x15, 0x07, 0x15, 0x04, 0x11, 0x00, 0x11, 0x0F])
NOP_48 = bytes([0x50] * 48)


def frame(arbitration_id, data):
    f = Frame()
    f.arbitration_id = arbitration_id
    f.data = data
    f.is_extended_id = True
    f.is_fd = True
    f.bitrate_switch = True
    return f


async def main(dev_id):
    c = moteus.Controller(id=dev_id)
    s = moteus.Stream(c)
    dev = moteus.get_singleton_transport()._devices[0]
    await asyncio.wait_for(s.flush_read(), 2.0)
    await asyncio.wait_for(s.command(b'tel stop'), 5.0)
    await asyncio.wait_for(s.flush_read(), 2.0)
    await asyncio.wait_for(s.command(b'd mark can'), 5.0)

    position = c.make_position(position=math.nan, velocity=0.0,
                               maximum_torque=0.0, watchdog_timeout=0.1).data
    spin = c.make_position(position=math.nan, velocity=SPIN_REV_S,
                           accel_limit=SPIN_ACCEL, maximum_torque=SPIN_TORQUE_NM,
                           watchdog_timeout=0.1).data
    spin_down = c.make_position(position=math.nan, velocity=0.0,
                                accel_limit=SPIN_ACCEL, maximum_torque=SPIN_TORQUE_NM,
                                watchdog_timeout=0.1).data
    stop = c.make_stop().data
    command = frame(dev_id, position)             # source 0, no reply
    request = frame(0x8000 | BROADCAST_ID, TELEMETRY_REQUEST)

    drv_last = None

    async def report(name):
        nonlocal drv_last
        # isr_max is latched once a second, so read right after a phase
        # it covers the phase's last full second.
        ss = await asyncio.wait_for(s.read_data('servo_stats'), 8.0)
        drv = await asyncio.wait_for(s.read_data('drv8323'), 8.0)
        print(f'          {name}: isr_max {ss.isr_max_cycles} cycles '
              f'({ss.isr_max_cycles / CYCLES_PER_US:.1f} us of 33.3), '
              f'mode {ss.mode}, fault {ss.fault}', flush=True)
        reads = drv.status_count - drv_last.status_count if drv_last else 0
        writes = drv.config_count - drv_last.config_count if drv_last else 0
        ok = drv.fsr1 == 0 and drv.fsr2 == 0 and drv.fault_config == 0
        print(f'          gate driver: {reads} status reads, {writes} config writes, '
              f'fsr1 {drv.fsr1:#05x} fsr2 {drv.fsr2:#05x} fault_config {drv.fault_config}'
              f'{"" if ok else "  <-- CHECK"}', flush=True)
        drv_last = drv

    async def ticks(fill, seconds=PHASE_S, command_at=lambda t: command):
        replies = 0

        async def count():
            nonlocal replies
            while True:
                f = await dev.receive_frame()
                if f.arbitration_id & 0x7fff == (dev_id << 8):
                    replies += 1

        counter = asyncio.create_task(count())
        n = 0
        start = time.monotonic()
        next_t = start
        while time.monotonic() - start < seconds:
            if fill:
                for other in (3, 4, 5):
                    await dev.send_frame(frame(other, NOP_48))
            await dev.send_frame(command_at(time.monotonic() - start))
            await dev.send_frame(request)
            if fill:
                for other in (3, 4):
                    await dev.send_frame(frame(other << 8, NOP_48))
            n += 1
            next_t += TICK_S
            await asyncio.sleep(max(0.0, next_t - time.monotonic()))
        await dev.send_frame(frame(dev_id, stop))
        await asyncio.sleep(0.05)
        counter.cancel()
        try:
            await counter
        except asyncio.CancelledError:
            pass
        late = time.monotonic() - next_t
        print(f'          {n} ticks ({n / seconds:.0f}/s, ended {late * 1e3:+.1f} ms '
              f'vs schedule), {replies} telemetry replies', flush=True)

    print('start the capture now; t=0 in 3 s', flush=True)
    await asyncio.sleep(3.0)
    t0 = time.monotonic()
    print('t=0', flush=True)

    def banner(name):
        print(f'{time.monotonic() - t0:6.2f} s  phase {name}', flush=True)

    await report('start')
    try:
        banner('1 robot pattern')
        await ticks(fill=False)
        await report('1')
        await asyncio.sleep(GAP_S)

        banner('2 robot pattern + bus fill')
        await ticks(fill=True)
        await report('2')
        await asyncio.sleep(GAP_S)

        banner('3 request stress')
        qr = moteus.QueryResolution()
        qr._extra = {0x06d: moteus.INT16, 0x06e: moteus.INT16, 0x06f: moteus.INT16}
        c_q = moteus.Controller(id=dev_id, query_resolution=qr)
        n = 0
        end = time.monotonic() + PHASE_S
        while time.monotonic() < end:
            await asyncio.wait_for(c_q.query(), 1.0)
            n += 1
        print(f'          {n / PHASE_S:.0f} queries/s', flush=True)
        await report('3')
        await asyncio.sleep(GAP_S)

        banner(f'4 spin {SPIN_REV_S} rev/s, max torque {SPIN_TORQUE_NM} N m')
        p0 = (await asyncio.wait_for(c.query(), 1.0)).values[moteus.Register.POSITION]
        frame_spin = frame(dev_id, spin)
        frame_down = frame(dev_id, spin_down)
        await ticks(fill=False, seconds=SPIN_S + SPIN_DOWN_S,
                    command_at=lambda t: frame_spin if t < SPIN_S else frame_down)
        p1 = (await asyncio.wait_for(c.query(), 1.0)).values[moteus.Register.POSITION]
        print(f'          turned {p1 - p0:+.2f} rev (expect about '
              f'{SPIN_REV_S * (SPIN_S - SPIN_REV_S / SPIN_ACCEL / 2):.1f} + the spin-down)',
              flush=True)
        await report('4')
        await asyncio.sleep(GAP_S)

        banner('5 enable cycles (zero torque)')
        for _ in range(5):
            await ticks(fill=False, seconds=0.3)
            await asyncio.sleep(0.2)
        await report('5')
        await asyncio.sleep(GAP_S)
    finally:
        await asyncio.wait_for(s.command(b'd stop'), 5.0)
        await asyncio.wait_for(s.command(b'd mark phase'), 5.0)

    fu = await asyncio.wait_for(s.read_data('aux2_fusion'), 8.0)
    print(f'{time.monotonic() - t0:6.2f} s  done; fusion i2c_errors {fu.i2c_errors} '
          f'resyncs {fu.resyncs} gaps {fu.gyro_gaps} arrival_unknown {fu.arrival_unknown}',
          flush=True)


if __name__ == '__main__':
    asyncio.run(main(int(sys.argv[1])))
