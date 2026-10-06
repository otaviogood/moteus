"""Logic-analyzer capture with robot-like CAN traffic, for the
MOTEUS_SCOPE_MARKERS build on the BENCH motor.  Phase 4 SPINS THE MOTOR,
gently: the rotor must be free.  It only runs when asked for (--phases).

Usage: PYTHONPATH=lib/python python3 utils/imu_fusion_bench/scope_can_capture.py <id>
           [--phases 1,2,3,5]
Analysis: utils/imu_fusion_bench/scope_can_turnaround.py.

Probes: CAN TX and RX (logic side of the transceiver), DBG1 (control
interrupt) and DBG2, which this script switches to the CAN frame marker
(`d mark can`: high from the start of a frame's processing until its
command reaches the servo, or the end of the frame if it carries none,
notched low while the frame's IMU fusion work runs).

Phases (3 s unless noted), separated by 1 s with no bus traffic at all:

  1  robot pattern   120 Hz ticks, each: a position command to the board
                     (max torque 0, no reply), then the robot's broadcast
                     telemetry block request (60 02, 11-bit ID 0x07F; the
                     block reply comes back as 11-bit 0x700 | id)
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
  6  command updates the robot's tick (orin/rl_api/driver.py), 10 s: one
                     full zero-torque command, then 120 Hz ticks, each: the
                     telemetry block request, then COMMAND_AT_S later the
                     robot's per-tick command update (0x64, then
                     0x020/0x021 and 0x023/0x024 as floats, 11-bit ID) to
                     the board, between dummy frames of the same size to
                     absent boards (the other joints on a robot bus).  For
                     command frame -> in use (scope_can_turnaround.py
                     reports every command frame); capture it alone
                     (--phases 6).

Phases 1-3, 5 and 6 command zero torque.  Each phase ends with a stop
command and a report: isr_max (its last full second), the mode after the
stop, and the gate driver's status reads and register readback (fsr1/fsr2
must stay 0 and fault_config 0; a corrupted SPI read would show up as
stray bits).  The dummy frames are addressed so that the board's hardware filter drops
them; they only occupy the bus.
"""
import argparse
import asyncio
import math
import struct
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
UPDATE_S = 10.0
# Phase 6: the runner sends at command_delay_s - COMMAND_SEND_TO_IN_USE_S
# after the request (orin/robot_metadata.py; 5.833 - 0.5 ms).
COMMAND_AT_S = 5.334e-3
# orin/moteus_command.py UPDATE: a command update keeps the board's command
# and changes only the registers it writes (fw/command_update.h).
COMMAND_UPDATE = 0x64
# Absent boards for the phase-6 dummies (never 7: its replies would look
# like 11-bit block replies).  NOPs, so a board that is there ignores them.
DUMMY_IDS = (4, 5, 6, 8, 9)
CYCLES_PER_US = 170.0
BROADCAST_ID = 0x7F
# orin/can_backend.request_joint_telemetry: the robot's per-tick request,
# the telemetry block (fw/telemetry_block.h), answered without the
# reply-request bit.
TELEMETRY_REQUEST = bytes([0x60, 0x02])
NOP_48 = bytes([0x50] * 48)


def float_writes(regs):
    """orin/moteus_command.float_writes: float WRITE subframes for
    [(register, value), ...] in ascending order, runs of up to 3 sharing
    one header."""
    out = bytearray()
    i = 0
    while i < len(regs):
        n = 1
        while n < 3 and i + n < len(regs) and regs[i + n][0] == regs[i][0] + n:
            n += 1
        out += bytes([0x0C + n, regs[i][0]])
        for _reg, value in regs[i:i + n]:
            out += struct.pack('<f', value)
        i += n
    return bytes(out)


def command_update(position, velocity, kp_scale, kd_scale):
    """The robot's per-tick command update in the rate modes
    (canmonitor_runner per_tick_command_registers: 0x020, 0x021, 0x023,
    0x024), in revolutions and rev/s like the wire."""
    return bytes([COMMAND_UPDATE]) + float_writes(
        [(0x020, position), (0x021, velocity), (0x023, kp_scale), (0x024, kd_scale)])


def frame(arbitration_id, data):
    f = Frame()
    f.arbitration_id = arbitration_id
    f.data = data
    # 11-bit whenever the ID fits, like the robot host (orin/can_utils.py)
    f.is_extended_id = arbitration_id > 0x7FF
    f.is_fd = True
    f.bitrate_switch = True
    return f


async def main(dev_id, phases):
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
    request = frame(BROADCAST_ID, TELEMETRY_REQUEST)

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
                if f.arbitration_id == (0x700 | dev_id):  # block reply
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
        if 1 in phases:
            banner('1 robot pattern')
            await ticks(fill=False)
            await report('1')
            await asyncio.sleep(GAP_S)

        if 2 in phases:
            banner('2 robot pattern + bus fill')
            await ticks(fill=True)
            await report('2')
            await asyncio.sleep(GAP_S)

        if 3 in phases:
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

        if 4 in phases:
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

        if 5 in phases:
            banner('5 enable cycles (zero torque)')
            for _ in range(5):
                await ticks(fill=False, seconds=0.3)
                await asyncio.sleep(0.2)
            await report('5')
            await asyncio.sleep(GAP_S)
        if 6 in phases:
            banner(f'6 command updates, {UPDATE_S:.0f} s, sent {COMMAND_AT_S * 1e3:.3f} ms '
                   'after each request (zero torque)')
            # The full command the updates keep: position mode, max torque 0.
            await dev.send_frame(command)
            await asyncio.sleep(0.02)
            size = len(command_update(0.0, 0.0, 0.0, 0.0))
            dummy = [frame(other, bytes([0x50] * size))
                     for other in DUMMY_IDS if other != dev_id]
            replies = 0

            async def count():
                nonlocal replies
                while True:
                    f = await dev.receive_frame()
                    if f.arbitration_id == (0x700 | dev_id):
                        replies += 1

            counter = asyncio.create_task(count())
            n = late = 0
            start = time.monotonic()
            next_t = start
            while next_t - start < UPDATE_S:
                await dev.send_frame(request)
                t_req = time.monotonic()
                # asyncio.sleep is ~1 ms coarse: busy-wait the last stretch
                await asyncio.sleep(max(0.0, t_req + COMMAND_AT_S - 1.5e-3 - time.monotonic()))
                while time.monotonic() < t_req + COMMAND_AT_S:
                    pass
                if time.monotonic() - t_req > COMMAND_AT_S + 0.5e-3:
                    late += 1
                # a slowly changing position, never used (max torque 0)
                upd = frame(dev_id, command_update(1e-3 * math.sin(n / 60.0), 0.0, 0.0, 0.0))
                for f in dummy[:2] + [upd] + dummy[2:]:
                    await dev.send_frame(f)
                n += 1
                next_t += TICK_S
                await asyncio.sleep(max(0.0, next_t - time.monotonic()))
            counter.cancel()   # before the query: it would take its reply
            try:
                await counter
            except asyncio.CancelledError:
                pass
            # Still in position mode after the updates (each re-commands the
            # full command and restarts its watchdog)?
            mode = (await asyncio.wait_for(c.query(), 1.0)).values[moteus.Register.MODE]
            await dev.send_frame(frame(dev_id, stop))
            await asyncio.sleep(0.05)
            print(f'          {n} ticks, {late} updates sent > 0.5 ms late (host), '
                  f'{replies} telemetry replies, mode before the stop {int(mode)}'
                  f'{"" if int(mode) == 10 else "  <-- expected 10 (position)"}', flush=True)
            await report('6')
            await asyncio.sleep(GAP_S)
    finally:
        await asyncio.wait_for(s.command(b'd stop'), 5.0)
        await asyncio.wait_for(s.command(b'd mark phase'), 5.0)

    fu = await asyncio.wait_for(s.read_data('aux2_fusion'), 8.0)
    print(f'{time.monotonic() - t0:6.2f} s  done; fusion i2c_errors {fu.i2c_errors} '
          f'resyncs {fu.resyncs} gaps {fu.gyro_gaps} arrival_unknown {fu.arrival_unknown}',
          flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('id', type=int)
    parser.add_argument('--phases', default='1,2,3,5',
                        help='comma-separated phases to run; 4 spins the motor '
                             '(default 1,2,3,5: zero torque only; 6, the '
                             'command-update capture, is run alone)')
    args = parser.parse_args()
    asyncio.run(main(args.id, {int(x) for x in args.phases.split(',') if x}))
