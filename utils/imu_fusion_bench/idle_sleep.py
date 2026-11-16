"""Idle sleep bench test (fw/idle_sleep.h): put one board to sleep, keep
the bus silent, wake it and report what happened.  Nothing is commanded
to move; the board only sleeps while its motor is stopped (mode 0).

Usage: PYTHONPATH=lib/python python3 utils/imu_fusion_bench/idle_sleep.py <id>
           [--asleep SECONDS] [--watchdog SECONDS] [--no-wake] [--no-bus-wake]

The sleep request (68 01 ss) is BROADCAST: every board on the bus that
has this firmware and a stopped motor goes to sleep.  Close tview and
anything else that talks on the bus first: any frame wakes the boards.

  --asleep S    how long to keep the bus silent (default 10): the time
                to read the supply's current
  --watchdog S  1..32: the board also wakes by itself after S seconds
                (the firmware's bench safety net).  Default 0: only the
                bus wakes it.
  --no-wake     send nothing after the silence
  --no-bus-wake bench flag: the bus does not wake the board, only the
                watchdog does (needs --watchdog)

After the silence it asks the board for its uptime every 10 ms until it
answers.  The uptime (a millisecond counter that starts at reset) tells
when the board restarted:
  - at the first wake frame: it slept the whole time and the bus woke it
  - earlier: it woke by itself (the watchdog, or bus noise)
  - never (uptime longer than the test): it did not sleep
"""
import argparse
import asyncio
import time

import moteus
from moteus.transport_device import Frame

BROADCAST_ID = 0x7F
SLEEP_REQUEST = 0x68
SLEEP_FORMAT = 0x01
UPTIME = 0x070
UPTIME_START = (1 << 31) - 300000
NO_BUS_WAKE = 0x01
MODE = moteus.Register.MODE


def frame(arbitration_id, data):
    f = Frame()
    f.arbitration_id = arbitration_id
    f.data = data
    f.is_extended_id = arbitration_id > 0x7FF
    f.is_fd = True
    f.bitrate_switch = True
    return f


async def query(c, timeout):
    r = await asyncio.wait_for(
        c.custom_query({UPTIME: moteus.multiplex.INT32,
                        MODE: moteus.multiplex.INT8}), timeout)
    # The counter starts 300 s before its wrap (fw/system_info.cc).
    return (r.values[UPTIME] - UPTIME_START) & 0xFFFFFFFF, r.values[MODE]


async def main(args):
    c = moteus.Controller(id=args.id)
    dev = moteus.get_singleton_transport()._devices[0]

    uptime_ms, mode = await query(c, 2.0)
    print(f'before: up {uptime_ms / 1000.0:.1f} s, mode {mode}')
    if mode != 0:
        print('the motor is not stopped (mode 0): the board would ignore the request')
        return

    request = [SLEEP_REQUEST, SLEEP_FORMAT, args.watchdog]
    if args.no_bus_wake:
        if not args.watchdog:
            print('--no-bus-wake needs --watchdog')
            return
        request.append(NO_BUS_WAKE)
    await dev.send_frame(frame(BROADCAST_ID, bytes(request)))
    t_sleep = time.monotonic()
    print(f'sleep request sent; bus silent for {args.asleep:.0f} s '
          '(read the supply current now)')
    await asyncio.sleep(args.asleep)
    if args.no_wake:
        print('no wake frame sent')
        return

    t_wake = time.monotonic()
    attempts = 0
    while True:
        attempts += 1
        try:
            uptime_ms, mode = await query(c, 0.010)
            break
        except asyncio.TimeoutError:
            if time.monotonic() - t_wake > 5.0:
                print(f'no answer in 5 s ({attempts} requests): the board did not wake')
                return
    t_reply = time.monotonic()
    restarted = t_reply - uptime_ms / 1000.0
    print(f'first answer {1000.0 * (t_reply - t_wake):.0f} ms after the first wake '
          f'frame (request {attempts}); up {uptime_ms} ms, mode {mode}')
    since_sleep = restarted - t_sleep
    if since_sleep < -0.5:
        print('the board has been up since before the request: it did not sleep')
    elif restarted >= t_wake - 0.05:
        print(f'it restarted {1000.0 * (restarted - t_wake):+.0f} ms from the first wake '
              f'frame: asleep for the whole {args.asleep:.0f} s, woken by the bus')
    else:
        print(f'it restarted {since_sleep:.2f} s after the sleep request, before any '
              'wake frame: woken by the watchdog or by something on the bus')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('id', type=int)
    parser.add_argument('--asleep', type=float, default=10.0)
    parser.add_argument('--watchdog', type=int, default=0)
    parser.add_argument('--no-wake', action='store_true')
    parser.add_argument('--no-bus-wake', action='store_true')
    asyncio.run(main(parser.parse_args()))
