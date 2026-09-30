"""Board reply turnaround from a Logic 2 capture of scope_can_capture.py.

Export the capture from Logic 2 as CSV (File > Export Data > CSV, digital
channels only): one row per transition, "Time [s]" then one column per
channel, 0/1.  Channels, logic side of the board's transceiver:

  --tx    the board's CAN TX (only the board drives it: its acks and frames)
  --rx    CAN RX (the whole bus)
  --dbg1  DBG1 (PC14): control interrupt, high while it runs
  --dbg2  DBG2 (PC15), in `d mark can` mode: high while a CAN frame is
          processed, notched low during its fusion work

For every telemetry request (11-bit ID 0x07F, read off RX) this finds the
end of the board's ack pulse for it on TX (the end of the request's ack
slot, the reference in docs/latency.md), the board's block reply
(11-bit 0x700 | id) on RX, and the DBG2 edges in between.  With the block
request, mjlib writes the reply AFTER the frame completes, and the block's
quaternion read is marked by the inverted fusion marker, so DBG2 shows:

  r0  rises: the frame reaches the main loop (StartFrame)
  f1  falls / r1 rises: the frame-start fusion work (BeginFrame)
  f2  falls: the frame is done (CompleteFrame)
  r2  rises: the block's quaternion read is done; DBG2 then stays high
      until the next frame (a marker artifact)

Reported, from the ack end:

  pickup      ack end -> r0
  frame       r0 -> f2 (mjlib's frame processing, BeginFrame included)
  quaternion  f2 -> r2 (response start and the block's quaternion read)
  rest        r2 -> reply start (the other block registers, the send)
  turnaround  ack end -> reply start
  busy        another frame started on the bus inside the turnaround;
              rest and turnaround are also given for replies whose bus
              was idle for >= 11 bit times before they started (never
              held back by it)

With --dbg1, the cpu time (minus control-interrupt time) of pickup ..
rest is printed as well.

Usage: python3 scope_can_turnaround.py capture.csv --id 2 --tx 0 --rx 1
           [--dbg1 2 --dbg2 3]

Without --dbg2 only the turnaround (and busy) is reported; without --dbg1
cpu is left out.
"""

import argparse
import sys

import numpy as np

NOMINAL_BIT_S = 1e-6        # 1 Mbit/s arbitration phase
IDLE_BITS = 11              # recessive bits before a start of frame
DBG2_GAP_S = 60e-6          # DBG2 notches shorter than this join one pulse
ACK_MAX_S = 3e-6            # a TX low pulse this short is an ack


def load(path, columns):
    with open(path) as f:
        header = f.readline().strip().split(',')
    data = np.loadtxt(path, delimiter=',', skiprows=1)
    names = [h.strip() for h in header]
    out = {}
    for key, ch in columns.items():
        name = f'Channel {ch}'
        if name not in names:
            sys.exit(f'{path}: no column "{name}" (columns: {names})')
        col = names.index(name)
        out[key] = data[:, col].astype(int)
    return data[:, 0], out


def edges(t, v):
    """(rise times, fall times) of a 0/1 trace given at transitions."""
    d = np.diff(v)
    return t[1:][d > 0], t[1:][d < 0]


def level_at(t, v, when):
    i = np.searchsorted(t, when, side='right') - 1
    return v[np.clip(i, 0, len(v) - 1)]


def frame_starts(t, rx):
    """Start-of-frame times: RX falls after at least IDLE_BITS recessive."""
    rises, falls = edges(t, rx)
    starts = []
    last_rise = -1.0
    ri = 0
    for f in falls:
        while ri < len(rises) and rises[ri] < f:
            last_rise = rises[ri]
            ri += 1
        if last_rise < 0 or f - last_rise >= IDLE_BITS * NOMINAL_BIT_S * 0.95:
            starts.append(f)
    return np.array(starts)


def decode_id(t, rx, sof):
    """(11-bit base id, extended?) from the nominal-rate bits after SOF,
    sampled mid-bit, with CAN bit stuffing removed."""
    bits = []
    run_val, run_len = 0, 1          # SOF is one dominant bit
    k = 1
    while len(bits) < 13 and k < 40:
        b = int(level_at(t, rx, sof + (k + 0.5) * NOMINAL_BIT_S))
        k += 1
        if run_len == 5:             # stuff bit: opposite of the run, dropped
            run_val, run_len = b, 1
            continue
        run_len = run_len + 1 if b == run_val else 1
        run_val = b
        bits.append(b)
    if len(bits) < 13:
        return None, None
    base = 0
    for b in bits[:11]:
        base = (base << 1) | b
    return base, bits[12] == 1       # bit 12 is IDE (bit 11 RRS / SRR)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('csv')
    ap.add_argument('--id', type=int, required=True, help='board id')
    for name in ('tx', 'rx', 'dbg1', 'dbg2'):
        ap.add_argument(f'--{name}', type=int, required=name in ('tx', 'rx'),
                        help=f'Logic 2 channel number of {name.upper()}')
    args = ap.parse_args()

    t, ch = load(args.csv, {k: getattr(args, k) for k in ('tx', 'rx', 'dbg1', 'dbg2')
                            if getattr(args, k) is not None})
    sofs = frame_starts(t, ch['rx'])
    frames = [(s,) + decode_id(t, ch['rx'], s) for s in sofs]
    requests = [s for s, i, ext in frames if i == 0x07F and ext is False]
    replies = np.array([s for s, i, ext in frames
                        if i == (0x700 | args.id) and ext is False])
    print(f'{len(sofs)} frames, {len(requests)} telemetry requests (0x07F), '
          f'{len(replies)} block replies (0x{0x700 | args.id:03x})')

    tx_rises, tx_falls = edges(t, ch['tx'])
    no_edges = (np.zeros(0), np.zeros(0))
    d2_rises, d2_falls = edges(t, ch['dbg2']) if 'dbg2' in ch else no_edges
    d1_rises, d1_falls = edges(t, ch['dbg1']) if 'dbg1' in ch else no_edges

    rx_rises, _ = edges(t, ch['rx'])

    def isr_in(lo, hi):
        """DBG1-high time inside [lo, hi]."""
        total = 0.0
        for r in d1_rises[(d1_rises > lo - 40e-6) & (d1_rises < hi)]:
            k = np.searchsorted(d1_falls, r, side='right')
            if k == len(d1_falls):
                continue
            a, b = max(r, lo), min(d1_falls[k], hi)
            if b > a:
                total += b - a
        return total

    names = ('pickup', 'frame', 'quaternion', 'rest', 'turnaround')
    wall, cpu, busy, idle_ok = [], [], [], []
    for k, req in enumerate(requests):
        # the reply must come before the next request (else it was lost)
        limit = requests[k + 1] if k + 1 < len(requests) else np.inf
        nxt = replies[(replies > req) & (replies < limit)]
        if not len(nxt):
            continue
        reply = nxt[0]
        # the board's ack for the request: the first short TX low pulse
        # after the request's SOF
        ack_end = None
        for f in tx_falls[(tx_falls > req) & (tx_falls < reply)]:
            r = tx_rises[tx_rises > f]
            if len(r) and r[0] - f <= ACK_MAX_S:
                ack_end = r[0]
                break
        if ack_end is None:
            continue
        points = [ack_end]
        if 'dbg2' in ch:
            r0 = d2_rises[(d2_rises > ack_end) & (d2_rises < reply)]
            if not len(r0):
                continue
            r0 = r0[0]
            f = d2_falls[d2_falls > r0]
            r = d2_rises[d2_rises > r0]
            if len(f) < 2 or len(r) < 2 or not (f[0] < r[0] < f[1] < r[1] < reply):
                continue
            points += [r0, f[1], r[1]]
        points.append(reply)
        segs = np.diff(points)
        if 'dbg2' in ch:
            wall.append(list(segs) + [reply - ack_end])
            if 'dbg1' in ch:
                cpu.append([b - a - isr_in(a, b)
                            for a, b in zip(points[:-1], points[1:])]
                           + [reply - ack_end - isr_in(ack_end, reply)])
        else:
            wall.append([np.nan] * 4 + [reply - ack_end])
        busy.append(bool(np.any((sofs > ack_end) & (sofs < reply))))
        before = rx_rises[rx_rises < reply]
        idle_ok.append(len(before) and reply - before[-1] >= 11 * NOMINAL_BIT_S)

    if not wall:
        sys.exit('no request/reply pairs found: check the channel numbers '
                 'and that the capture is the `d mark can` build')
    wall = np.array(wall) * 1e6
    idle_ok = np.array(idle_ok, dtype=bool)
    print(f'{len(wall)} request/reply pairs, {sum(busy)} with another frame '
          f'in the way, {idle_ok.sum()} with the bus idle >= 11 bits before '
          'the reply')

    def table(title, a):
        print(f'{title:>14} {"median":>8} {"p90":>8} {"max":>8}')
        for k, name in enumerate(names):
            col = a[:, k]
            if np.isnan(col).all():
                continue
            print(f'{name:>14} {np.median(col):8.1f} {np.percentile(col, 90):8.1f} '
                  f'{col.max():8.1f}')

    table('wall us', wall)
    if (~idle_ok).any() and idle_ok.any():
        sub = wall[idle_ok]
        for k in (3, 4):
            col = sub[:, k]
            if not np.isnan(col).all():
                print(f'{names[k] + "*":>14} {np.median(col):8.1f} '
                      f'{np.percentile(col, 90):8.1f} {col.max():8.1f}'
                      '   (* bus idle before the reply)')
    if cpu:
        table('cpu us', np.array(cpu) * 1e6)


if __name__ == '__main__':
    main()
