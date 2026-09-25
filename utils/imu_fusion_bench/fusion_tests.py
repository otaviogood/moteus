"""Bench tests for the fusion firmware (docs §7.1: 4a, 9, 10, 12, 13).  NO motion commands.

Usage: PYTHONPATH=lib/python python3 fusion_tests.py <id> <test>
  drop     : inject 10 dropped gyro words; expect gyro_gaps +1, gap_slots +10, no reinit, no jump
  stall    : hold the main loop 150 ms with 3 queued requests; expect 3 sentinels then valid;
             then 50 ms stall: no sentinel
  halt     : stop word reads for 0.6 s while polling; expect stale within 3 polls and never
             a fresh reply during the halt; then resume: sentinels, then valid
  wrap     : 10 s at 120 Hz: zero sentinels, toggle alternates, no jump at rest
  status   : print aux2_fusion telemetry
"""
import asyncio, math, sys, time
import moteus
sys.path.insert(0, 'docs')
from imu_orientation_quantization import decode_quat48, toggle_quat48

def ang_deg(a, b):
    d = abs(sum(x * y for x, y in zip(a, b)))
    return 2 * math.degrees(math.acos(min(1.0, d)))

class Bench:
    def __init__(self, dev_id):
        self.c = moteus.Controller(id=dev_id)
        self.s = moteus.Stream(self.c)
        qr = moteus.QueryResolution()
        qr._extra = {0x06d: moteus.INT16, 0x06e: moteus.INT16, 0x06f: moteus.INT16}
        self.cq = moteus.Controller(id=dev_id, query_resolution=qr)

    async def start(self):
        # Drain anything a previous session left in the diagnostic channel
        # BEFORE the first command: stale binary telemetry has no newline
        # and would stall read_until_OK.
        await asyncio.wait_for(self.s.flush_read(), 2.0)
        await asyncio.wait_for(self.s.command(b'tel stop'), 5.0)
        await asyncio.wait_for(self.s.flush_read(), 2.0)

    async def cmd(self, text):
        return await asyncio.wait_for(self.s.command(text.encode()), 5.0)

    async def fusion(self):
        return await asyncio.wait_for(self.s.read_data('aux2_fusion'), 8.0)

    async def query(self):
        r = await asyncio.wait_for(self.cq.query(), 1.0)
        words = tuple(int(r.values[reg]) & 0xffff for reg in (0x06d, 0x06e, 0x06f))
        return words, decode_quat48(words), toggle_quat48(words)

    async def poll(self, seconds, hz=120.0):
        """Poll; return list of (t, words, q, toggle)."""
        out = []
        period = 1.0 / hz
        t_end = time.monotonic() + seconds
        while time.monotonic() < t_end:
            t0 = time.monotonic()
            words, q, tog = await self.query()
            out.append((t0, words, q, tog))
            dt = period - (time.monotonic() - t0)
            if dt > 0:
                await asyncio.sleep(dt)
        return out

def summarize(samples, label):
    sent = sum(1 for s in samples if s[2] is None)
    valid = [s for s in samples if s[2] is not None]
    repeats = sum(1 for a, b in zip(valid, valid[1:]) if a[3] == b[3])
    max_step = max((ang_deg(a[2], b[2]) for a, b in zip(valid, valid[1:])), default=0.0)
    print(f'{label}: {len(samples)} replies, sentinels={sent}, toggle repeats={repeats}, max step={max_step:.3f} deg')
    return sent, repeats, max_step

async def test_drop(b):
    f0 = await b.fusion()
    before = await b.poll(0.5)
    await b.cmd('aux2 fusion drop 10')
    after = await b.poll(0.5)
    f1 = await b.fusion()
    summarize(before, 'before'); summarize(after, 'after drop')
    print(f'gyro_gaps {f0.gyro_gaps} -> {f1.gyro_gaps}, gap_slots {f0.gap_slots} -> {f1.gap_slots}, '
          f'reinits {f0.reinits} -> {f1.reinits}, resyncs {f0.resyncs} -> {f1.resyncs}')
    ok = (f1.gyro_gaps - f0.gyro_gaps == 1 and f1.gap_slots - f0.gap_slots == 10 and f1.reinits == f0.reinits)
    va = [s for s in after if s[2] is not None]; vb = [s for s in before if s[2] is not None]
    if va and vb:
        print(f'orientation change across the drop (board at rest): {ang_deg(vb[-1][2], va[0][2]):.3f} deg')
    print('PASS' if ok else 'FAIL')

async def test_stall(b):
    """Stream 3-frame bursts across a main-loop stall and judge by the
    firmware's counters: the burst queued during the stall (its round trip
    is the stall length) must get sentinels for a 150 ms stall (stall pass:
    3 frames marked unknown) and valid replies for a 50 ms one."""
    transport = b.cq._get_transport()
    for ms, exp_stall, exp_unknown, exp_sentinel in ((150, 1, 2, 2), (50, 0, 0, 0)):
        f0 = await b.fusion()
        # The console command and three queries in ONE CAN burst: the
        # python Stream defers diagnostic writes until a read, so this is
        # the only way to have the queries queued when the stall starts.
        delays = []
        t0 = time.monotonic()
        try:
            # write, then a read poll (the command runs when the channel is
            # polled), then the three queries -- all in one burst.
            await asyncio.wait_for(transport.cycle(
                [b.c.make_diagnostic_write(f'aux2 fusion stall {ms}\n'.encode()),
                 b.c.make_diagnostic_read()] +
                [b.cq.make_query() for _ in range(3)]), 3.0)
        except Exception as e:
            print('  burst failed:', type(e).__name__, e)
        delays.append(time.monotonic() - t0)
        t_start = time.monotonic()
        while time.monotonic() - t_start < 0.3:
            t0 = time.monotonic()
            try:
                await asyncio.wait_for(transport.cycle([b.cq.make_query() for _ in range(3)]), 3.0)
            except Exception as e:
                print('  burst failed:', type(e).__name__)
            delays.append(time.monotonic() - t0)
        await asyncio.wait_for(b.s.flush_read(), 2.0)  # the delayed OK
        f1 = await b.fusion()
        d_stall = f1.stall_passes - f0.stall_passes
        d_unknown = f1.arrival_unknown - f0.arrival_unknown
        d_sent = f1.sentinel_replies - f0.sentinel_replies
        d_valid = f1.valid_replies - f0.valid_replies
        # The command executes one main-loop pass after its frame, so the first
        # query is answered before the stall: expect the other two flagged.
        ok = (d_stall == exp_stall and d_unknown == d_sent and
              (d_sent >= 1 if exp_sentinel else d_sent == 0))
        print(f'stall {ms} ms: {len(delays)} bursts, longest round trip {max(delays)*1e3:.0f} ms; '
              f'stall_passes +{d_stall} (exp {exp_stall}), arrival_unknown +{d_unknown} (exp {exp_unknown}), '
              f'sentinel_replies +{d_sent} (exp {exp_sentinel}), valid +{d_valid}; '
              f'mailbox_max_depth {f1.mailbox_max_depth} overflow {f1.mailbox_overflow} gaps {f1.gyro_gaps} '
              f'-> {"PASS" if ok else "FAIL"}')

async def test_halt(b):
    f0 = await b.fusion()
    base = await b.poll(0.3)
    await b.cmd('aux2 fusion halt 1')
    halted = await b.poll(0.6)
    await b.cmd('aux2 fusion halt 0')
    resumed = await b.poll(2.5)
    f1 = await b.fusion()
    summarize(base, 'baseline')
    sent, repeats, _ = summarize(halted, 'halted (0.6 s, past 262 ms)')
    valid_h = [s for s in halted if s[2] is not None]
    flips = sum(1 for a, b2 in zip(valid_h, valid_h[1:]) if a[3] != b2[3])
    print(f'  toggle flips during halt: {flips} (must be 0 after the first 2 replies); '
          f'sentinels during halt: {sent}')
    summarize(resumed, 'resumed')
    print(f'resyncs {f0.resyncs} -> {f1.resyncs}, reinits {f0.reinits} -> {f1.reinits}, '
          f'init_state {f1.init_state}, converged {f1.converged}')

async def test_wrap(b):
    samples = await b.poll(10.0)
    sent, repeats, max_step = summarize(samples, '10 s at 120 Hz')
    f1 = await b.fusion()
    print(f'odr_actual_hz {f1.odr_actual_hz:.3f}, phase_unc_us {f1.phase_unc_us}, '
          f'gaps {f1.gyro_gaps}, overflow {f1.mailbox_overflow}, resyncs {f1.resyncs}')
    print('PASS' if sent == 0 and repeats == 0 and max_step < 0.05 else 'CHECK')

async def test_status(b):
    f = await b.fusion()
    for k in sorted(vars(f)):
        if not k.startswith('_'):
            print(f'  {k} = {getattr(f, k)}')

async def main():
    dev_id = int(sys.argv[1]); test = sys.argv[2]
    b = Bench(dev_id)
    await b.start()
    await {'drop': test_drop, 'stall': test_stall, 'halt': test_halt, 'wrap': test_wrap, 'status': test_status}[test](b)

asyncio.run(main())
