"""Hand-rock return-to-pose test for the aux2 fusion (READ-ONLY: no motion commands).

Usage: PYTHONPATH=lib/python python3 utils/imu_fusion_bench/rock_test.py <id> [--out file.npz]
           [--seconds 60]
       python3 utils/imu_fusion_bench/rock_test.py --analyze file.npz

Two questions, no external reference needed:

  drift   Lay the board flat against a corner (or any fixed stop), rock it
          by hand, put it back against the same corner.  Its true
          orientation is the same as before, so whatever rotation the
          fusion reports between the two still periods is filter error;
          the tilt part should then decay (accelerometer correction), the
          heading part cannot.
  timing  The quaternion is integrated from the gyro, so its rate (from
          consecutive quaternions) should line up with the gyro registers
          0x065-0x067 (the newest sample's bias-corrected rate) read in the
          SAME reply, ahead by at most the extrapolation from the newest
          sample to the request (~0-1 ms).  A lead of several ms means the
          fusion's sample times are off.

Cues (printed live): 0-6 s still in the corner; 6-16 s slow rocking (tilt
+-30 deg, ~1 Hz, one axis); 16-31 s fast rocking (tilt +-45 deg, 2-3 Hz:
about 5-10 rad/s, like a foot shaken by hand); at 31 s put it back in the
corner exactly and leave it alone to the end.

Polls 0x065-0x067 (float) and 0x06d-0x06f (int16) in one query as fast as
the adapter answers (~500 Hz on the fdcanusb), host-stamped, and prints
the aux2_fusion telemetry before and after.
"""
import argparse
import asyncio
import json
import math
import sys
import time

import numpy as np

sys.path.insert(0, 'docs')

GYRO_REGS = (0x065, 0x066, 0x067)
QUAT_REGS = (0x06d, 0x06e, 0x06f)
GYRO_FULL_SCALE = math.radians(2000.0)
CUES = [(0, 'STILL in the corner'), (6, 'slow rocking: +-30 deg, ~1 Hz, one axis'),
        (16, 'FAST rocking: +-45 deg, 2-3 Hz'), (31, 'put it BACK in the corner, hands off')]
FUSION_FIELDS = ['initialized', 'converged', 'timing_degraded', 'last_reinit_reason',
                 'freq_fine', 'odr_actual_hz', 'gyro_words', 'accel_words', 'gyro_gaps',
                 'gap_slots', 'mailbox_overflow', 'mailbox_max_depth', 'fifo_overruns',
                 'resyncs', 'reinits', 'i2c_errors', 'stall_passes', 'arrival_unknown',
                 'sentinel_replies', 'valid_replies', 'phase_unc_us', 'bias']


async def capture(dev_id, seconds):
    import moteus
    from imu_orientation_quantization import decode_quat48
    c = moteus.Controller(id=dev_id)
    s = moteus.Stream(c)
    await asyncio.wait_for(s.flush_read(), 2.0)
    await asyncio.wait_for(s.command(b'tel stop'), 5.0)
    await asyncio.wait_for(s.flush_read(), 2.0)

    async def status():
        fu = await asyncio.wait_for(s.read_data('aux2_fusion'), 8.0)
        return {n: (list(getattr(fu, n)) if n == 'bias' else getattr(fu, n))
                for n in FUSION_FIELDS if hasattr(fu, n)}

    before = await status()
    fields = {r: moteus.F32 for r in GYRO_REGS}
    fields.update({r: moteus.INT16 for r in QUAT_REGS})
    t, g, q = [], [], []
    cue = 0
    print('starting in 2 s', flush=True)
    await asyncio.sleep(2.0)
    t0 = time.monotonic()
    while True:
        el = time.monotonic() - t0
        if el >= seconds:
            break
        if cue < len(CUES) and el >= CUES[cue][0]:
            print(f'  t={el:4.1f}s  >>> {CUES[cue][1]}', flush=True)
            cue += 1
        ts = time.monotonic()
        r = await asyncio.wait_for(c.custom_query(fields), 1.0)
        words = tuple(int(r.values[k]) & 0xFFFF for k in QUAT_REGS)
        qq = decode_quat48(words)
        t.append(ts - t0)
        g.append([float(r.values[k]) for k in GYRO_REGS])
        q.append(qq if qq is not None else (math.nan,) * 4)
    after = await status()
    return np.array(t), np.array(g), np.array(q, dtype=float), before, after


def quat_mul(a, b):
    w1, x1, y1, z1 = a[..., 0], a[..., 1], a[..., 2], a[..., 3]
    w2, x2, y2, z2 = b[..., 0], b[..., 1], b[..., 2], b[..., 3]
    return np.stack([w1*w2 - x1*x2 - y1*y2 - z1*z2, w1*x2 + x1*w2 + y1*z2 - z1*y2,
                     w1*y2 - x1*z2 + y1*w2 + z1*x2, w1*z2 + x1*y2 - y1*x2 + z1*w2], -1)


def conj(q):
    return q * np.array([1.0, -1.0, -1.0, -1.0])


def rotvec(q):
    q = q * np.where(q[..., :1] < 0, -1.0, 1.0)
    s = np.linalg.norm(q[..., 1:], axis=-1)
    ang = 2 * np.arctan2(s, q[..., 0])
    return q[..., 1:] / np.maximum(s, 1e-12)[..., None] * ang[..., None]


def up_body(q):
    w, x, y, z = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
    return np.stack([2*(x*z - w*y), 2*(w*x + y*z), w*w - x*x - y*y + z*z], -1)


def mean_quat(q):
    q = q * np.where((q @ q[0])[:, None] < 0, -1.0, 1.0)
    m = q.mean(0)
    return m / np.linalg.norm(m)


def xcorr_lead_ms(ref, test, fs, max_ms=20.0, up=10):
    """ms by which ``test`` LEADS ``ref`` (Pearson over the overlap,
    linear upsampling, parabolic peak)."""
    n = len(ref)
    tt = np.arange(n) / fs
    tu = np.arange(0, tt[-1], 1 / (fs * up))
    a = np.interp(tu, tt, ref)
    b = np.interp(tu, tt, test)
    N = len(tu)
    L = int(max_ms / 1000 * fs * up)
    lags = np.arange(-L, L + 1)
    c = []
    for l in lags:
        x = a[max(0, -l):N - max(0, l)]
        y = b[max(0, l):N - max(0, -l)]
        x = x - x.mean()
        y = y - y.mean()
        c.append(np.mean(x * y) / (x.std() * y.std()))
    c = np.array(c)
    k = int(np.argmax(c))
    d = 0.0
    if 0 < k < len(c) - 1:
        den = c[k - 1] - 2 * c[k] + c[k + 1]
        d = 0.5 * (c[k - 1] - c[k + 1]) / den if den else 0.0
    return -(lags[k] + d) / (fs * up) * 1000, float(c[k])


def analyze(t, g, q, before, after):
    ok = np.all(np.isfinite(q), axis=1)
    dt = np.diff(t)
    print(f'{len(t)} replies in {t[-1]:.1f} s ({len(t) / t[-1]:.0f} Hz, p99 gap {1e3 * np.percentile(dt, 99):.1f} ms), '
          f'{(~ok).sum()} sentinels')
    for k in ('gyro_gaps', 'gap_slots', 'fifo_overruns', 'mailbox_overflow', 'resyncs', 'reinits',
              'i2c_errors', 'stall_passes', 'arrival_unknown', 'sentinel_replies'):
        if k in before and k in after:
            d = after[k] - before[k]
            print(f'  {k:18s} +{d}' + ('   <--' if d and k not in ('sentinel_replies',) else ''))
    for k in ('odr_actual_hz', 'freq_fine', 'phase_unc_us', 'timing_degraded', 'last_reinit_reason'):
        if k in after:
            print(f'  {k:18s} {after[k]}')
    gmax = np.abs(g[ok]).max()
    print(f'gyro max |axis| {gmax:.2f} rad/s ({100 * gmax / GYRO_FULL_SCALE:.0f}% of +-2000 dps full scale)')

    t, g, q = t[ok], g[ok], q[ok].copy()
    for i in range(1, len(q)):
        if np.dot(q[i], q[i - 1]) < 0:
            q[i] = -q[i]
    wmag = np.linalg.norm(g, axis=1)
    moving = wmag > 0.2
    if not moving.any():
        print('no motion seen (|gyro| > 0.2 rad/s)')
        return
    t_start = t[np.argmax(moving)]
    t_stop = t[len(t) - 1 - np.argmax(moving[::-1])]
    print(f'motion {t_start:.1f} .. {t_stop:.1f} s (|gyro| > 0.2 rad/s)')

    # --- drift: still before vs still after, same corner ---
    pre = (t > 1.0) & (t < t_start - 0.3)
    if pre.sum() < 50:
        print('drift: not enough still time before the motion')
    else:
        q_ref = mean_quat(q[pre])
        up_ref = up_body(q_ref)
        print('drift vs the still pose before the motion (deg): total rotation / tilt / heading')
        for after_s in (0.5, 1, 2, 5, 10, 20, 30):
            w = (t > t_stop + after_s - 0.25) & (t < t_stop + after_s + 0.25)
            if w.sum() < 10:
                continue
            qa = mean_quat(q[w])
            rot = np.degrees(np.linalg.norm(rotvec(quat_mul(conj(q_ref), qa))))
            tilt = np.degrees(np.arccos(np.clip(np.dot(up_body(qa), up_ref), -1, 1)))
            # heading: rotation about world up between the two
            d = quat_mul(qa, conj(q_ref))
            head = np.degrees(rotvec(d)[2])
            print(f'  {after_s:5.1f} s after: {rot:6.2f} / {tilt:6.2f} / {head:+7.2f}')

    # --- timing: quat-derived body rate vs the gyro registers, same replies ---
    fs = 500.0
    tg = np.arange(t_start, t_stop, 1 / fs)
    if len(tg) < fs * 3:
        print('timing: not enough motion')
        return
    qg = np.stack([np.interp(tg, t, q[:, k]) for k in range(4)], 1)
    qg /= np.linalg.norm(qg, axis=1, keepdims=True)
    w_q = rotvec(quat_mul(conj(qg[:-2]), qg[2:])) * fs / 2      # central difference, body frame
    w_q = np.vstack([w_q[:1], w_q, w_q[-1:]])
    w_g = np.stack([np.interp(tg, t, g[:, k]) for k in range(3)], 1)
    ax = int(np.argmax(np.var(w_g, axis=0)))
    gain = np.polyfit(w_g[:, ax], w_q[:, ax], 1)[0]
    lead, c = xcorr_lead_ms(w_g[:, ax], w_q[:, ax], fs)
    print(f'timing (axis {ax}, the most excited): quat rate vs gyro register gain {gain:.3f}, '
          f'quat LEADS the gyro by {lead:+.2f} ms (c={c:.3f}); expected latency_comp - (0.3..0.5) ms: '
          f'~1.5-1.7 with the production 2.0 ms compensation, 0..~1 with `fusion latency 0`')
    for name, sel in (('slow', (tg >= 6) & (tg < 16)), ('fast', (tg >= 16) & (tg < 31))):
        if sel.sum() > fs * 2:
            l2, c2 = xcorr_lead_ms(w_g[sel, ax], w_q[sel, ax], fs)
            print(f'  {name} part: {l2:+.2f} ms (c={c2:.3f}), max |rate| {np.abs(w_g[sel, ax]).max():.1f} rad/s')


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('id', type=int, nargs='?')
    ap.add_argument('--seconds', type=float, default=60.0)
    ap.add_argument('--out', default=None, help='save the capture (.npz)')
    ap.add_argument('--analyze', default=None, help='analyze a saved capture instead')
    args = ap.parse_args()
    if args.analyze:
        z = np.load(args.analyze, allow_pickle=True)
        meta = json.loads(str(z['meta']))
        analyze(z['t'], z['g'], z['q'], meta['before'], meta['after'])
        return
    if args.id is None:
        ap.error('id is required unless --analyze')
    t, g, q, before, after = asyncio.run(capture(args.id, args.seconds))
    if args.out:
        np.savez_compressed(args.out, t=t, g=g, q=q, meta=json.dumps(
            dict(id=args.id, before=before, after=after, cues=CUES), default=float))
        print(f'saved {args.out}')
    analyze(t, g, q, before, after)


if __name__ == '__main__':
    main()
