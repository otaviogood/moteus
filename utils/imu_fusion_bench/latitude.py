"""THROW-AWAY: estimate latitude from Earth's rotation with the fusion IMU (READ-ONLY on the bus).

    PYTHONPATH=lib/python python3 utils/imu_fusion_bench/latitude.py <id> [--settle 45] [--min-measure 30]

Earth turns at 15.04 deg/h (4.2 mdps) about its axis: at latitude L that is Omega*sin(L) about the
local vertical and Omega*cos(L) about the horizontal north direction.  A gyro at rest reads its own
bias (about 0.5 dps on this chip, ~100x more) plus Earth's rate, and nothing separates the two while
the board stays put.  Turning it does: the bias turns with the sensor, Earth's rate stays fixed in
the room.  So the script asks you to set the board down at different headings (and optionally
upside down) and fits both.

Keep the desk quiet: vibration (typing, footsteps) pauses the firmware's bias learner, and the
script then counts only the time the learner runs, so poses take longer.

The fusion's orientation does not drift at rest because its bias learner absorbs every constant
rate, Earth's included; the learned bias IS the rate it removed.  Per still pose the script takes
the orientation from the quat48 registers (0x6d-0x6f) and averages the learned bias once the
learner has settled; then per pose  m = b + R^T w  (b: gyro bias, w: Earth's rate in the fusion's
world frame, R: body->world from the quaternion).

Two estimates are printed, refined after every pose:
  * horizontal: poses on the same face, different headings.  |w_horizontal| = Omega*cos(L).
    Immune to gyro g-sensitivity (gravity stays fixed in the sensor frame); cannot tell N from S.
  * full: needs the board flipped too; gives L = atan2(w_up, |w_horizontal|) and |w| as a check
    against 15.04 deg/h.  Flipping reverses gravity in the sensor frame, so any gyro g-sensitivity
    lands directly in w_up; disagreement between the two estimates measures it.
A linear bias drift term (temperature) is fitted once there are enough poses.

Nothing is sent to the board except telemetry reads and register queries.  Ctrl-C prints a final
summary.  Expect a few degrees after ~8 poses (~12 min); the limit is gyro bias stability.
"""
import argparse, asyncio, math, sys, time
import numpy as np
sys.path.insert(0, 'docs')
from imu_orientation_quantization import decode_quat48

OMEGA_E = 7.2921159e-5                 # rad/s
MDPS = 180.0 / math.pi * 1000.0        # rad/s -> mdps
DEG_H = 180.0 / math.pi * 3600.0       # rad/s -> deg/h
HEADINGS = [0, 180, 90, 270]           # suggested heading cycle per face (decorrelates drift)


def rot(q):
    """body -> world rotation matrix for q = (w, x, y, z); world z is up."""
    w, x, y, z = q
    return np.array([[1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y)],
                     [2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x)],
                     [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y)]])


def qangle_deg(a, b):
    return 2.0 * math.degrees(math.acos(min(1.0, abs(float(np.dot(a, b))))))


def face_name(up):
    ax = int(np.argmax(np.abs(up)))
    return ('+' if up[ax] > 0 else '-') + 'XYZ'[ax] + ' up'


def rel_yaw_deg(q, q0):
    Rr = rot(q) @ rot(q0).T
    return math.degrees(math.atan2(Rr[1, 0], Rr[0, 0])) % 360.0


class Pose:
    def __init__(self, q, t):
        self.q = np.asarray(q, float); self.t_start = t; self.samples = []; self.times = []
        self.stat_time = 0.0     # seconds with the bias learner running (settle + measure)
        self.meas_time = 0.0     # seconds of those spent measuring
        self.paused_time = 0.0   # seconds the learner was paused (vibration) while at rest
        self.R = rot(self.q); self.up = self.R[2, :].copy()

    def add(self, bias, t):
        self.samples.append(np.asarray(bias, float)); self.times.append(t)

    def seconds(self):
        return self.meas_time

    def mean(self):
        return np.mean(self.samples, axis=0)

    def t_mid(self):
        return 0.5 * (self.times[0] + self.times[-1])

    def half_split_mdps(self):
        n = len(self.samples) // 2
        if n < 2:
            return float('nan')
        return float(np.max(np.abs(np.mean(self.samples[:n], 0) - np.mean(self.samples[n:], 0)))) * MDPS


def faces_of(poses):
    """Cluster poses by the up vector in the sensor frame (same face if within 20 deg)."""
    refs = []; idx = []
    for p in poses:
        for i, r in enumerate(refs):
            if np.dot(r, p.up) > math.cos(math.radians(20)):
                idx.append(i); break
        else:
            refs.append(p.up); idx.append(len(refs) - 1)
    return idx, len(refs)


def fit(poses, model, drift):
    """Least squares.  model 'full': [b(3), w(3)]; 'horiz': [c_face(3 each), wx, wy].  -> dict or None."""
    fidx, nf = faces_of(poses)
    t0 = poses[0].t_mid()
    nb = 3 if model == 'full' else 3 * nf
    nw = 3 if model == 'full' else 2
    ncol = nb + nw + (3 if drift else 0)
    A = np.zeros((3 * len(poses), ncol)); y = np.zeros(3 * len(poses))
    for k, p in enumerate(poses):
        r = slice(3 * k, 3 * k + 3)
        off = 0 if model == 'full' else 3 * fidx[k]
        A[r, off:off + 3] = np.eye(3)
        for j in range(nw):
            A[r, nb + j] = p.R[j, :]            # R^T e_j
        if drift:
            A[r, nb + nw:nb + nw + 3] = np.eye(3) * ((p.t_mid() - t0) / 1000.0)
        y[r] = p.mean()
    An = A / np.maximum(np.linalg.norm(A, axis=0), 1e-12)
    sv = np.linalg.svd(An, compute_uv=False)
    if len(y) < ncol or sv[-1] < 1e-3 * sv[0]:
        return None
    x, *_ = np.linalg.lstsq(A, y, rcond=None)
    dof = len(y) - ncol
    res = y - A @ x
    out = {'x': x, 'dof': dof, 'nw_at': nb, 'faces': nf, 'n': len(poses), 'drift': drift, 'model': model}
    out['rms_mdps'] = float(np.sqrt(res @ res / max(dof, 1))) * MDPS if dof > 0 else float('nan')
    out['cov'] = (res @ res / dof) * np.linalg.inv(A.T @ A) if dof > 0 else None
    return out


def latitude_of(model, w):
    if model == 'full':
        return math.degrees(math.atan2(w[2], math.hypot(w[0], w[1])))
    return math.degrees(math.acos(min(1.0, math.hypot(w[0], w[1]) / OMEGA_E)))


def describe(f, rng):
    i = f['nw_at']; nw = 3 if f['model'] == 'full' else 2
    w = f['x'][i:i + nw]
    lat = latitude_of(f['model'], w)
    mag = float(np.linalg.norm(w))
    lat_sd = mag_sd = None
    if f['cov'] is not None:
        s = rng.multivariate_normal(f['x'], f['cov'], 3000, check_valid='ignore')[:, i:i + nw]
        lat_sd = float(np.std([latitude_of(f['model'], v) for v in s]))
        mag_sd = float(np.std(np.linalg.norm(s, axis=1)))
    pm = lambda v, sd, fmt: (fmt % v) + ('' if sd is None else ' +- ' + (fmt.replace('+', '') % sd))
    extra = f"{f['n']} poses, {f['faces']} face(s), fit rms {f['rms_mdps']:.3f} mdps" + (', drift term' if f['drift'] else '')
    if f['model'] == 'full':
        return (f"  full:       latitude {pm(lat, lat_sd, '%+.1f')} deg   |earth rate| {pm(mag * DEG_H, None if mag_sd is None else mag_sd * DEG_H, '%.2f')} deg/h "
                f"(true 15.04)   [{extra}]")
    return (f"  horizontal: latitude {pm(lat, lat_sd, '%.1f')} deg (N or S)   horizontal rate {pm(mag * DEG_H, None if mag_sd is None else mag_sd * DEG_H, '%.2f')} deg/h "
            f"= 15.04*cos(lat)   [{extra}]")


def report(poses, rng, drift_ok, header):
    print(header)
    shown = False
    for model in ('horiz', 'full'):
        f = None
        if drift_ok and len(poses) >= 5:
            f = fit(poses, model, True)
        if f is None:
            f = fit(poses, model, False)
        if f is not None:
            print(describe(f, rng)); shown = True
        elif model == 'full' and len(poses) >= 2:
            print('  full:       needs poses on a second face (flip the board over) to separate the vertical component')
    if not shown:
        print('  (need at least two headings on the same face)')


def suggestion(poses):
    fidx, nf = faces_of(poses)
    cur = len(fidx) - 1
    on_face = [p for p, i in zip(poses, fidx) if i == fidx[cur]]
    k = len(on_face)
    if nf == 1 and k >= 8:
        return 'flip the board over (other face down) and keep going; that enables the full estimate'
    target = HEADINGS[k % len(HEADINGS)]
    now = rel_yaw_deg(poses[-1].q, on_face[0].q)
    return f'turn it flat on the table to about {target} deg from this face\'s first pose (now at {now:.0f} deg), then leave it'


async def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('id', type=int)
    ap.add_argument('--settle', type=float, default=45.0, help='seconds after coming to rest before measuring (bias learner time constant ~10 s)')
    ap.add_argument('--min-measure', type=float, default=30.0, help='seconds of measurement before a pose counts')
    ap.add_argument('--poll', type=float, default=0.05, help='sleep between samples')
    ap.add_argument('--no-drift', action='store_true', help='do not fit a linear bias drift')
    ap.add_argument('--max-seconds', type=float, default=0.0, help='stop after this long (testing)')
    args = ap.parse_args()
    import moteus
    rng = np.random.default_rng(0)
    c = moteus.Controller(id=args.id); s = moteus.Stream(c)
    await asyncio.wait_for(s.flush_read(), 2.0)
    await asyncio.wait_for(s.command(b'tel stop'), 5.0)
    await asyncio.wait_for(s.flush_read(), 2.0)
    qr = moteus.QueryResolution()
    qr._extra = {0x06d: moteus.INT16, 0x06e: moteus.INT16, 0x06f: moteus.INT16}
    c_q = moteus.Controller(id=args.id, query_resolution=qr)

    print(f'Earth rate 15.04 deg/h = {OMEGA_E * MDPS:.2f} mdps.  Put the board flat on a table and leave it; '
          f'each pose takes ~{args.settle + args.min_measure:.0f} s (longer is better).')
    print('The script says when to turn it.  Handle it gently; Ctrl-C for a final summary.\n')
    poses = []; cur = None; state = 'moving'; t_start = time.monotonic(); t_prev = None
    last_status = 0.0; told_to_move = False; last_provisional = 0.0
    try:
        while True:
            now = time.monotonic()
            if args.max_seconds and now - t_start > args.max_seconds:
                break
            f = await asyncio.wait_for(s.read_data('aux2_fusion'), 8.0)
            r = await asyncio.wait_for(c_q.query(), 1.0)
            q = decode_quat48(tuple(int(r.values[g]) & 0xffff for g in (0x06d, 0x06e, 0x06f)))
            now = time.monotonic()
            if q is None:
                await asyncio.sleep(args.poll); continue
            q = np.asarray(q, float)
            dt = 0.0 if t_prev is None else min(1.0, now - t_prev)
            t_prev = now
            # A real move rotates the board by more than a degree.  The firmware's stationary flag
            # is not a motion detector here: desk vibration drops it for seconds at a time while
            # the board sits untouched; it only means the bias learner is paused.
            wmax = max(abs(v) for v in f.omega)
            if cur is None:
                moving = (not f.stationary) or wmax > 0.0175
            else:
                moving = qangle_deg(q, cur.q) > 1.0 or wmax > 0.0524
            if moving:
                if cur is not None and state == 'measuring' and cur.seconds() >= args.min_measure:
                    poses.append(cur)
                    p = cur; m = p.mean() * MDPS
                    fi, _ = faces_of(poses)
                    first = [pp for pp, i in zip(poses, fi) if i == fi[-1]][0]
                    print(f'\npose {len(poses)} stored: {face_name(p.up)}, heading {rel_yaw_deg(p.q, first.q):.0f} deg, '
                          f'{p.seconds():.0f} s measured ({p.paused_time:.0f} s paused by vibration), gyro mean ({m[0]:+.2f}, {m[1]:+.2f}, {m[2]:+.2f}) mdps, '
                          f'half-split {p.half_split_mdps():.3f} mdps')
                    report(poses, rng, not args.no_drift, f'estimate after {len(poses)} poses:')
                elif cur is not None and state != 'moving':
                    print(f'\n  (moved after {now - cur.t_start:.0f} s; pose too short, discarded)')
                if state != 'moving':
                    print('  moving...')
                cur = None; state = 'moving'; told_to_move = False
            else:
                if state == 'moving':
                    cur = Pose(q, now); state = 'settling'
                    print(f'  at rest ({face_name(cur.up)}); settling {args.settle:.0f} s')
                elif f.stationary:
                    cur.stat_time += dt
                else:
                    cur.paused_time += dt
                if state == 'settling' and cur.stat_time >= args.settle:
                    state = 'measuring'; print('  measuring')
                if state == 'measuring' and f.stationary:
                    cur.meas_time += dt
                    cur.add(f.bias, now)
                    if not told_to_move and cur.seconds() >= args.min_measure:
                        told_to_move = True
                        print(f'  pose {len(poses) + 1} has enough data (staying longer improves it).  Next: '
                              f'{suggestion(poses + [cur])}')
                    if cur.seconds() >= args.min_measure and now - last_provisional > 30.0 and len(poses) >= 1:
                        last_provisional = now
                        report(poses + [cur], rng, not args.no_drift, f'  provisional, including the current pose:')
            if now - last_status > 15.0 and cur is not None and state != 'moving':
                last_status = now
                el = now - cur.t_start
                print(f'  [{state} {el:.0f} s, learner paused {cur.paused_time:.0f} s by vibration]  learned bias ({f.bias[0] * MDPS:+.2f}, {f.bias[1] * MDPS:+.2f}, '
                      f'{f.bias[2] * MDPS:+.2f}) mdps')
            await asyncio.sleep(args.poll)
    except (KeyboardInterrupt, asyncio.CancelledError):
        pass
    final = poses + ([cur] if cur is not None and state == 'measuring' and cur.seconds() >= args.min_measure else [])
    if final:
        report(final, rng, not args.no_drift, f'\nfinal ({len(final)} poses):')
    else:
        print('\nno complete poses')

if __name__ == '__main__':
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
