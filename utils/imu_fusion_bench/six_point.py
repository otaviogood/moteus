"""Accelerometer zero-g offset and scale by ellipsoid fit (READ-ONLY on the bus; nothing persistent).

    PYTHONPATH=lib/python python3 utils/imu_fusion_bench/six_point.py <id> [--apply [--persist]] [--settle 3] [--avg 4]
    python3 utils/imu_fusion_bench/six_point.py --selftest

You do not need to know which sensor axis is which.  The script asks for poses in terms of the
physical board (flat, flipped over, standing on each edge) and works out from the readings which
sensor direction was up in each one.  Gravity has a constant magnitude, so the readings lie on an
ellipsoid centred at the bias with semi-axes equal to the scale factors; six or more well-spread
poses determine it without knowing any orientation (15 degrees off is fine).  After each pose it
shows which of the six sensor directions have been covered and, if something is still missing after
the scripted poses, says what to do with the board next.  Reports the bias per axis (mg), the scale,
the fit residual and leave-one-out stability (quality checks), a sensor-axis map for the board, and
the tilt error the bias would cause; --apply loads the result into the board's `imu_cal` config
group (RAM), --persist also saves it to flash (`conf write`).
"""
import argparse, asyncio, math, sys, time
import numpy as np

# Physical poses; no sensor-axis knowledge needed.  Together they put each sensor axis up and down
# provided the chip is mounted square to the board (which the coverage check verifies).
POSES = [
    'lay the board FLAT on the table, component side UP',
    'flip it over: FLAT, component side DOWN',
    'stand it on one of its LONG edges',
    'stand it on the OPPOSITE long edge',
    'stand it on one of its SHORT edges',
    'stand it on the OPPOSITE short edge',
    # Two tilted poses give the fit redundancy: with exactly six poses it has no residual at all.
    'lean it against something at about 45 degrees, component side up',
    'lean it at about 45 degrees on a different edge (or stand it on a corner)',
]
NUM_AXIS_POSES = 6     # the first six prompts; used for the axis map
COVER_G = 0.6          # a direction counts as covered once some pose has > 0.6 g along it (~53 deg off-axis)
MOTION_MG = 2.0        # spread of the averaged samples above this means the board moved
AXES = 'XYZ'


def fit_ellipsoid(a):
    """a: (N,3) accelerometer readings in g -> (bias[3], scale[3], residual_rms_mg, ok)."""
    a = np.asarray(a, dtype=np.float64)
    # Axis-aligned ellipsoid: A x^2 + B y^2 + C z^2 + D x + E y + F z = 1 (linear in A..F).
    M = np.column_stack([a[:, 0]**2, a[:, 1]**2, a[:, 2]**2, a[:, 0], a[:, 1], a[:, 2]])
    coef, *_ = np.linalg.lstsq(M, np.ones(len(a)), rcond=None)
    A, B, C, D, E, F = coef
    if min(A, B, C) <= 0:
        return None, None, None, False
    bias = np.array([-D / (2 * A), -E / (2 * B), -F / (2 * C)])
    # Radius after completing the squares.
    k = 1 + A * bias[0]**2 + B * bias[1]**2 + C * bias[2]**2
    scale = np.sqrt(k / np.array([A, B, C]))
    r = np.linalg.norm((a - bias) / scale, axis=1) - 1.0
    return bias, scale, float(np.sqrt(np.mean(r**2)) * 1000.0), True


def coverage(meas):
    """-> (3,2) array: best +g and best -g seen along each sensor axis."""
    best = np.zeros((3, 2))
    if len(meas):
        a = np.asarray(meas, dtype=np.float64)
        best[:, 0] = np.maximum(a.max(axis=0), 0.0)
        best[:, 1] = np.maximum(-a.min(axis=0), 0.0)
    return best


def missing(meas):
    """Directions (axis index, sign index 0:+ 1:-) not yet covered."""
    best = coverage(meas)
    return [(ax, sg) for ax in range(3) for sg in (0, 1) if best[ax, sg] < COVER_G]


def dir_name(ax, sg):
    return f'{"+-"[sg]}{AXES[ax]}'


def describe_up(a):
    """'sensor +Y (5.3 deg off)' for the dominant direction of reading a."""
    a = np.asarray(a, dtype=np.float64); u = a / np.linalg.norm(a)
    ax = int(np.argmax(np.abs(u))); sg = 0 if u[ax] > 0 else 1
    off = math.degrees(math.acos(min(1.0, abs(u[ax]))))
    return f'sensor {dir_name(ax, sg)} ({off:.1f} deg off-axis)'


def hint(meas, names):
    """Physical instruction for the first missing direction, based on the poses already captured."""
    miss = missing(meas)
    if not miss:
        return None
    ax, sg = miss[0]; a = np.asarray(meas, dtype=np.float64)
    opp = a[:, ax] if sg == 1 else -a[:, ax]          # component along the opposite direction
    k = int(np.argmax(opp))
    if opp[k] >= COVER_G:
        return (f'still need gravity along sensor {dir_name(ax, sg)}: go back to pose {k + 1} '
                f'("{names[k]}") and turn the board UPSIDE DOWN from there (what was up goes down).')
    return (f'still need gravity along sensor {AXES[ax]} (either sign): no pose has had that axis near '
            f'vertical, so the chip is not square to the board edges.  Tilt the board about 45 degrees '
            f'from a flat pose toward an unused edge, or stand it on a corner, and try again.')


def leave_one_out_mg(meas):
    """(largest change in any bias component when one REDUNDANT pose is dropped [mg], number of
    redundant poses).  A pose is essential if dropping it uncovers a direction; those are skipped."""
    b0, *_ = fit_ellipsoid(meas)
    worst = 0.0; redundant = 0
    for i in range(len(meas)):
        rest = [m for j, m in enumerate(meas) if j != i]
        if missing(rest):
            continue
        b, _, _, ok = fit_ellipsoid(rest)
        if not ok:
            continue
        redundant += 1
        worst = max(worst, float(np.max(np.abs(b - b0))) * 1000.0)
    return worst, redundant


def print_coverage(meas):
    best = coverage(meas)
    cells = [f'{AXES[ax]}: +{best[ax, 0]:.2f}/-{best[ax, 1]:.2f}' for ax in range(3)]
    left = [dir_name(ax, sg) for ax, sg in missing(meas)]
    tail = 'all six directions covered' if not left else 'missing ' + ' '.join(left)
    print(f'  coverage (best g seen)  {"   ".join(cells)}   -> {tail}')


def selftest():
    rng = np.random.default_rng(1)
    true_b = np.array([0.009, -0.004, 0.012]); true_s = np.array([1.004, 0.997, 1.002])
    axes = np.array([[1,0,0],[-1,0,0],[0,1,0],[0,-1,0],[0,0,1],[0,0,-1]], dtype=float)
    worst = 0.0
    for trial in range(200):
        ups = []
        for ax in axes:
            # Up to 15 deg of random misalignment per pose.
            v = ax + rng.normal(0, 0.15, 3); v /= np.linalg.norm(v); ups.append(v)
        for _ in range(rng.integers(0, 5)):  # optional extra random poses
            v = rng.normal(size=3); v /= np.linalg.norm(v); ups.append(v)
        meas = [u * true_s + true_b + rng.normal(0, 0.0003, 3) for u in ups]
        b, s, res, ok = fit_ellipsoid(meas)
        assert ok
        assert not missing(meas), coverage(meas)
        worst = max(worst, float(np.max(np.abs(b - true_b))) * 1000)
    print(f'selftest: 200 trials, 6-10 poses up to ~15 deg off-axis, 0.3 mg noise: worst bias error {worst:.2f} mg')
    # Guidance: five poses, -Z never seen -> hint points at the +Z pose and says flip it.
    five = [axes[i] * true_s + true_b for i in range(5)]
    assert missing(five) == [(2, 1)], missing(five)
    h = hint(five, POSES); assert 'pose 5' in h and 'UPSIDE DOWN' in h, h
    # Chip rotated 45 deg in the board plane: edges give 0.7/0.7 splits, flat poses cover Z only.
    rot = [np.array([0, 0, 1.]), np.array([0, 0, -1.]), np.array([.7, .7, 0]), np.array([-.7, -.7, 0])]
    assert (0, 1) in missing(rot) or (1, 1) in missing(rot) or not missing(rot)
    assert 'square' in hint([np.array([0, 0, 1.]), np.array([0, 0, -1.])], POSES)
    assert describe_up([0.02, -0.98, 0.1]).startswith('sensor -Y')
    six = [axes[i] * true_s + true_b for i in range(6)]
    assert leave_one_out_mg(six)[1] == 0                      # every pose essential
    tilt = np.array([1., 1., 1.]) / math.sqrt(3.0)                  # unit vector, no axis over 0.6 g
    w, n = leave_one_out_mg(six + [tilt * true_s + true_b])
    assert n == 1 and w < 1.0, (w, n)                          # one redundant pose, stable
    print('selftest: coverage / hint / axis naming OK')
    return 0 if worst < 1.0 else 1


async def capture(s, settle, avg):
    """Wait for the low-pass to settle, then average accel_raw_g -> (mean g, spread mg, n)."""
    await asyncio.sleep(settle)
    samples = []; t_end = time.monotonic() + avg
    while time.monotonic() < t_end:
        f = await asyncio.wait_for(s.read_data('aux2_fusion'), 8.0)
        samples.append(np.asarray(f.accel_raw_g, dtype=np.float64))
        await asyncio.sleep(0.1)
    a = np.asarray(samples)
    return a.mean(axis=0), float(np.max(a.std(axis=0))) * 1000.0, len(samples)


async def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('id', type=int, nargs='?')
    ap.add_argument('--apply', action='store_true', help='conf set the imu_cal values on the board (RAM)')
    ap.add_argument('--persist', action='store_true', help='with --apply: conf write afterwards')
    ap.add_argument('--selftest', action='store_true')
    ap.add_argument('--settle', type=float, default=3.0, help='seconds to wait after Enter (the low-pass is ~1 s)')
    ap.add_argument('--avg', type=float, default=4.0, help='seconds to average per pose')
    ap.add_argument('--poses', type=int, default=len(POSES),
                    help='minimum number of poses before fitting (six is the bare minimum, more adds checks)')
    args = ap.parse_args()
    if args.selftest:
        sys.exit(selftest())
    import moteus
    c = moteus.Controller(id=args.id); s = moteus.Stream(c)
    await asyncio.wait_for(s.flush_read(), 2.0)
    await asyncio.wait_for(s.command(b'tel stop'), 5.0)
    await asyncio.wait_for(s.flush_read(), 2.0)
    # accel_raw_g is the uncorrected accelerometer, so the current imu_cal does not affect the measurement.
    print('The board must be still while each pose is measured (about %.0f s after Enter).' % (args.settle + args.avg))
    print("Answer 'r' to redo the last pose if it moved, 'd' to stop early and fit what you have.")
    meas = []; names = []
    while True:
        i = len(meas)
        if i < len(POSES):
            what = POSES[i]
        else:
            h = hint(meas, names)
            if h is None and i >= args.poses:
                break
            what = (h or 'extra pose: any steady tilt, edge or corner not used yet') if i >= len(POSES) else POSES[i]
        ans = input(f'\nPose {i + 1}: {what}\n  hold still and press Enter... ').strip().lower()
        if ans.startswith('d'):
            break
        if ans.startswith('r') and meas:
            meas.pop(); names.pop(); print('  last pose discarded'); continue
        a, spread, n = await capture(s, args.settle, args.avg)
        print(f'  accel = ({a[0]:+.4f}, {a[1]:+.4f}, {a[2]:+.4f}) g  |a| {np.linalg.norm(a):.4f}  '
              f'spread {spread:.1f} mg ({n} samples)  -> up was {describe_up(a)}')
        if spread > MOTION_MG:
            print(f'  WARNING: spread {spread:.1f} mg means the board moved during the average; '
                  f"answer 'r' at the next prompt to redo it")
        meas.append(a); names.append(what)
        print_coverage(meas)
        if len(meas) >= 6 and not missing(meas):
            b, sc, res, ok = fit_ellipsoid(meas)
            if ok:
                print(f'  running fit: bias ({b[0]*1000:+.1f}, {b[1]*1000:+.1f}, {b[2]*1000:+.1f}) mg, '
                      f'scale ({sc[0]:.4f}, {sc[1]:.4f}, {sc[2]:.4f}), residual {res:.2f} mg rms')
    if len(meas) < 6:
        print(f'\nonly {len(meas)} poses; need at least six for a fit'); return
    b, sc, res, ok = fit_ellipsoid(meas)
    if not ok:
        print('\nfit failed (poses not spread enough); rerun and add the poses it asks for'); return
    left = missing(meas)
    print('\nResult (accelerometer, sensor frame):')
    for i, ax in enumerate(AXES):
        print(f'  {ax}: bias {b[i]*1000:+7.2f} mg   scale {sc[i]:.4f}')
    print(f'  fit residual {res:.2f} mg rms over {len(meas)} poses (noise-limited is ~0.3; several mg means a '
          f'pose was not still, or cross-axis misalignment)')
    loo, nred = leave_one_out_mg(meas)
    if nred:
        print(f'  leave-one-out stability: bias moves at most {loo:.2f} mg when any one of the {nred} redundant '
              f'poses is dropped (under ~1 mg is a good fit)')
    else:
        print('  no redundant poses, so no stability check; rerun with more poses for one')
    if left:
        print(f'  WARNING: fitted without gravity along {", ".join(dir_name(*d) for d in left)}; '
              f'those bias/scale values are poorly determined')
    print(f'  tilt error from this bias, uncorrected: up to {math.degrees(math.asin(min(1.0, np.linalg.norm(b)))):.3f} deg '
          f'(depends on pose)')
    print('\nSensor axis map for this board (which sensor direction pointed up in each pose):')
    for k, (a, nm) in enumerate(zip(meas, names)):
        if k < NUM_AXIS_POSES:
            print(f'  pose {k + 1} ({nm}): up = {describe_up(a)}')
    cmds = [f'conf set imu_cal.accel_bias.{i} {b[i]:.6f}' for i in range(3)] + \
           [f'conf set imu_cal.accel_scale.{i} {sc[i]:.6f}' for i in range(3)]
    print('\nconfig (imu_cal group):')
    for cmd in cmds:
        print(f'  {cmd}')
    if args.apply:
        for cmd in cmds:
            await asyncio.wait_for(s.command(cmd.encode()), 5.0)
        print('applied to the board (RAM; takes effect immediately)')
        if args.persist:
            await asyncio.wait_for(s.command(b'conf write'), 20.0)
            print('persisted with conf write')
        else:
            print('not persisted: add --persist, or run `conf write` yourself')
    elif args.persist:
        print('--persist does nothing without --apply')

if __name__ == '__main__':
    asyncio.run(main())
