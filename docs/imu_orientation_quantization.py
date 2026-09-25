#!/usr/bin/env python3
"""Numerical backing for docs/imu_orientation_redesign.md.

Runs four self-contained analyses (numpy only, no hardware):

  1. Why the LSM6DSV16X SFLP "game vector" (fp16 x,y,z, w reconstructed)
     gives ~3 degree steps: geodesic error vs the quaternion's w.
  2. Error of candidate 48-bit orientation encodings that fit the existing
     three int16 registers 0x06d-0x06f.
  3. The chosen wire format ("smallest three", 3 x 15 bit + 2 bit index):
     reference encoder/decoder, round-trip self test, and fixed test
     vectors for firmware / host unit tests.
  4. Pure-gyro orientation integration error vs gyro sample rate, for an
     aggressive multi-axis limb motion (why 960 Hz is enough).

    python3 docs/imu_orientation_quantization.py [--fast]

--fast lowers the reference integration rate of analysis 4 (about 4x
quicker, numbers move in the 4th decimal).
"""
import argparse
import math

import numpy as np

SQRT2 = math.sqrt(2.0)
Q48_LEVELS = 32767          # 15-bit unsigned code range 0..32767
Q48_IDX_SHIFT = 45          # 2-bit index of the omitted component
Q48_TOGGLE_BIT = 47         # freshness toggle: flips on every valid reply that saw a newer sample
Q48_PAYLOAD_MASK = (1 << 47) - 1


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------

def rand_unit_quat(rng, n):
    q = rng.normal(size=(n, 4))
    return q / np.linalg.norm(q, axis=1, keepdims=True)


def rand_quat_with_w(rng, w, n):
    v = rng.normal(size=(n, 3))
    v /= np.linalg.norm(v, axis=1, keepdims=True)
    return np.concatenate([np.full((n, 1), w), v * math.sqrt(1.0 - w * w)], 1)


def ang_deg(qa, qb):
    """Geodesic angle between rotations (q and -q are the same rotation)."""
    d = np.abs((qa * qb).sum(-1)).clip(0.0, 1.0)
    return np.degrees(2.0 * np.arccos(d))


# ---------------------------------------------------------------------------
# 1. the current wire format: fp16 x,y,z with w = sqrt(1 - |xyz|^2)
# ---------------------------------------------------------------------------

def sflp_wire(q):
    """What the Orin reconstructs from the fp16 x,y,z wire triplet.

    Mirrors ST's sflp2q / orin/imu_common.py:decode_game_quat_xyz: if the
    rounded triplet overshoots the unit sphere it is renormalized before w is
    recovered, and the result is a unit quaternion (so the geodesic error
    below is meaningful; an un-normalized output would hide errors).
    """
    w = q[..., :1]
    xyz = np.where(w < 0, -q[..., 1:], q[..., 1:])
    xyz = xyz.astype(np.float32).astype(np.float16).astype(np.float64)
    v2 = (xyz ** 2).sum(-1, keepdims=True)
    xyz = np.where(v2 > 1.0, xyz / np.sqrt(np.maximum(v2, 1e-30)), xyz)
    v2 = np.minimum((xyz ** 2).sum(-1, keepdims=True), 1.0)
    out = np.concatenate([np.sqrt(1.0 - v2), xyz], -1)
    return out / np.linalg.norm(out, axis=-1, keepdims=True)


def analysis_fp16(rng):
    print("1. SFLP game vector (fp16 xyz, w reconstructed): error vs w\n")
    print("   w=cos(theta/2)  theta_from_ref   error p50 / p99 / max [deg]   first step off w=0")
    for w in (0.9, 0.7, 0.5, 0.3, 0.2, 0.1, 0.05, 0.02, 0.0):
        q = rand_quat_with_w(rng, w, 200000)
        e = ang_deg(q, sflp_wire(q))
        theta = math.degrees(2.0 * math.acos(w))
        # smallest |xyz| change fp16 can express near 1 is 2^-11; the w it implies:
        step = 2.0 * math.asin(math.sqrt(2.0 * 2 ** -11 - w * w)) if w * w < 2 * 2 ** -11 else 0.0
        print(f"   {w:5.2f}          {theta:6.1f}       {np.percentile(e, 50):7.3f} "
              f"{np.percentile(e, 99):7.3f} {e.max():7.3f}        {math.degrees(step):5.2f}")
    print("\n   w = cos(yaw/2) * cos(tilt/2): a board mounted Z-down (tilt 180) has w = 0 at EVERY yaw;")
    print("   any other board reaches w = 0 when its yaw is 180 deg from the power-on heading.\n")


# ---------------------------------------------------------------------------
# 2. candidate 48-bit encodings
# ---------------------------------------------------------------------------

def rt_fixed12(q):
    s = 2047.0
    qq = np.round(q * s) / s
    return qq / np.linalg.norm(qq, axis=1, keepdims=True)


def rt_smallest_three(q, bits=15):
    q = q.copy()
    i = np.argmax(np.abs(q), axis=1)
    sign = np.sign(q[np.arange(len(q)), i])
    sign[sign == 0] = 1
    q *= sign[:, None]
    mask = np.ones_like(q, dtype=bool)
    mask[np.arange(len(q)), i] = False
    others = q[mask].reshape(-1, 3)
    levels = 2 ** bits - 1
    code = np.round((others * SQRT2 + 1.0) / 2.0 * levels)
    dec = (code / levels * 2.0 - 1.0) / SQRT2
    w = np.sqrt(np.clip(1.0 - (dec ** 2).sum(1, keepdims=True), 0.0, 1.0))
    out = np.zeros_like(q)
    out[mask] = dec.ravel()
    out[np.arange(len(q)), i] = w[:, 0]
    return out / np.linalg.norm(out, axis=1, keepdims=True)


def rt_rotation_vector(q):
    q = np.where(q[:, :1] < 0, -q, q)
    v = q[:, 1:]
    s = np.linalg.norm(v, axis=1, keepdims=True)
    th = 2.0 * np.arctan2(s, q[:, :1])
    r = v / np.maximum(s, 1e-12) * th
    r2 = np.round(r / math.pi * 32767) / 32767 * math.pi
    th2 = np.linalg.norm(r2, axis=1, keepdims=True)
    ax = r2 / np.maximum(th2, 1e-12)
    return np.concatenate([np.cos(th2 / 2), ax * np.sin(th2 / 2)], 1)


def analysis_encodings(rng):
    print("2. 48-bit encodings in three int16 registers: geodesic error, 400k random orientations\n"
          "   (p50 / p99 / sampled max; see analysis 3 for the proven bound of the chosen format)\n")
    q = rand_unit_quat(rng, 400000)
    rows = [("12:12:12:12 fixed point, renormalize", rt_fixed12),
            ("smallest-three 3x15 bit + 2 bit index", rt_smallest_three),
            ("smallest-three 3x14 bit + index", lambda q: rt_smallest_three(q, 14)),
            ("rotation vector 3x16 bit", rt_rotation_vector)]
    for name, f in rows:
        e = ang_deg(q, f(q))
        print(f"   {name:40s} p50 {np.percentile(e, 50):.4f}  p99 {np.percentile(e, 99):.4f}  max {e.max():.4f} deg")
    q0 = rand_quat_with_w(rng, 0.0, 400000)
    e = ang_deg(q0, sflp_wire(q0))
    print(f"   {'current fp16 xyz, worst case (w = 0)':40s} p50 {np.percentile(e, 50):.4f}  "
          f"p99 {np.percentile(e, 99):.4f}  max {e.max():.4f} deg\n")


# ---------------------------------------------------------------------------
# 3. the chosen wire format: reference implementation + test vectors
# ---------------------------------------------------------------------------

def encode_quat48(q, toggle=0):
    """Unit quaternion (w, x, y, z) -> three uint16 register words (0x06d, 0x06e, 0x06f).

    Smallest-three: the component with the largest magnitude is omitted (the
    whole quaternion is negated first if that component is negative, which
    leaves the rotation unchanged).  The other three, in ascending index
    order, each lie in [-1/sqrt(2), +1/sqrt(2)] and are stored as unsigned
    15-bit codes.  Bit layout of the 48-bit little-endian value:
        bits  0-14  code of the first  remaining component
        bits 15-29  code of the second remaining component
        bits 30-44  code of the third  remaining component
        bits 45-46  index (0..3) of the omitted component
        bit  47     freshness toggle (firmware flips it on each valid reply that
                    used a newer sample than the previous valid reply)
    Low 47 bits all zero never occurs for a real quaternion; that is the
    "no data" sentinel (the firmware sends all 48 bits zero).
    """
    q = np.asarray(q, dtype=np.float64)
    q = q / np.linalg.norm(q)
    i = int(np.argmax(np.abs(q)))      # ties -> lowest index
    if q[i] < 0:
        q = -q
    others = [q[k] for k in range(4) if k != i]
    v = 0
    for j, x in enumerate(others):
        u = max(-1.0, min(1.0, x * SQRT2))
        code = int(round((u + 1.0) / 2.0 * Q48_LEVELS))
        v |= code << (15 * j)
    v |= i << Q48_IDX_SHIFT
    v |= (int(toggle) & 1) << Q48_TOGGLE_BIT
    return (v & 0xFFFF, (v >> 16) & 0xFFFF, (v >> 32) & 0xFFFF)


def decode_quat48(words):
    """Three uint16 register words -> unit quaternion (w, x, y, z), or None for the sentinel."""
    w0, w1, w2 = (int(w) & 0xFFFF for w in words)
    v = (w0 | (w1 << 16) | (w2 << 32)) & Q48_PAYLOAD_MASK   # bit 47 is the toggle, not payload
    if v == 0:
        return None
    i = (v >> Q48_IDX_SHIFT) & 0x3
    others = []
    for j in range(3):
        code = (v >> (15 * j)) & 0x7FFF
        others.append((code / Q48_LEVELS * 2.0 - 1.0) / SQRT2)
    q = np.zeros(4)
    q[[k for k in range(4) if k != i]] = others
    q[i] = math.sqrt(max(0.0, 1.0 - sum(x * x for x in others)))
    return q / np.linalg.norm(q)


def toggle_quat48(words):
    """Freshness toggle bit of three register words (bit 15 of 0x06f)."""
    return (int(words[2]) >> 15) & 1


def smallest_three_bound_deg(bits=15):
    """Rigorous (finite-perturbation) worst-case rotation error of the
    smallest-three format, in ideal real arithmetic.

    Let q be the true unit quaternion, q_i its omitted (largest-magnitude,
    made positive) component, so q_i >= 1/2 and the other three have norm
    |q_o| = sqrt(1 - q_i^2) <= sqrt(3/4).

    1. Stored components: each rounding error |d_j| <= s/2 with
       s = sqrt(2)/(2^bits - 1), so the error vector of the three stored
       components has norm |d| <= sqrt(3) * s / 2.
    2. Reconstruction: r = sum q_j^2 (true), r' = sum (q_j + d_j)^2 (decoded);
       |r' - r| <= 2 |q_o| |d| + |d|^2 =: D.
       q_i' = sqrt(1 - r') and q_i = sqrt(1 - r), so
       |q_i' - q_i| = |r' - r| / (q_i' + q_i) <= D / (sqrt(1/4 - D) + 1/2).
    3. The decoded 4-vector is q' = q + e with |e| <= sqrt(|d|^2 + |q_i'-q_i|^2).
       After normalisation the angle phi between q and q'/|q'| obeys
       sin(phi) = |e_perp| / |q'| <= |e| / (1 - |e|), and the rotation angle
       between the two unit quaternions is exactly 2 * phi (cos(theta/2) =
       |q . q'/|q'||).  Hence theta <= 2 asin(|e| / (1 - |e|)).
    (Note: for a chord distance c between unit quaternions the rotation angle
    is 4 asin(c/2) >= 2c, so "angle <= 2 x chord" would be the wrong way
    round; the sine form above is the correct direction.)
    Float32 firmware arithmetic adds ~1e-7 relative error, negligible here,
    but the firmware unit test must check its own implementation against
    this bound rather than assume it.
    """
    s = SQRT2 / (2 ** bits - 1)
    d = math.sqrt(3.0) * s / 2.0
    q_o = math.sqrt(0.75)
    big_d = 2.0 * q_o * d + d * d
    d_omitted = big_d / (math.sqrt(max(0.0, 0.25 - big_d)) + 0.5)
    e = math.hypot(d, d_omitted)
    return math.degrees(2.0 * math.asin(e / (1.0 - e)))


def analysis_wire_format(rng):
    print("3. Chosen wire format: smallest-three, 3 x 15 bit + 2 bit index (bit 47 = freshness toggle)\n")
    q = rand_unit_quat(rng, 20000)
    e = np.array([ang_deg(qi, decode_quat48(encode_quat48(qi))) for qi in q])
    bound = smallest_three_bound_deg()
    assert e.max() < bound, (e.max(), bound)
    assert decode_quat48((0, 0, 0)) is None
    assert decode_quat48((0, 0, 0x8000)) is None            # sentinel is the low 47 bits
    qt = q[0]
    w0, w1 = encode_quat48(qt, 0), encode_quat48(qt, 1)
    assert toggle_quat48(w0) == 0 and toggle_quat48(w1) == 1
    assert w1[2] == (w0[2] | 0x8000) and w1[:2] == w0[:2]
    assert np.allclose(decode_quat48(w0), decode_quat48(w1))    # decoder masks the toggle
    print(f"   analytic worst-case bound: {bound:.7f} deg (rigorous finite-perturbation bound, ideal arithmetic)")
    print(f"   round trip on 20k random orientations: sampled max {e.max():.4f} deg, sentinel OK, toggle masked OK")
    print("   (the sampled maxima in analysis 2 are sampled, not bounds; the bound above is the")
    print("    acceptance figure for the firmware / host unit tests)")
    print("   test vectors (register words 0x06d, 0x06e, 0x06f as unsigned hex):")
    vectors = [
        ("identity", (1.0, 0.0, 0.0, 0.0)),
        ("90 deg about x", (math.cos(math.pi / 4), math.sin(math.pi / 4), 0.0, 0.0)),
        ("tie on largest (0.5,0.5,0.5,0.5)", (0.5, 0.5, 0.5, 0.5)),
        ("w = 0 (old format worst case)", (0.0, 0.6, 0.0, 0.8)),
        ("negative largest component", (-0.8, 0.36, -0.48, 0.0)),
        ("general", (0.2, -0.5, 0.7, -0.4)),
    ]
    for name, qv in vectors:
        qn = np.asarray(qv) / np.linalg.norm(qv)
        words = encode_quat48(qn)
        back = decode_quat48(words)
        print(f"     {name:36s} q={np.round(qn, 4).tolist()}  ->  "
              f"{words[0]:#06x} {words[1]:#06x} {words[2]:#06x}   err {ang_deg(qn, back):.4f} deg")
    print()


# ---------------------------------------------------------------------------
# 4. pure-gyro integration error vs sample rate
# ---------------------------------------------------------------------------

def limb_omega(t, vib):
    wx = 6 * np.sin(2 * np.pi * 1.3 * t) + 0.3 * np.sin(2 * np.pi * 9 * t)
    wy = 5 * np.sin(2 * np.pi * 2.1 * t + 1) + 0.2 * np.sin(2 * np.pi * 7 * t)
    wz = 3 * np.sin(2 * np.pi * 0.7 * t + 2) + 0.15 * np.sin(2 * np.pi * 11 * t)
    if vib:   # 280 Hz vibration, 0.2 rad/s amplitude (~1e-4 rad), coning-like
        wx = wx + 0.2 * np.sin(2 * np.pi * 280 * t)
        wy = wy + 0.2 * np.cos(2 * np.pi * 280 * t)
    return np.stack([wx, wy, wz], -1)


def qmul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
                     w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                     w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])


def integrate(ws, dt):
    """Exact exponential map per step; ws[k] is the rate used over step k."""
    q = np.array([1.0, 0.0, 0.0, 0.0])
    out = np.empty((len(ws) + 1, 4))
    out[0] = q
    norms = np.linalg.norm(ws, axis=1)
    for k, (w, n) in enumerate(zip(ws, norms)):
        th = n * dt
        ax = w / n if n > 0 else np.array([1.0, 0.0, 0.0])
        q = qmul(q, np.concatenate([[math.cos(th / 2)], math.sin(th / 2) * ax]))
        q /= np.linalg.norm(q)
        out[k + 1] = q
    return out


def analysis_gyro_rate(f_ref):
    print(f"4. Pure-gyro integration (no accel correction) vs gyro rate; reference {f_ref // 1000} kHz,\n"
          f"   3 s of motion, gyro quantized to 70 mdps/LSB (+-2000 dps full scale)\n")
    T = 3.0
    lsb = math.radians(0.070)
    for vib in (False, True):
        t_ref = np.arange(int(T * f_ref)) / f_ref
        q_ref = integrate(limb_omega(t_ref + 0.5 / f_ref, vib), 1.0 / f_ref)
        peak = np.linalg.norm(limb_omega(t_ref, vib), axis=1).max()
        print(f"   {'with' if vib else 'no'} 280 Hz vibration, peak |omega| {peak:.1f} rad/s")
        print("     rate     sample&hold max / final     trapezoid max / final   [deg]")
        for f in (240, 480, 960, 1920, 3840):
            n = int(T * f)
            t = np.arange(n + 1) / f
            ws = np.round(limb_omega(t, vib) / lsb) * lsb
            idx = (t * f_ref).astype(int).clip(0, len(q_ref) - 1)
            e_sh = ang_deg(integrate(ws[:-1], 1.0 / f), q_ref[idx])
            e_tr = ang_deg(integrate(0.5 * (ws[:-1] + ws[1:]), 1.0 / f), q_ref[idx])
            print(f"     {f:5d} Hz     {e_sh.max():6.3f} / {e_sh[-1]:6.3f}            "
                  f"{e_tr.max():6.3f} / {e_tr[-1]:6.3f}")
    print()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--fast", action="store_true", help="24 kHz reference in analysis 4 instead of 96 kHz")
    args = ap.parse_args()
    rng = np.random.default_rng(0)
    analysis_fp16(rng)
    analysis_encodings(rng)
    analysis_wire_format(rng)
    analysis_gyro_rate(24000 if args.fast else 96000)


if __name__ == "__main__":
    main()
