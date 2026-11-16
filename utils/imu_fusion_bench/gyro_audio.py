"""Gyro audio bench check (fw/gyro_audio.h): poll one board at the robot's
120 Hz, report what comes back and write it as a sound file.  READ-ONLY:
nothing is commanded.

Usage: PYTHONPATH=lib/python python3 utils/imu_fusion_bench/gyro_audio.py <id> [seconds]
           [--wav FILE] [--axis main|x|y|z] [--signal rate|accel] [--highpass HZ] [--speed N]
           [--block] [--save FILE]

Every tick sends the gyro audio request (66 02, 11-bit broadcast ID
0x07F) and collects the reply (11-bit ID 0x780 | id): three signed bytes
per 960 Hz gyro sample, the change of each gyro axis since the previous
sample (the angular acceleration), compressed.  With --block each tick
first sends the telemetry block request (60 02), like the robot's tick,
and the block replies are counted too.

Reports: replies per request, samples per reply, the reply's frame
length, whether the sample numbers are continuous, the sample rate and
the level per axis in raw counts (1 count = 70 mdps).

--wav writes a mono 16-bit sound file of one direction: "main" is the
direction the board shook most in, x/y/z a gyro axis.  --signal rate
(default) is the gyro's angular velocity, rebuilt as the running sum of
the changes; accel is the change per sample, which sounds brighter.
--highpass HZ (default 30) drops slower motion; 0 keeps everything.  The
band ends at 470 Hz, so use headphones, or --speed 4 to play it four
times faster and two octaves higher.
--save writes the decoded samples to an .npz.

Tap the board or the motor, or turn the motor by hand, while it runs.
"""
import argparse
import asyncio
import time
import wave

import numpy as np

import moteus
from moteus.transport_device import Frame

TICK_S = 1.0 / 120.0
BROADCAST_ID = 0x7F
AUDIO_REQUEST = bytes([0x66, 0x02])
AUDIO_FORMAT = 0x02
AUDIO_REPLY_ID_BASE = 0x780
BLOCK_REQUEST = bytes([0x60, 0x02])
BLOCK_REPLY_ID_BASE = 0x700
COUNT_DPS = 0.070
UPSAMPLE = 8


def _decode_table():
    c = np.arange(128)
    m = np.where(c < 32, c, (16 + (c & 15)) << np.maximum((c >> 4) - 1, 0))
    table = np.zeros(256, dtype=np.int32)
    table[:128] = m
    table[129:] = -m[127:0:-1]      # bytes 129..255 are codes -127..-1
    table[128] = -m[127]
    return table


DECODE = _decode_table()            # indexed by the byte (uint8)


def frame(arbitration_id, data):
    f = Frame()
    f.arbitration_id = arbitration_id
    f.data = data
    f.is_extended_id = arbitration_id > 0x7FF
    f.is_fd = True
    f.bitrate_switch = True
    return f


def highpass(x, rate_hz, corner_hz):
    """x minus its moving average over about one period of corner_hz
    (0: x as it is)."""
    if corner_hz <= 0:
        return x
    n = max(3, int(round(rate_hz / corner_hz)) | 1)
    window = np.hanning(n + 2)[1:-1]
    window /= window.sum()
    pad = np.pad(x, ((n // 2, n // 2), (0, 0)), mode='edge')
    low = np.stack([np.convolve(pad[:, a], window, mode='valid')
                    for a in range(x.shape[1])], axis=1)
    return x - low


def upsample(x, factor=UPSAMPLE, lobes=8):
    """Band-limited interpolation of a 1-D signal to factor times the rate."""
    n = np.arange(-lobes * factor, lobes * factor + 1)
    h = np.sinc(n / factor) * np.hanning(len(n) + 2)[1:-1]
    y = np.zeros(len(x) * factor)
    y[::factor] = x
    return np.convolve(y, h, mode='same')


def main_axis(x):
    """The unit direction with the most energy."""
    _w, v = np.linalg.eigh(x.T @ x)
    axis = v[:, -1]
    return axis if axis[np.argmax(np.abs(axis))] > 0 else -axis


def write_wav(path, mono, rate_hz, speed):
    peak = float(np.max(np.abs(mono))) if len(mono) else 0.0
    scale = 0.9 * 32767.0 / peak if peak > 0 else 0.0
    pcm = np.clip(np.round(mono * scale), -32767, 32767).astype('<i2')
    with wave.open(path, 'wb') as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(int(round(rate_hz * speed)))
        w.writeframes(pcm.tobytes())
    return peak


async def main(args):
    dev_id = args.id
    c = moteus.Controller(id=dev_id)
    s = moteus.Stream(c)
    dev = moteus.get_singleton_transport()._devices[0]
    await asyncio.wait_for(s.flush_read(), 2.0)

    replies = []      # (host time, first sample number, bytes, frame length)
    blocks = 0
    other = 0

    async def collect():
        nonlocal blocks, other
        while True:
            f = await dev.receive_frame()
            if f.arbitration_id == (AUDIO_REPLY_ID_BASE | dev_id) and not f.is_extended_id:
                d = bytes(f.data)
                if len(d) >= 4 and d[0] == AUDIO_FORMAT and len(d) >= 4 + 3 * d[1]:
                    replies.append((time.monotonic(), d[2] | (d[3] << 8),
                                    d[4:4 + 3 * d[1]], len(d)))
                else:
                    other += 1
            elif f.arbitration_id == (BLOCK_REPLY_ID_BASE | dev_id):
                blocks += 1
            else:
                other += 1

    collector = asyncio.create_task(collect())
    audio_request = frame(BROADCAST_ID, AUDIO_REQUEST)
    block_request = frame(BROADCAST_ID, BLOCK_REQUEST)
    n = 0
    start = time.monotonic()
    next_t = start
    while time.monotonic() - start < args.seconds:
        if args.block:
            await dev.send_frame(block_request)
            await asyncio.sleep(0.0015)   # the robot asks after parsing the replies
        await dev.send_frame(audio_request)
        n += 1
        next_t += TICK_S
        await asyncio.sleep(max(0.0, next_t - time.monotonic()))
    await asyncio.sleep(0.05)
    collector.cancel()
    try:
        await collector
    except asyncio.CancelledError:
        pass

    print(f'{n} requests in {args.seconds:.1f} s; {len(replies)} audio replies'
          + (f', {blocks} telemetry block replies' if args.block else '')
          + (f', {other} other frames' if other else ''))
    if not replies:
        print('no audio reply: old firmware, no IMU fusion on this board, or a wrong id')
        return

    # The first reply carries the backlog since the previous poll.
    first = replies[0]
    print(f'first reply: {len(first[2]) // 3} samples (backlog), sample number {first[1]}')
    body = replies[1:]
    counts = {}
    lengths = {}
    lost = 0
    expected = (first[1] + len(first[2]) // 3) & 0xFFFF
    chunks = []
    newest = []       # (host time of a reply, samples up to and including it)
    total = 0
    for _t, seq, data, length in body:
        k = len(data) // 3
        counts[k] = counts.get(k, 0) + 1
        lengths[length] = lengths.get(length, 0) + 1
        gap = (seq - expected) & 0xFFFF
        lost += gap
        if gap:
            # Samples the host never got read as no change.
            chunks.append(np.zeros((gap, 3), dtype=np.int32))
        chunks.append(DECODE[np.frombuffer(data, dtype=np.uint8)].reshape(-1, 3))
        total += gap + k
        newest.append((_t, total))
        expected = (seq + k) & 0xFFFF
    print('samples per reply: ' + ', '.join(f'{k}: {v}' for k, v in sorted(counts.items())))
    print('reply frame length (bytes): '
          + ', '.join(f'{k}: {v}' for k, v in sorted(lengths.items())))
    if len(body) < 2 or total == 0:
        return
    accel = np.concatenate(chunks).astype(np.float64)   # change per sample
    # A line through the replies, without the first ones (the backlog).
    fit = np.asarray(newest[8:] if len(newest) > 16 else newest)
    rate = float(np.polyfit(fit[:, 0] - fit[0, 0], fit[:, 1], 1)[0])
    if not rate > 0:
        rate = 960.0
    print(f'{total} samples, {lost} lost between replies, {rate:.1f} samples/s')

    size = np.linalg.norm(accel, axis=1)
    limit = int((np.abs(accel) >= 1984).sum())
    print(f'change per sample, raw counts (1 count = {COUNT_DPS} deg/s): rms '
          + ', '.join(f'{a} {v:.2f}' for a, v in zip('xyz', np.sqrt((accel ** 2).mean(axis=0))))
          + f'; size p50 {np.percentile(size, 50):.1f}, p99 {np.percentile(size, 99):.1f}, '
          f'p99.9 {np.percentile(size, 99.9):.1f}, max {size.max():.1f}'
          + (f'; {limit} values at the +-1984 limit' if limit else ''))
    loud = np.argsort(size)[-8:][::-1]
    print('loudest samples (s after the first: counts): '
          + ', '.join(f'{k / rate:.3f}: {size[k]:.0f}' for k in loud))

    if args.save:
        np.savez(args.save, change=accel.astype(np.int32), rate_hz=rate)
        print(f'saved {args.save}')

    if args.wav:
        if args.signal == 'rate':
            signal = np.cumsum(accel, axis=0)
        else:
            signal = accel
        signal = highpass(signal, rate, args.highpass)
        if args.axis == 'main':
            axis = main_axis(signal)
        else:
            axis = np.eye(3)['xyz'.index(args.axis)]
        mono = upsample(signal @ axis)
        peak = write_wav(args.wav, mono, rate * UPSAMPLE, args.speed)
        print(f'wrote {args.wav}: {len(mono) / (rate * UPSAMPLE * args.speed):.1f} s, '
              f'{args.signal} along ({axis[0]:+.2f}, {axis[1]:+.2f}, {axis[2]:+.2f}), '
              f'full scale = {peak / 0.9:.1f} counts'
              + (f', played {args.speed:g}x faster' if args.speed != 1 else ''))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('id', type=int)
    parser.add_argument('seconds', type=float, nargs='?', default=5.0)
    parser.add_argument('--block', action='store_true')
    parser.add_argument('--save', default='')
    parser.add_argument('--wav', default='')
    parser.add_argument('--axis', choices=['main', 'x', 'y', 'z'], default='main')
    parser.add_argument('--signal', choices=['rate', 'accel'], default='rate')
    parser.add_argument('--highpass', type=float, default=30.0,
                        help='remove motion slower than this, Hz; 0 keeps everything')
    parser.add_argument('--speed', type=float, default=1.0)
    asyncio.run(main(parser.parse_args()))
