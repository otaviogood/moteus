"""docs §7 test 3: `conf write` while the fusion runs (persists the current RAM config)."""
import asyncio, sys, time, moteus
async def main(dev_id):
    c = moteus.Controller(id=dev_id); s = moteus.Stream(c)
    await asyncio.wait_for(s.flush_read(), 2.0)
    await asyncio.wait_for(s.command(b'tel stop'), 5.0)
    await asyncio.wait_for(s.flush_read(), 2.0)
    f0 = await asyncio.wait_for(s.read_data('aux2_fusion'), 8.0)
    ss0 = await asyncio.wait_for(s.read_data('servo_stats'), 8.0)
    t0 = time.monotonic()
    r = await asyncio.wait_for(s.command(b'conf write'), 10.0)
    dt = time.monotonic() - t0
    await asyncio.sleep(1.5)
    f1 = await asyncio.wait_for(s.read_data('aux2_fusion'), 8.0)
    ss1 = await asyncio.wait_for(s.read_data('servo_stats'), 8.0)
    print(f'conf write round trip {dt*1e3:.0f} ms')
    for k in ('gyro_gaps', 'gap_slots', 'mailbox_overflow', 'fifo_overruns', 'resyncs',
              'reinits', 'i2c_errors', 'stall_passes', 'arrival_unknown', 'sentinel_replies'):
        print(f'  {k}: {getattr(f0, k)} -> {getattr(f1, k)}')
    print(f'  mailbox_max_depth {f1.mailbox_max_depth}, init_state {f1.init_state}, converged {f1.converged}')
    print(f'  servo mode {ss1.mode} fault {ss1.fault}, isr_max_cycles {ss0.isr_max_cycles} -> {ss1.isr_max_cycles}')
    r = await asyncio.wait_for(s.command(b'conf get aux2.i2c.devices.0.type', allow_any_response=True), 5.0)
    print('  persisted type:', r.decode().strip())
asyncio.run(main(int(sys.argv[1])))
