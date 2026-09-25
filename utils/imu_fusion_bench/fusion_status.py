"""Print the aux2_fusion telemetry record (READ-ONLY)."""
import asyncio, sys, moteus
FIELDS = ['init_state', 'initialized', 'converged', 'timing_degraded', 'toggle', 'storage_owner',
          'last_reinit_reason', 'freq_fine', 'odr_actual_hz', 'gyro_words', 'accel_words',
          'gyro_gaps', 'gap_slots', 'mailbox_overflow', 'mailbox_max_depth',
          'fifo_overruns', 'resyncs', 'reinits', 'i2c_errors', 'stall_passes', 'arrival_unknown',
          'sentinel_replies', 'valid_replies', 'phase_unc_us',
          'q', 'bias', 'omega']
async def main(dev_id):
    c = moteus.Controller(id=dev_id); s = moteus.Stream(c)
    await asyncio.wait_for(s.flush_read(), 2.0)
    await asyncio.wait_for(s.command(b'tel stop'), 5.0)
    await asyncio.wait_for(s.flush_read(), 2.0)
    f = await asyncio.wait_for(s.read_data('aux2_fusion'), 8.0)
    for k in FIELDS:
        if hasattr(f, k):
            print(f'  {k} = {getattr(f, k)}')
    ss = await asyncio.wait_for(s.read_data('servo_stats'), 8.0)
    print(f'  servo mode={ss.mode} fault={ss.fault} isr_max_cycles={ss.isr_max_cycles}')
asyncio.run(main(int(sys.argv[1])))
