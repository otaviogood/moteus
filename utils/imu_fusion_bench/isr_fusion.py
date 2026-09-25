"""ISR timing with the fusion running: zero-torque position mode on the BENCH motor (docs §7 test 2).
Sends `d pos nan 0 0 t4` (no torque, 4 s timeout) and always `d stop` in finally."""
import asyncio, sys, moteus
async def main(dev_id):
    c = moteus.Controller(id=dev_id); s = moteus.Stream(c)
    await asyncio.wait_for(s.flush_read(), 2.0)
    await asyncio.wait_for(s.command(b'tel stop'), 5.0)
    await asyncio.wait_for(s.flush_read(), 2.0)
    fu = await asyncio.wait_for(s.read_data('aux2_fusion'), 8.0)
    print(f'fusion init_state={fu.init_state} converged={fu.converged} gyro_words={fu.gyro_words}')
    for i in range(2):
        ss = await asyncio.wait_for(s.read_data('servo_stats'), 8.0)
        print(f'idle: mode={ss.mode} fault={ss.fault} isr_max_cycles={ss.isr_max_cycles} ({ss.isr_max_cycles/170:.1f} us) final_timer={ss.final_timer}/{ss.total_timer}')
        await asyncio.sleep(1.1)
    try:
        await asyncio.wait_for(s.command(b'd pos nan 0 0 t4'), 5.0)
        for i in range(3):
            await asyncio.sleep(1.1)
            ss = await asyncio.wait_for(s.read_data('servo_stats'), 8.0)
            print(f'position mode (zero torque): mode={ss.mode} fault={ss.fault} isr_max_cycles={ss.isr_max_cycles} ({ss.isr_max_cycles/170:.1f} us) final_timer={ss.final_timer}/{ss.total_timer}')
    finally:
        await asyncio.wait_for(s.command(b'd stop'), 5.0)
    await asyncio.sleep(0.3)
    ss = await asyncio.wait_for(s.read_data('servo_stats'), 8.0)
    fu = await asyncio.wait_for(s.read_data('aux2_fusion'), 8.0)
    print(f'after stop: mode={ss.mode} fault={ss.fault}; fusion gaps={fu.gyro_gaps} overflow={fu.mailbox_overflow} resyncs={fu.resyncs} max_depth={fu.mailbox_max_depth}')
asyncio.run(main(int(sys.argv[1])))
