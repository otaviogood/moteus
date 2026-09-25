# How this fork differs from mjbots/moteus

This repository is Otavio Good's fork of
[mjbots/moteus](https://github.com/mjbots/moteus), used on the humanoid3
robot (host code in the separate `humanoid3` repository).  The robot has
two moteus boards per joint: a **motor board** that drives the motor, and a
**sensor board** (a moteus with no gate driver) on the output side of the
joint's spring.  Each board also reads an LSM6DSV16X IMU on its aux2 I2C
port.

The fork is based on upstream `main` at `b734185f` ("Add a utility to plot
hall effect calibration errors"), merged in `e3f0f1fc`.  Everything below
is relative to that commit.  To see the raw difference:

```
git fetch upstream
git diff upstream/main            # tracked files
git status --short                # plus new files
```

At the time of writing (2026-09-25) most of the IMU fusion, telemetry and
latency work below was not yet committed.

## Summary

| Area | What the fork adds or changes |
|---|---|
| Sensor boards | A board with no gate driver boots as a "sensor board": `otavio_flags` bit 0, its own LED pins, a filtered broadcast reply |
| IMU | LSM6DSV16X on aux I2C with on-board orientation fusion (I2C device type 3), a persistent `imu_cal` config group, and an `aux2_fusion` telemetry record |
| Registers | 0x065–0x067 gyro rate and 0x06d–0x06f quaternion (both fork-only) |
| CAN protocol | A fixed-layout "telemetry block" request (`0x60 0x01`); hardware receive timestamps |
| Upstream bug fix | Encoder PLL gains are now computed on boards with no motor configured |
| Latency / CPU | Non-blocking DRV8323 status poll on r4.x, IMU interrupt path in CCM, interrupt profiler, logic-analyzer markers |
| Build safety | The firmware image can no longer overlap the persistent config pages |
| Client libraries | Python and C++ register definitions and examples for the IMU registers |
| Tools and docs | Bench scripts, design and latency write-ups, protocol docs |

## 1. Sensor boards (no gate driver)

Files: `fw/moteus_hw.cc`, `fw/moteus_hw.h`, `fw/measured_hw_rev.h`,
`fw/moteus.cc`, `fw/firmware_info.cc`, `fw/board_debug.cc`, `fw/fdcan.cc`.

- **Detection.** Upstream's r4.x family detection loops forever waiting for
  the DRV8323 to answer.  The fork gives up after 10 attempts and marks the
  board as a sensor board with `g_otavio_flags = 1`.  The flag is exposed as
  `firmware.otavio_flags` in telemetry.
- **LEDs.** Sensor boards use different LED pins: red PF1, yellow PC13 and
  green PF0.
  - `d led` swaps the two LEDs and inverts their state on these boards,
    because of how those boards were built.
  - `FDCan::Send` lights the yellow LED for the duration of every CAN send.
    The pin is set up once, when `FDCan` is constructed.
- **Broadcast reply filter** (`fw/broadcast_reply_filter.h`).
  - In a broadcast request (CAN destination 0x7F), a sensor board answers
    only reads that touch 0x050–0x053 (encoder position and velocity) or
    0x065–0x06f (gyro and quaternion).
  - The filter runs in `FDCanMicroServer` before mjlib sees the frame.
  - Requests addressed to the board itself are never filtered, and motor
    boards are unaffected.
- **Debug output pin** (`fw/otavio.h`): `SendDebugf()` bit-bangs 115200-baud
  serial on the DBG2 pin for logic-analyzer printf debugging.  Nothing calls
  it by default.  `MoteusHwPins::otavio_pin` names PC11 on sensor boards for
  the same purpose.

## 2. IMU and on-board orientation fusion

Design, bench results and history: `docs/imu_orientation_redesign.md`.
Wire-format numbers: `docs/imu_orientation_quantization.py`.

**Configuration.** Set `aux2.i2c.devices.0.type 3` (`lsm6dsv16x`), address
106, and an I2C bus of at least 400 kHz.  Upstream has no type 3.
- `poll_rate_us` is ignored: the driver reads whenever the bus is free.
- Only one fusion device per board is allowed.
- The registers report only an **aux2** fusion.

**Firmware pieces** (all new):

| File | Role |
|---|---|
| `fw/lsm6dsv16x_fusion.h` | Interrupt-side driver: non-blocking chip init, then FIFO status/word reads, at most one I2C transaction per control cycle |
| `fw/imu_fusion.h` | Main-loop fusion: Mahony filter, orientation history, register replies |
| `fw/imu_fusion_storage.{h,cc}` | The single static fusion state block, and TIM3 ownership |
| `fw/quat48.h` | Quaternion wire encoding |
| `fw/imu_cal.h` | The `imu_cal` config group (accelerometer bias and scale per board) |

**Sampling.** The chip FIFO batches gyro samples at 960 Hz and
accelerometer samples at 120 Hz.  The chip is configured for ±2000 dps
with a 149 Hz LPF1, and ±4 g.

**Filter.** Mahony filtering with trapezoid integration.
- The accelerometer correction is gated on |a| and |ω|, and its innovation
  is clamped.
- The gyro bias is learned on all three axes while the board is
  stationary, with a fast capture right after boot.

**Timing.** TIM3 runs as a free-running 16-bit counter with 4 µs ticks.
- It stamps the FIFO words.
- FDCAN also uses it as its receive timestamp counter, so every CAN frame
  carries its arrival time.
- Pin modes that would reprogram TIM3 (hardware quadrature, BiSS-C, PWM
  output) are refused while the fusion runs.

**Replies.** A quaternion reply is the orientation at the moment the
request frame arrived.
- It is interpolated from the history, or extrapolated up to 8 ms.
- The console command `aux2 fusion latency <us>` shifts the evaluation
  time; it isn't persisted yet.

**Startup and recovery.**
- Replies are the "no data" sentinel until the filter converges: after
  0.2 s if the board starts still, otherwise 1 s.
- An I2C failure re-initializes the chip, waiting 4 ms between attempts.
- The fusion keeps its heading across a re-initialization.

**Chip settings beyond the datasheet defaults.** The I2C anti-spike filter
is locked on (IF_CFG.ASF_CTRL).  Otherwise, a glitch that looks like the I3C
broadcast address turns it off until the next power cycle.

**Telemetry and console.**
- Telemetry record: `aux2_fusion` (or `aux1_fusion`).  It is a separate
  record because appending the fields to `aux2` overflowed the 2 KB
  telemetry buffer and halted the board.
- Console: `aux2 fusion gains <kp> <ki> | latency <us> | halt <0|1> | drop <n> | stall <ms>`.
  These are bench hooks, applied at runtime only.
- Accelerometer calibration: `conf set imu_cal.accel_bias.N` /
  `imu_cal.accel_scale.N`, then `conf write`.
  `utils/imu_fusion_bench/six_point.py` measures the values.

**Compatibility notes.**
- **Stored config.** Before the fusion, type 3 meant the chip's own SFLP
  "game rotation vector" sent as fp16 x, y, z at 0x072–0x074.  A board whose
  stored config says type 3 runs the fusion after flashing this firmware,
  with no config change.
- **Register move.** 0x072–0x074 now belong to upstream's PWM inputs, which
  is why the quaternion moved to 0x06d–0x06f.
- **Sample config.** `configs/IMU_I2C.cfg` sets up the IMU on aux2
  (`moteus_tool --write-config configs/IMU_I2C.cfg`, then `conf write`).

## 3. Registers and CAN protocol

**New registers** (read-only; they sit in gaps of upstream's map, below
0x80, so each register number still fits in one byte):

| Register | Meaning | Types |
|---|---|---|
| 0x065–0x067 | Aux2 fusion gyro rate: bias-corrected, newest 960 Hz sample, rad/s, in the quaternion's body frame. NaN until converged or when stale | int8 0.1, int16 0.001, int32 1e-6, float 1 |
| 0x06d–0x06f | Aux2 quaternion, "smallest three" 48-bit format: three 15-bit components, the index of the omitted one, and bit 47 as a freshness toggle. All-zero low 47 bits is the "no data" sentinel | int16 raw words |

`docs/protocol/registers.md` documents both.  Read each triple in one
subframe (`0x17 0x65`, `0x17 0x6d`) so all three words come from one sample.

**Telemetry block** (`fw/telemetry_block.h`).
- **Request.** A request whose first two bytes are `0x60 0x01` gets a
  fixed-layout reply: one type byte, then that board's register values
  packed with no per-register headers.
- **Values.** Each value is exactly what a normal register read returns.

  | Type byte | Board | Contents | Size |
  |---|---|---|---|
  | `0x61` | Motor board | Quaternion, gyro rate, q current (int32), power (int16), motor temperature, bus voltage, board temperature, fault, mode | 24 bytes |
  | `0x62` | Sensor board | Encoder position and PLL velocity, quaternion, gyro rate | 17 bytes |

- **Piggybacked reads.** Ordinary reads that follow the marker are answered
  after the block, as many whole items as fit in 64 bytes.
- **Unknown versions** are ignored.
- **Why.** With ordinary reads the same values took a 48-byte motor frame
  and a 24-byte sensor frame; the block needs 24 and 20.
- **Implementation detail.** A block-only request hands mjlib a single NOP
  byte, because a 0-byte read makes the multi-transport layer re-process the
  previous frame.

**Receive timestamps.**
- `FDCan` enables the external timestamp counter and records each frame's
  start-of-frame stamp.
- `MultiTransportDatagramServer` reports whether the current frame came in
  over CAN, its stamp, and the RX FIFO fill level.
- The fusion uses these to evaluate replies at the frame's arrival time.

**Protocol docs.** `docs/protocol/can.md` documents the tunneled-stream
subframes (0x40–0x42).  `docs/protocol/diagnostic.md` documents `d led`.

## 4. Fix to upstream: encoder PLL on boards without a motor

`fw/motor_position.h`: upstream computes the PLL gains at the end of
`HandleConfigUpdate`, after the motor checks.
- On a board with `motor.poles 0` (every sensor board), the function
  returns early with `kMotorNotConfigured`, so the gains stay zero.
- With a nonzero `pll_filter_hz`, 0x050 then froze and 0x051 read 0.

The fork computes the gains right after the per-source config loop.  Test:
`MotorPositionPllWithoutMotor` in `fw/test/motor_position_test.cc`, which
fails without the fix.  This is a candidate to send upstream.

## 5. Latency and CPU changes

Measurements and reasoning: `docs/latency.md`.

- **DRV8323 status poll** (`fw/drv8323.cc`).
  - Upstream bit-bangs the gate-driver SPI with `wait_us(1)`, blocking the
    main loop 0.3–0.55 ms every 10 ms while the driver is enabled.
  - On r4.x (family 0) the fork uses the SPI1 peripheral instead, and splits
    the two status reads into one step per millisecond.
  - Other hardware families keep the bit-banged SPI.
  - Result: telemetry-reply p99 went from 760 to 372 µs, and command p99 from
    610 to 230 µs.
- **Fast RAM for the IMU interrupt path.** `AuxPort::ISR_I2C_Update`,
  `Stm32I2c::StartReadMemory` and the fusion driver's interrupt methods run
  from CCM.
  - To make room, upstream's debug-stream emitter body moved to flash
    (`ISR_EmitDebug`). It only runs with `servo.emit_debug` set.
  - Result: the control interrupt is about 2 µs shorter.
- **Interrupt profiler** (`fw/bldc_servo.cc`, `fw/bldc_servo_structs.h`,
  `fw/bldc_servo_control.h`).
  - In `MOTEUS_PERFORMANCE_MEASURE` builds, upstream's fixed `dwt.*` fields
    are replaced by `servo_stats.dwt.mean[]` and `.max[]` over 16 named
    points, latched once a second.
  - Every build now also reports `servo_stats.isr_max_cycles`, the longest
    control cycle over the last second.
- **Logic-analyzer markers** (`fw/scope_markers.h`, `-DMOTEUS_SCOPE_MARKERS`
  builds, r4.x only).
  - DBG1 shows the control interrupt.
  - DBG2 shows a phase marker, CAN frame handling, or fusion work, selected
    with `d mark phase|can|fusion`.
  - Without the define the markers compile to nothing.

## 6. Build, flash and tooling

- **Flash size guard.** `WORKSPACE` sets `MBED_APP_SIZE` to `0x006f000`
  (upstream `0x0070000`), so an image that would overlap the two
  persistent-config pages fails to link instead of erasing the calibration.
  Always check the margin before flashing.
- **Build config.**
  - `fw/BUILD` adds the new headers, sources and tests, plus a dependency on
    `mjlib/multiplex:format`.
  - `.bazelrc` gains commented-out debug-optimization flags only.
- **Option bytes.** `fw/program_option_bytes.sh` works with the newer
  OpenOCD, using `reset run` and `shutdown`.
- **Editor.** `.vscode/` and `fw/.vscode/` hold a Cortex-Debug launch config
  and a build task.
- **Firmware tests** (host, `tools/bazel test --config=host //fw:test`):
  `quat48_test`, `imu_fusion_test`, `imu_cal_test`,
  `broadcast_reply_filter_test` and `telemetry_block_test`, plus the new PLL
  case in `motor_position_test`.

## 7. Client libraries and scripts

- **Python** (`lib/python/moteus/protocol.py`).
  - Adds `AUX2_GYROX/Y/Z` and `AUX2_QUATERNIONX/Y/Z`.
  - Quaternion words are returned as raw integers, never scaled: a scaled
    int16 read would turn the valid word 0x8000 into NaN.
- **C++** (`lib/cpp/mjbots/moteus/`).
  - `kAux2GyroX/Y/Z` and `kAux2QuaternionX/Y/Z`, a `QuaternionRead`
    command, and `Controller::ReadQuaternion()` / `AsyncReadQuaternion()`.
  - The reply parser scales the gyro registers (new `kGyroRate` type) and
    returns quaternion words verbatim (new `kRawInt` type, no NaN mapping).
    Test: `QueryImuRegisters` in `test/moteus_protocol_test.cc`.
- **Examples.** `lib/python/examples/read_quaternion.py` and
  `lib/cpp/examples/read_quaternion.cc` decode and print the orientation.
- **`otavio_motor_check.py`** is a post-programming health check.
  - It checks that the IMU reports an orientation, the encoder is nonzero,
    and the FET and motor temperatures are 18–38 °C.
  - Use `--nomotor` for sensor boards.
- **`utils/imu_fusion_bench/`**: bench scripts for the fusion. It covers:
  - live status and cold-start timing;
  - persistence tests and the six-position accelerometer calibration;
  - the CAN and scope captures behind `docs/latency.md`.
- **`README.md`** has a short note on reading the IMU over an fdcanusb.

## 8. Documentation added

- `docs/imu_orientation_redesign.md`: the IMU fusion design, wire format,
  timing model, calibration plan and bench results.
- `docs/imu_orientation_quantization.py`: the numbers behind the quaternion
  format choice.
- `docs/latency.md`: where the robot's latency goes, what changed, and
  what's left.
- Edits to `docs/protocol/registers.md`, `can.md`, `diagnostic.md` and
  `docs/reference/configuration.md` / `encoders.md` for the items above.

## Files in the repository root that are not source

`DS_lsm6dsv16x.pdf` (the IMU datasheet), and `analog.csv` / `digital.csv`
(logic-analyzer captures of an aux2 I2C bus). They are working material
and are untracked.

## Keeping up with upstream

- **Registers.** Fork-only registers sit in gaps of upstream's map.  When
  merging, check that upstream hasn't claimed 0x065–0x067 or 0x06d–0x06f; it
  already took 0x072–0x074 once.
- **Subframe 0x60.** The telemetry block uses subframe type 0x60, which
  upstream mjlib doesn't use today.
- **Merge conflicts.** Upstream changes to `aux_port.h`, `fdcan_micro_server.h`,
  `moteus_controller.cc`, `drv8323.cc` or `motor_position.h` are the likely
  conflict points.
