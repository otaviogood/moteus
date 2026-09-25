# Latency: measurements and what to do about them

Status 2026-09-23.  Measured on the bench board (moteus r4.x, id 2, IMU
fusion on aux2, 30 kHz PWM) with a Saleae on CAN RX/TX + DBG1/DBG2 and the
firmware's cycle-count profiler.  Nothing here has been measured on the
robot yet.

## 1. Summary

- On the board, a CAN frame costs a few hundred microseconds: the reply to
  the robot's telemetry request starts **330 µs** (median, 400 µs max)
  after the request, and a position command reaches the control loop
  **175 µs** (median, 260 µs max) after its frame.
- Those numbers are set by CPU time, not the bus: the control interrupt
  takes **68 %** of the CPU in position mode, so every microsecond of
  main-loop work takes about 3.3 µs of real time.
- The interrupt is now ~2 µs shorter and the DRV8323 stalls are gone (§4).
  What's left in the interrupt is upstream control math; it won't get much
  faster without a PWM-rate change.
- The big latencies are elsewhere: the observation and command paths on the
  host side (≈ 20 ms, §5 items 1–3) and the host/transport time around the
  bus (≈ 1 ms command pickup, 1.5–2.6 ms reply burst per bus, §5 items 4–5).

## 2. One robot tick, board side (measured)

Times are from the end of the request frame's ACK slot; the frame becomes
valid for the receiver about 7 µs later (ACK delimiter + end of frame).
Bench pattern: at 120 Hz, a position command to the board, then the robot's
broadcast telemetry request (0x7F, 20 bytes, 48-byte reply).

| Step | Median | Max | Notes |
|---|---|---|---|
| Frame valid → main loop picks it up | 58 µs | 131 µs | 29 µs median when stopped (interrupt duty 45 %) |
| Telemetry request processing | 233 µs | 294 µs | 69 µs of CPU; the rest is the control interrupt |
| Processing done → reply on the bus | 35 µs | 63 µs | |
| **Request → reply starts** | **330 µs** | **400 µs** | the 48-byte reply then takes ~110 µs on the bus |
| Position command processing | 103 µs | 161 µs | 32 µs of CPU, ends when the command reaches the servo |
| Command handed off → control loop uses it | 4 µs | 13 µs | next control interrupt |
| **Command frame → control loop uses it** | **175 µs** | **261 µs** | |

The bench adapter (fdcanusb) sends about one frame per millisecond, so here
the command and the request never arrived back to back.  On the robot they
will, and the request then waits behind the command's processing.  Estimated
from the numbers above, the reply still starts about 340 µs after the
request, because the command's processing overlaps the request's own wait
for the main loop.  Each further board's reply on the same bus adds
~115 µs: for 4 boards, the last reply ends around 0.8 ms after the request.

Motion changes nothing: a 0.5 rev/s spin gives the same numbers as a
zero-torque hold.

## 3. Where the CPU goes

### Control interrupt (30 kHz, 33.3 µs period)

Worst case per period: 15.2 µs stopped, 23.7 µs position hold, 24.6 µs with
a moving setpoint (the heaviest path).  Breakdown in position hold (means):

| Stage | µs | Owner |
|---|---|---|
| Entry, ADC wait and reads | 2.4 | upstream |
| Encoder read (aux1 SPI) | 0.9 | upstream |
| IMU I2C byte poll + fusion driver | 1.45 (2.5 max) | ours |
| Position update (sources, PLL, output) | 3.8 | upstream |
| Velocity/temperature filters, thermistors | 1.35 | upstream |
| Current state (scaling, Clarke/Park, power limit) | 3.0 | upstream |
| Mode and fault checks | 1.4 | upstream |
| Trajectory (`UpdateCommand`) | 1.9 (2.8 moving) | upstream |
| Position PID, torque limits, cogging, flux brake | 2.2 | upstream |
| Current loop | 3.6 | upstream |
| PWM output | 1.7 | upstream |

Only the I2C stage is ours, and it has been moved into fast RAM (§4).  The
rest is dense upstream math with nothing obviously wasted.  The double
precision in the interrupt (`motor_position.h:420`, `ISR_SetOutputPosition`)
only runs once, when the output position is homed from a reference source.

### Main loop, per CAN frame (CPU time, excluding the interrupt)

| Frame | Total | Of which |
|---|---|---|
| Robot telemetry request (13 registers) | 69 µs | IMU fusion 12 µs (frame start 3.9 + quaternion reply 8.3), register handling and reply ~50 µs |
| Position command | 32 µs | frame start 3.9, parsing and handoff ~28 |
| Query while stopped (10 registers) | 42 µs | IMU fusion 12, the rest ~30 |

3–4 µs of CPU per register read is slow for what it does.  That code runs
from flash, and CCM (the fast on-chip RAM) has only 2.1 KB free.

### Telemetry request (humanoid3, 2026-09-24)

The per-tick request is now the **telemetry block** (`fw/telemetry_block.h`,
`docs/protocol/registers.md`): two bytes, `0x60 0x01`.  Each board answers
one type byte and a fixed list of register values with no per-register
headers, built by calling the controller's own register reads (same
scaling and NaN codes):

| Board | Values | Reply |
|---|---|---|
| Motor | quaternion, gyro rate (int16 ×3 each), q current (int32, 0.001 A), power (int16, 0.05 W), motor temperature, bus voltage (0.5 V), board temperature, fault, mode (int8) | 24 bytes, exactly a 24-byte frame |
| Sensor | 0x050–0x051 encoder position + PLL velocity, quaternion, gyro rate (int16) | 17 bytes, 20-byte frame |

With ordinary register reads the same values took 42 bytes (48-byte frame)
and 22 bytes (24-byte frame): every read item carries a 2-byte header, and
the motor board also answered its own 0x050–0x051, which the host throws
away.  The block also skips mjlib's per-register parse and encode, which
§3 measured at 3–4 µs of CPU per register (not re-measured).

Read subframes after the marker are answered after the block, as many
whole items as fit in 64 bytes.  A block-only request hands mjlib a single
no-op byte: a 0-byte read is an error to the multi-transport layer, which
then has mjlib re-process the previous frame (found on the bench: a stale
console reply rode along).

The motor-side position and velocity (0x001–0x002, float, rotor
revolutions) are in `TELEMETRY_REQUEST_FULL` only: the block plus that
read, a 34-byte motor reply (48-byte frame); sensor boards
filter the extra reads out and send the block alone.  The driver sends it
until every joint's `ANGLE_OFFSET_DEG` has latched: `sendPositionCommand`
adds the offset to every command, and it latches once from the first 0x001
that arrives with the sensor angle known.  Calibration tools, `--diag`,
traces and `canmonitor.py` always use the full request.  A motor board that
resets mid-run is not re-latched (as before).  Verified on the bench and in
both simulators.

### Broadcast reply filter (sensor boards answer only what's used)

A sensor board (`otavio_flags` bit 0: no gate driver found at boot)
answers only the broadcast reads (destination 0x7F) that touch
`kSensorBoardBroadcastReads` (`fw/broadcast_reply_filter.h`):

- 0x050–0x053: encoder position and velocity
- 0x065–0x06f: the gyro-rate slot and the quaternion

The filter runs in `FDCanMicroServer::Poll` before mjlib sees the frame.
There's no config: motor boards are unaffected, and requests addressed to
a board directly are never filtered.  Cost: about 900 B of flash.

The filter still applies to ordinary reads in a broadcast; with the block
request it only matters for reads piggybacked after the block.

## 4. What changed during this investigation

| Change | Effect |
|---|---|
| DRV8323 status poll: hardware SPI1 on r4.x instead of bit-banged SPI, split into one step per ms (`fw/drv8323.cc`) | Removed 0.3–0.55 ms main-loop stalls every 10 ms while the driver is enabled (4 % of frames were delayed).  Reply p99 760 → 372 µs, command p99 610 → 230 µs |
| IMU I2C interrupt path moved into CCM (`ISR_I2C_Update`, fusion driver `ISR_Start`/`ISR_Complete`, `Stm32I2c::StartReadMemory`); room made by moving the debug-stream emitter body to flash | Interrupt −2 µs in every mode (I2C stage 2.4 → 1.45 µs mean, 4.4 → 2.5 µs max) |
| Scope markers reduced to DBG1/DBG2 with `d mark phase\|can\|fusion` | CAN frames and fusion work visible on one probe |
| Interrupt profiler: 16 points, per-second mean and max in `servo_stats.dwt` (`MOTEUS_PERFORMANCE_MEASURE` builds) | The §3 breakdown |
| Fusion gyro rate served at 0x065–0x067 (`ImuFusion::RateAt`, `fw/moteus_controller.cc`) | Item 1 below: the rate is on the wire; the host stores it (`GYROM_*`/`GYROS_*`) |
| Telemetry block request (`fw/telemetry_block.h`, `FDCanMicroServer`) | Motor reply 48 → 24-byte frame (with int16 power), sensor 24 → 20-byte frame; no per-register protocol work for the robot's request |
| Encoder PLL gains computed before the motor checks in `MotorPosition::HandleConfigUpdate` (`fw/motor_position.h`) | A board with no motor (poles 0, every sensor board) used to freeze 0x050 and read 0x051 = 0 when its PLL was turned on; now the PLL runs (item 2) |

Known and left alone: each time the gate driver is enabled, one control
period stretches to 44–47 µs (upstream PWM timer restart).  It happens once
per enable, not in steady running.

## 5. Every latency, biggest first

| # | Where | Now | Fix | Retrain? |
|---|---|---|---|---|
| 1 | **Angular-velocity observation**: the host differences quaternions, then low-passes (α = 0.5) | ≈ 4 ms (half-step difference at 120 Hz) + ≈ 8 ms (filter) ≈ **12 ms** | **Done (2026-09-25), needs a retrain:** the fusion's bias-corrected gyro rate of the newest 960 Hz sample, 0x065–0x067, in the per-tick request (bench at rest: 0 ± 0.06 dps).  humanoid3: `--imu-ang-vel-source gyro` trains on it (engine site rate, misalignment, fusion delay + sample age, bias/scale/white noise, int16 wire; no low-pass); the runner reads it from the checkpoint and the chest uses sensord's raw gyro.  The board frame is the quaternion's, so no mounting rotation is needed | yes |
| 2 | **Joint (post-spring) velocity**: backward difference of the sensor board's output encoder (0x050) at 120 Hz | ≈ **4 ms** | **Done (2026-09-25), needs a retrain:** 0x051 in the per-tick request; PLL fixed for boards with no motor.  humanoid3: `--joint-velocity-source sensor_pll` trains on the firmware PLL at 50 Hz (noise = kp × encoder sigma, 0.17 rad/s median); the runner refuses to take over while a sensor board's 0x051 is exactly 0.  Left: set `pll_filter_hz 50` on the 24 sensor boards after the rollout (it also makes 0x050 the PLL-filtered position) | yes |
| 3 | **Position staircase**: each command holds for 8.3 ms | ≈ **4 ms** on average | Send a velocity with each position command; position mode then ramps the setpoint between commands | yes |
| 4 | **Reply burst**: the host waits for every board on its bus | 1.5–2.6 ms per bus (measured earlier from the host) | The board side is ~0.33 ms to the first reply + ~115 µs per board (§2), so over half of this is host/transport.  Board-side levers: fewer or smaller registers (int16 instead of float32), then a faster CAN FD data rate if the transceivers allow it | no |
| 5 | **Command pickup**: nothing happened for ~1.0 ms after the host sent a frame | ~1 ms | The board takes 175 µs median (260 µs max) from frame to control loop, and the frame itself ~85 µs on the bus, so ~0.7 ms is on the host side (Python loop, socketcan queue, driver).  Next: timestamp both ends on the robot (§6) | — |
| 6 | **Fixed IMU sensor delay**: chip gyro filters + I2C transfer | ≈ 1–1.5 ms | Measure it, then set `latency_comp`: it exists only as the console command `aux2 fusion latency <us>` and isn't saved yet.  Or widen the gyro filter | no (small) |
| 7 | **Current slew limit** `servo.max_current_desired_rate` = 10,000 A/s | up to 1.2 ms on a full ±6 A swing | Raise it (e.g. 30,000–50,000 A/s) and check current noise on the bench | small |
| 8 | **Current loop** `servo.pid_dq_hz` = 100 Hz with R·i feedforward | effective τ ≈ 0.4 ms (L = 0.163 mH, R = 0.30 Ω) | 200–300 Hz → ≈ 0.25 ms; small gain, some noise risk | small |
| 9 | **Board reply turnaround** (telemetry request → reply starts) | 330 µs median, 400 µs max | CPU-bound (§3): fewer registers per request, or a faster register path | no |
| 10 | **Command to control loop** (frame → used by the servo) | 175 µs median, 261 µs max | Already handed off right after the frame; the time is main-loop starvation by the interrupt.  Same levers as #9 | no |
| 11 | PLL velocity for the damping term (992 Hz) | ≈ 0.16 ms | fine as is | — |
| 12 | PWM applies the next period's voltage | 33 µs | negligible | — |

Encoder PLL noise at rest (bench board 2, onboard AS5047 with a magnet,
600 samples at 120 Hz per setting, each measured twice; encoder-shaft
degrees = joint degrees on a sensor board):

| `pll_filter_hz` | 0x051 σ | 120 Hz difference of 0x050 σ | 0x050 σ |
|---|---|---|---|
| 0 (the robot's sensor boards today) | — | 7.3 °/s | 0.040° |
| 50 | 10.6 °/s | 0.77 °/s | 0.005° |
| 100 | 21.8 °/s | 1.3 °/s | 0.007° |
| 248 | 50.7 °/s | 2.2 °/s | 0.013° |
| 992 | 192 °/s | 5.1 °/s | 0.030° |

The PLL velocity noise grows about linearly with the bandwidth, ≈ 0.21 °/s
per Hz, so the ≈ 0.2 ms of the 992 Hz setting is not usable.  The velocity
output of this (critically damped, type 2) PLL has no delay at DC; at 5 Hz
of joint motion its delay is ≈ 0.8 ms at 50 Hz and ≈ 1.9 ms at 35 Hz.  So
35–50 Hz gives today's noise with ≈ 1–2 ms delay instead of 4 ms.  Any
setting also cuts the position noise by 3–8×, which helps the existing
difference as well.

Two more aggressive options:

- **Predict ahead:** with `latency_comp` larger than the sensor delay, the
  firmware reports the orientation a few ms in the future (up to 8 ms),
  cancelling host compute and transport time.  Training has to model it.
- **Higher gyro rate:** 1920 Hz needs I2C DMA and gains little, because
  replies are already evaluated at the request time.

### CPU-level levers, if the board-side numbers matter later

- **PWM rate** (`servo.pwm_rate_hz`, 30 kHz now).  At 20 kHz the interrupt
  takes about 45 % of the CPU instead of 68 %, so all main-loop latencies
  (§2) shrink by roughly a third.  Costs current-loop bandwidth, and the PWM
  frequency becomes audible; the robot's control would need re-checking.
- **Register path in the main loop** (3–4 µs of CPU per register): profile it
  with DBG2 sub-markers, then consider a leaner encoder for the fixed robot
  telemetry request.  CCM is nearly full, so moving it there means evicting
  something else.
- **Fewer registers per request**: every register dropped saves 3–4 µs of
  CPU (~10–13 µs of real time) per board per tick, plus bus time.

## 6. Recommended next steps

1. **Items 1–2**: firmware, host and trainer are in place (2026-09-25);
   what's left is the robot rollout, `pll_filter_hz 50` on the sensor
   boards, and a retrain with both sources.  Roughly 12 + 2–3 ms off the
   observation path (item 2 is limited by the PLL noise, §5).  Item 3 only
   matters for position-mode policies; the robot runs torque mode now.
2. **Account for the host-side millisecond (items 4–5)**: on the robot,
   with the motors limp, timestamp frames on the Orin (socketcan hardware or
   kernel timestamps) against the bus (a logic analyzer on one bus's
   RX/TX).  This splits the ~1 ms command pickup and the reply-burst wait
   into Python, kernel, driver and bus.
3. **Trim the telemetry request** (item 9): drop registers the policy
   doesn't use, and use int16 where resolution allows.
4. **Bench tuning, no retrain needed or small**: measure the IMU sensor delay and
   set/persist `latency_comp` (6), raise the current slew limit (7), try a
   200–300 Hz current loop (8).

## 7. How to measure

- **Builds**: normal `tools/bazel build --config=target //:target`; scope
  markers `--copt=-DMOTEUS_SCOPE_MARKERS`; interrupt profiler
  `--copt=-DMOTEUS_PERFORMANCE_MEASURE`.  Flash with `moteus_tool --flash`
  over CAN only; dump the config before, `d reset`, compare after.
- **Scope markers** (`fw/scope_markers.h`, r4.x only): DBG1 = control
  interrupt, sampled ADCs → done.  DBG2 by console command:
  - `d mark phase`: `d mark 1`/`d mark 0` drive it.
  - `d mark can`: one pulse per CAN frame, notched low during the
    frame's fusion work.
  - `d mark fusion`: all main-loop fusion work.
- **CAN capture**: `utils/imu_fusion_bench/scope_can_capture.py <id>`.
  Probes: CAN RX/TX on the logic side of the transceiver, DBG1, DBG2.
  Sample at 100 MS/s (the 5 Mbit/s data phase has 200 ns bits).  Phases:
  1. robot pattern at 120 Hz
  2. the same plus dummy fill frames
  3. back-to-back queries
  4. a gentle spin (0.5 rev/s, 0.2 N·m max; the rotor must be free)
  5. five zero-torque enable cycles

  Each phase ends with a report of `isr_max` and the gate driver's checks.
- **Capture analysis**: decode CAN FD from RX (standard IDs for ids below
  0x800, extended above; destuff; BRS at 1/5 Mbit/s); match each frame to
  its DBG2 pulse (the pulse starts with a 0.1 µs blip before the frame-start
  notch); subtract DBG1-high time for CPU time.  The scripts used so far
  live outside the repo.
- **Interrupt profile**: with the profiler build, read `servo_stats.dwt.mean`
  / `.max` (cycles at 170 MHz from interrupt entry to each point in
  `BldcServoStatus::PerfPoint`) after a couple of seconds in each state.
