# IMU orientation redesign (LSM6DSV16X on aux2)

**Current state (2026-09-22): implemented, bench-verified.** The fusion is
the only LSM6DSV16X mode: config type 3 (`lsm6dsv16x`) selects it. The SFLP
modes (the old types 3 and 4), the SFLP bench comparison mode and the
transitional type 5 (`lsm6dsv16xFusion`) are gone; read "type 5" below as
"type 3". Existing type 3 configurations switch to the fusion when flashed.
The sections below are the design record and bench history.

Status: **design v8, 2026-09-21, not implemented.** v1–v7 were reviewed
the same day; the findings, v4's clock rethink, v6's freshness toggle and
v7's scoping are in §10. §1.5 states what the design assumes and what it
deliberately does not cover. Numbers
quoted below come from [`imu_orientation_quantization.py`](imu_orientation_quantization.py)
(run it: `python3 docs/imu_orientation_quantization.py`), from bench
measurements on motor id 2, and from the LSM6DSV16X datasheet DS13510 Rev 4
(cited as "DS §x" / "DS Table x"). Nothing in this design changes the CAN
request/reply bytes the Orin sends and receives — only the meaning of the six
data bytes.

Scope: the fork's IMU path only — `fw/aux_port.h` (LSM6DSV16X driver),
`fw/aux_common.h` (config/status structs), `fw/moteus_controller.cc`
(registers 0x06d–0x06f), `fw/fdcan.cc` (hardware frame arrival timestamps), plus two small new
files: `fw/timer_ownership.h`, `fw/imu_fusion_storage.cc`, and the
Orin's CAN parser / decode path. Motor control code is untouched.

---

## 1. The problem

### 1.1 What ships today

Every board (24 motor + 24 sensor) carries an LSM6DSV16X on aux2 (I2C,
address 106, `aux2.i2c.devices.0.type 3`, `poll_rate_us 2000`). The firmware
enables the chip's own sensor-fusion block ("SFLP") at 240 Hz and reads its
**game rotation vector** from the chip FIFO (`ReadIMUData` in
`fw/aux_port.h`). That vector is three IEEE half-floats x, y, z; the fourth
component is not transmitted by the chip at all (DS §1: "The X, Y, Z
quaternion components are stored in FIFO"). The firmware forwards the three
raw 16-bit patterns as registers 0x06d, 0x06e, 0x06f; the Orin reconstructs
`w = sqrt(1 - x² - y² - z²) ≥ 0` (`orin/imu_common.py:decode_game_quat_xyz`,
mirroring ST's `sflp2q`).

### 1.2 Root cause of the "~3 degree steps"

Reconstructing w is ill-conditioned when w is small: `dw ≈ (|xyz|/w)·d|xyz|`,
and the fp16 step for components in [0.5, 1) is 2⁻¹¹. Geodesic error of the
round trip for random orientations at fixed w (analysis 1):

| w = cos(θ/2) | θ from reference | error p50 / p99 / max | first representable step off w = 0 |
|---|---|---|---|
| 0.90 | 52° | 0.010 / 0.018 / 0.023° | – |
| 0.50 | 120° | 0.029 / 0.063 / 0.080° | – |
| 0.10 | 168° | 0.12 / 0.33 / 0.47° | – |
| 0.05 | 174° | 0.24 / 0.67 / 1.00° | – |
| 0.02 | 178° | 0.61 / 2.29 / 2.29° | 2.75° |
| **0.00** | **180°** | 0.04 / 2.64 / **3.22°** | **3.58°** |

A concrete case: the exact quaternion (0, 1/√2, 1/√2, 0) comes back from the
old format with a **1.68°** error (fp16 rounds 0.70711 to 0.70703, and w
then reconstructs as 0.0148 instead of 0). Keep this example in mind for §7:
near w = 0 the SFLP output is *wrong*, so it cannot serve as a reference
there.

When does w ≈ 0 happen? The game vector's world frame is gravity-aligned
(Z up) with an arbitrary heading fixed at power-on. For a board whose Z axis
is tilted α from world-up at heading ψ from that power-on heading,
`w = cos(ψ/2)·cos(α/2)`:

* a board mounted Z-down (α = 180°) has **w = 0 at every heading** — permanently broken;
* every other board reaches w = 0 when ψ → 180°. The heading is arbitrary per
  power cycle and drifts (measured 1e‑4…2.5e‑3 rad/s), and legs yaw when the
  robot turns. So it is a per-board, per-power-cycle lottery.

The chip cannot output w (SFLP only emits those three half-floats), so any
fix means either our own fusion or a different SFLP output. Own fusion was
chosen (§3).

### 1.3 Other defects in the current path (fixed by the same rework)

1. **X/Y/Z are not from one sample.** `quaternion_cache_valid_ = false` sits
   at the top of `MoteusController::Impl::Read()`; mjlib calls `Read()` once
   per register from the main loop, which the 30 kHz control ISR preempts
   freely. The per-frame hook `StartFrame()` exists and is the right place.
2. **Angular velocity is finite-differenced on the Orin** at 120 Hz from
   229–236 Hz SFLP samples, giving the 0.52× "one sample instead of two" blips
   (`orin/imu_common.py:ang_vel_from_quat_pair`). The gyro that measures ω
   directly is never read.
3. **Latency 7.3 ms** (measured, `humanoid3/docs/imu_fusion_delay_sim_plan.md`):
   ≈4 ms SFLP filter group delay + 2.1 ms mean sample age + 0–4 ms from the
   2 ms status-then-data polling two-step.
4. **Blocking I2C inside the ISR**: `ReadIMUData` calls `wait_i2c()` when the
   FIFO holds ≥ 2 words (~200 µs spin ≈ 6 control periods), which happens on
   clock-drift beats roughly once a second per board; and `InitLsm6dsv16x`
   blocks ~1 ms — not only at boot: `HandleConfigUpdate()` does
   `i2c_state_ = {}` (`fw/aux_port.h:1067`), so every `conf set aux2.*`
   re-runs the blocking init while the motor may be in position mode.
5. The forced `w ≥ 0` hemisphere makes the stream sign-discontinuous, which
   the Orin patches with a dot-product continuity check.

### 1.4 Constraints carried into the design

* Three registers below 0x80, one short-opcode read subframe (`0x17 0x6d`),
  8-byte reply — bus traffic must not grow.
* The reply must be self-consistent (one IMU sample per CAN frame).
* Calibration must survive flashing; see §5.6 for what this change does to
  the config groups (it is worse than v1 claimed).
* Control ISR budget, measured on bench motor 2 via `servo_stats.final_timer`
  (200 samples): ISR ends at 42.5–47.3 % of the 33.3 µs period idle, but at
  **66.8–76.5 % in position mode (7.8 µs free in the worst observed cycle)**.
  `final_timer` is a phase, not a duration — it cannot see an overrun of a
  whole period; §7 uses DWT cycle counts instead.
* `conf write` blocks the main loop: two flash pages are erased synchronously
  (`fw/stm32g4_flash.h:36-43`, `NbPages = 2`; STM32G474 tPE 22 ms typ /
  24.5 ms max per page → 44–49 ms) and then the configuration is programmed
  (tens of ms more). Budget the **whole save at ≈ 100 ms**. Anything the ISR
  hands to the main loop must survive that. Separately, the main loop's own
  watchdog (`fw/moteus.cc:359-366`) faults the servo whenever the loop stalls
  ≥ 4 ms *and* `servo.timing_fault` is enabled — pre-existing behaviour,
  independent of this design.
* **The firmware's microsecond timer is 16-bit, and that is fine.** The
  mbed microsecond timer is overridden to **TIM15, 16 bits wide**
  (`WORKSPACE:96-100`: `MBED_US_TIMER_TIM = TIM15`,
  `TIM_MST_BIT_WIDTH = 16`), so `MillisecondTimer::read_us()` wraps every
  **65.536 ms**; every existing use is a short delta. Nothing in this design
  measures a long interval — every long duration is a sample count — and the
  one long main-loop stall (`conf write`) is detected by a sample count, not
  by a timer (§5.7). The design's own clock is TIM3, 16-bit at 4 µs, which
  the FDCAN stamps frames with anyway.
* **RAM is budgeted, not free.** The object pool is 24 000 bytes
  (`fw/moteus.cc:206`) and the bench board reports **1 184 bytes
  available** (`system_info.pool_available`, board 2, 2026-09-21);
  `MoteusController::Impl` alone is 13 272 bytes and each `AuxPort` 5 016
  (DWARF of the current build). `main()`'s frame is 27.5 KB of the 96 KB
  main SRAM; CCM SRAM (32 KB) holds ISR code with 1.7 KB spare. New
  buffers go in one static block outside the pool (§5.8).
* The robot never moves without an explicit go-ahead; bench work on motor 2.

### 1.5 Operating assumptions, and what is deliberately not covered

The mechanisms in §5–§6 are sized for ordinary operation, not for every
failure. Stated once here so the rest of the document can use plain
language instead of "guaranteed":

* **One poller per board during operation** — the Orin driver. Diagnostic
  tools (`moteus_tool`, `check_robot_imus.py`, canmonitor) run when the
  driver is not, or at most one of them beside it; the host's
  three-in-a-row toggle rule absorbs one extra poller, no more.
* **A continuously streaming IMU.** The firmware recovers from I2C errors
  (resync → re-init) but not from a chip that silently stops delivering
  words (e.g. reset to defaults by an ESD event): such a board reads
  *stale* on the host until `conf set aux2.…` or a power cycle re-runs
  Init. The toggle bit is a **progress check**, not a proof of freshness.
* **A 16-bit clock** (TIM3, 4 µs, 262 ms wrap) with signed differences
  valid within ±131 ms. Comparisons outside that window are prevented by
  the stall latch and by the host's toggle rule *under the two assumptions
  above*, not by arithmetic.
* **Bounded CAN loss.** The sentinel hold (three quaternion replies) and
  the toggle survive one lost or interleaved reply. Anything longer is
  caught by the host's outage reset (§6), which is the backstop for every
  item in this list.
* **Not covered, by choice:** silent chip resets (above); three or more
  consecutive lost replies to one board (the outage reset handles the
  consequence, not the cause); more than one extra poller; main-loop
  stalls other than `conf write`.

---

## 2. Decision summary

| Topic | Decision |
|---|---|
| Orientation source | Own Mahony-type filter in firmware from raw gyro + accel; SFLP off (kept batched only for bench A/B) |
| Gyro / accel rates | Gyro 960 Hz, accel 120 Hz, both batched in the chip FIFO with separate batch rates |
| Data path | I2C byte pump stays in the 30 kHz ISR (as today); completed FIFO words go into a lock-free mailbox with a producer sequence number and the ISR time of arrival; **fusion math runs in the main loop** |
| Sample timing | Sample instants come from a **sample-clock model** driven by the FIFO word sequence (rate seeded from `INTERNAL_FREQ_FINE`, phase and rate refined by a slow floor tracker on arrival times; gaps counted exactly by sequence numbers; chip timestamp words re-establish the sample *index* after a loss and carry no STM32 phase) |
| Wire format | Registers 0x06d–0x06f unchanged; "smallest-three" quaternion: 3 × 15-bit + 2-bit index, bit 47 = **freshness toggle** (proven bound 0.0086°) |
| Per-frame consistency | Snapshot in `StartFrame()` |
| Reply timing | Each reply reports the orientation **at the request frame's arrival instant**, evaluated from a timestamped orientation history (interpolation, or extrapolation of at most 8 ms). The arrival instant is the FDCAN's **hardware SOF timestamp**, taken from TIM3 as the external timestamp counter — already in fusion-clock ticks, nothing to pair; no new interrupt. TIM3 ownership is registered; fusion mode requires it and refuses to start otherwise |
| Time base | **TIM3, free-running 16-bit at 4 µs** (262 ms wrap), the one clock for the ISR stamps, the history and the CAN stamps. Every time comparison is a signed 16-bit difference, valid within ±131 ms; a main-loop stall is detected by the ISR's gyro sample counter, never by a timer, and one latch — no stale CAN frames pending — gates the request comparison; whether the *data* is still advancing is judged by the host from the wire toggle bit, not by any firmware clock (§4.2, §5.7) |
| Memory | Mailbox (3 KB) + orientation history (1.3 KB) + filter state ≈ 4.5 KB in **one static block**, claimed by the port that runs the fusion; nothing added to the pool (§5.8) |
| Recovery | Re-initialization keeps the last heading, clears the history, holds the sentinel until converged **and until three sentinel replies have been sent**; the host latches sentinel/reinit events at parse time and resets its finite-difference baseline on every reinit and on any telemetry outage (§5.3, §6) |
| Chip init | Non-blocking state machine; starts with `FUNC_CFG_ACCESS = 0` then `SW_RESET`, so interrupted inits, mode switches and reboots leave no stale chip state |
| Config | No new persistent fields in v1 — only the new `type` enum value; filter gains are compile-time defaults with a runtime console override for tuning |
| Payload | Orientation only for now; gyro on the wire is a later phase |
| Integration | Trapezoid (average of neighbouring gyro samples) |

---

## 3. Alternatives considered

| Alternative | Why not |
|---|---|
| 4 × int16 quaternion | Needs 4 registers → long-form opcode, +3 bytes per reply, and there is no free 4-run below 0x80 next to 0x06d |
| 12:12:12:12 fixed point | 0.055° sampled worst case vs 0.0086° proven for smallest-three; no other advantage (analysis 2) |
| Rotation vector 3 × 16 bit | 0.0046° sampled — marginally better, but trig on decode and a sign seam at θ = π |
| Gravity vector + gyro payload | Loses yaw for good; deferred, yaw is unused by the policy today but may be wanted |
| SFLP gravity vector (16-bit) instead of own fusion | Internal precision unverified, keeps the 4 ms SFLP delay and the chip clock; fallback only |
| Fusion on the Orin | Needs raw accel+gyro at ≥ 240 Hz per board — CAN bandwidth ×2 (see `humanoid3/docs/can_frame_loss_2026-08-21.md`); 120 Hz integration aliases vibration |
| Polling the latest-value registers (0x22–0x2D) | A read takes 0.3–0.6 ms at one byte per ISR cycle; a sample arriving mid-read can be silently skipped (30–60 % per read at 960 Hz) = lost rotation. FIFO words cannot be lost |
| Fusion inside the ISR | Only 7.8 µs free per cycle in position mode; the main loop has the time and per-sample timestamps make the result identical |
| 1920 Hz gyro | ≈96 % of ISR cycles moving an I2C byte — no margin. Needs I2C DMA, a separate change; and the integration numbers show no benefit (analysis 4) |
| I2C DMA now | Would free the ISR from the byte pump; not needed for 960 Hz; later if ever |
| I2C fast-mode-plus (1 MHz) | Supported by the chip (DS Table 8) but irrelevant: throughput is bounded by the ISR byte pump, not the wire |
| FDCAN *internal* timestamp counter | Its time base follows the current bit time, which changes between arbitration and data phase with BRS on (RM0440 §44.4.8: use the external counter, TIM3, for CAN FD); traffic-dependent errors |
| RX-FIFO0 interrupt stamping in software (v3's primary) | `RF0N` is a level flag, not an event queue: two frames arriving before the handler runs leave the older one mis-stamped; the FIFO has three elements (`SRAMCAN_RF0_NBR`) and wraps; a correct version needs a drain-in-handler queue with ambiguity flags and a *measured* entry latency. Unnecessary: the peripheral stamps every element in hardware at SOF when TIM3 is its timestamp counter, and TIM3 is free on every board in the robot (§5.4) |
| Reporting the newest fused orientation, clamped | If request processing was delayed, the newest state is *after* the request; clamping reintroduces processing-time jitter — hence the history query (§5.4) |
| "valid" flag in bit 47 | A static flag says nothing the sentinel does not. v7 uses the bit as a **freshness toggle** instead (§4.2): it carries liveness, which a flag cannot |
| ISR no-data watchdog with re-init (v6) | Five lines in the ISR plus a resync path to answer "what if the chip silently stops" — a case we are not designing for. The toggle bit gives the host the same detection with one XOR; a silently reset chip stays stale until Init is re-run by config or power cycle |

### 3.1 Why 960 Hz is enough (analysis 4)

Pure-gyro integration, no accelerometer correction (worst case), 3 s of
aggressive multi-axis motion (peak 8.5 rad/s) plus a 280 Hz vibration, gyro
quantized to 70 mdps/LSB, compared with a 96 kHz reference. Max error during
the motion / error at the end:

| gyro rate | sample-and-hold | trapezoid |
|---|---|---|
| 240 Hz (what the SFLP integrates today) | 1.47 / 0.43° | 0.070 / 0.029° |
| 480 Hz | 0.74 / 0.21° | 0.027 / 0.005° |
| **960 Hz** | 0.37 / 0.11° | **0.009 / 0.002°** |
| 1920 Hz | 0.19 / 0.05° | 0.006 / 0.001° |
| 3840 Hz | 0.09 / 0.03° | 0.005 / 0.000° |

Sample-and-hold's error is a dt/2 lag, removed by the trapezoid. Above
~480 Hz the curve is flat. What limits real orientation quality is gyro bias,
accelerometer bias and dynamic acceleration — all rate-independent and the
filter's job (§5.3). The SFLP's maximum rate is 480 Hz (DS Table 324) and the
boards run it at 240 Hz, so this design integrates 4× more gyro information
than today.

---

## 4. Wire format (normative)

### 4.1 Registers

| register | today | new |
|---|---|---|
| 0x06d | fp16 bits of x | word 0 (bits 0–15 of the 48-bit value) |
| 0x06e | fp16 bits of y | word 1 (bits 16–31) |
| 0x06f | fp16 bits of z | word 2 (bits 32–47) |

Read as int16 only (`0x17 0x6d`, reply `0x27 0x6d` + 6 bytes, as today). The
firmware returns the bits verbatim via `Value(int16_t)` for type 1 and
answers any other type with the unknown-register error, exactly as the fp16
path does now. No `ScaleMapping` is involved.

### 4.2 Encoding ("smallest three")

Input: unit quaternion `q = (w, x, y, z)`, board → world, Z-up world,
arbitrary heading (same semantics as the game vector, so
`projected_gravity_from_quat` on the Orin is unchanged).

1. `i = argmax_k |q_k|` (ties → lowest index).
2. If `q_i < 0`, negate all four components (same rotation).
3. The three remaining components, in ascending index order, each lie in
   `[-1/√2, +1/√2]` (the largest is ≥ ½ in magnitude, so
   `w_i = sqrt(1 - Σ others²)` is always ≥ ½ and reconstruction error
   amplification is ≤ 1.73).
4. Each is stored as an unsigned 15-bit code:
   `code = round((x·√2 + 1)/2 · 32767)`, clamped to 0…32767.
5. 48-bit little-endian value: bits 0–14 code₀, 15–29 code₁, 30–44 code₂,
   45–46 `i`, bit 47 = **freshness toggle** (below).

Decode: undo step 4 (`x = (code/32767·2 - 1)/√2`), set
`q_i = sqrt(max(0, 1 - Σ others²))`, normalize. The result is always a valid
unit quaternion; `q` and `-q` describe the same rotation, so consumers that
difference consecutive samples keep the existing dot-product continuity check.

**Bit 47, freshness toggle.** The firmware flips this bit on every valid
*quaternion reply* — a reply that actually carries registers 0x06d–0x06f —
whose newest history sample is newer than the one the previous valid
quaternion reply saw (`if (newest_seq != last_reply_seq) toggle ^= 1`). The
bookkeeping runs when those registers are read for a reply, **not per
received CAN frame**: motor commands, diagnostic/config traffic and
broadcast frames that do not read the quaternion consume neither the toggle
nor a sentinel count (§5.4). A consumer
polling slower than the sample rate therefore sees the bit alternate on
every reply while data flows and repeat when it does not. This replaces any
firmware-side "is the data still fresh" judgement, which would need a clock
the firmware cannot trust across a counter wrap (§5.7); the host judges
freshness by counting repeats, which no wrap can fake. Decoders mask the
bit out before decoding (§6 has the host rule).

### 4.3 Sentinel

Low 47 bits all zero cannot occur for a real quaternion (it would put all
three stored components at −1/√2, Σ = 1.5). The firmware sends all 48 bits
zero whenever it cannot produce an orientation for the request instant under
the rules of §5.4 (filter not converged, history too old, I2C resync in
progress, arrival unknown). Decoders test the low 47 bits, treat it as "no
data" (hold the previous value and flag staleness) and forget the toggle
state — the toggle is only meaningful between consecutive valid replies.

### 4.4 Accuracy and test vectors (analysis 3)

(All vectors below have bit 47 = 0, i.e. word 0x06f bit 15 clear; on the
wire the toggle of §4.2 is ORed into that bit, and decoders mask it before
comparing.)

**Proven worst case: 0.0086°** in ideal arithmetic (finite-perturbation
derivation in the script's `smallest_three_bound_deg`, rigorous in both
directions: stored-component rounding ≤ half a step; the omitted component's
error bounded through the unit constraint without linearization; the
rotation angle after normalization bounded via `θ ≤ 2·asin(|e|/(1−|e|))`;
an independent evaluation gave 0.0085674°). Sampled maxima are smaller
(0.0067° over 20 k, 0.0073° over 400 k, 0.0075° over 1 M orientations); the
**bound is the acceptance figure** for the Python reference. The firmware's
float32 encoder is a separate implementation and gets its own test (§7).

| quaternion (w, x, y, z) | 0x06d | 0x06e | 0x06f |
|---|---|---|---|
| identity (1, 0, 0, 0) | 0x4000 | 0x2000 | 0x1000 |
| 90° about x (0.7071, 0.7071, 0, 0) | 0x7fff | 0x2000 | 0x1000 |
| tie (0.5, 0.5, 0.5, 0.5) | 0x6d40 | 0x36a0 | 0x1b50 |
| w = 0 (0, 0.6, 0, 0.8) | 0xc000 | 0x3b26 | 0x7000 |
| negative largest (−0.8, 0.36, −0.48, 0) | 0x9f6a | 0x35b8 | 0x1000 |
| general (0.2063, −0.5157, 0.7220, −0.4126) | 0xd2ab | 0x08a9 | 0x46aa |

Note the second vector: `0x7fff` interpreted as fp16 is NaN. That is why the
Orin change must happen at the CAN parser, not at the decoder (§6).

Reference implementation: `encode_quat48` / `decode_quat48` in the companion
script.

---

## 5. Firmware design

### 5.1 Chip configuration

All register facts below are from DS13510 Rev 4; "verify" marks behaviour
that must still be confirmed on the bench.

**Init sequence — the same skeleton for both device types, always
non-blocking (§5.2), always complete.** The chip keeps its register state
across firmware reboots and across `conf set` mode switches (only the
firmware restarts, the IMU stays powered), and the legacy init never touches
`FIFO_CTRL3` in gyro mode. So:

0. `FUNC_CFG_ACCESS (0x01)` ← 0x00. This register is reachable from every
   bank and forces the main bank. It comes *first* because an init
   interrupted by an I2C error (or a previous firmware) can leave the
   embedded-function bank selected, where address 0x12 is the read-only
   `EMB_FUNC_STATUS` (DS Table 262), not `CTRL3` — a reset written there
   would silently do nothing.
1. `CTRL3 (0x12)` ← `SW_RESET` (bit 0). Poll `CTRL3` until the bit
   self-clears (DS §9.16). All control registers are now at defaults; the
   embedded-function registers are written explicitly anyway below.
2. Write every register the mode depends on (table). Never rely on a default
   that the other mode might have changed.
3. `FIFO_CTRL4` ← bypass (0x00) then the mode's FIFO mode, which flushes the
   FIFO (DS §6.12.1).
4. Read `WHO_AM_I (0x0F)` = 0x70 and `INTERNAL_FREQ_FINE (0x4F)` once;
   fail the init (and retry after 100 ms) on a wrong WHO_AM_I.

Fusion mode (`type = kLsm6dsv16xFusion`):

| register | value | meaning (DS ref) |
|---|---|---|
| CTRL3 (0x12) | 0x44 | BDU = 1, IF_INC = 1 (§9.16) |
| CTRL1 (0x10) | 0x06 | accel ODR 120 Hz, high-performance mode (Table 52) |
| CTRL2 (0x11) | 0x09 | gyro ODR 960 Hz, high-performance mode (Table 55) |
| CTRL6 (0x15) | 0x24 | FS_G = ±2000 dps (70 mdps/LSB), LPF1_G_BW = 010 → 149 Hz at 960 Hz (Table 64); 100 (100 Hz) is the alternative if gear vibration shows |
| CTRL7 (0x16) | 0x01 | LPF1_G_EN (§9.20); LPF2 is fixed at 342 Hz for this ODR (Table 21) |
| CTRL8 (0x17) | 0x01 | FS_XL = ±4 g (0.122 mg/LSB), HP_LPF2_XL_BW = 000 → ODR/4 = 30 Hz (Table 69) |
| CTRL9 (0x18) | 0x08 | LPF2_XL_EN (§9.22) |
| FUNCTIONS_ENABLE (0x50) | 0x40 | TIMESTAMP_EN (§9.53) |
| FIFO_CTRL3 (0x09) | 0x96 | BDR_GY = 960 Hz (high nibble), BDR_XL = 120 Hz (low nibble) (Table 38) |
| FIFO_CTRL4 (0x0A) | 0x00 then 0xC6 | bypass, then DEC_TS_BATCH = 11 (timestamp every 32 gyro slots), continuous mode (Table 40) |
| FUNC_CFG_ACCESS (0x01) | 0x80 / 0x00 | enter / leave the embedded bank for the two writes below |
| EMB_FUNC_EN_A (0x04, emb.) | 0x00 | SFLP_GAME_EN = 0 (Table 266) — explicit, because type 3 left it on |
| EMB_FUNC_FIFO_EN_A (0x44, emb.) | 0x00 | no SFLP batching (Table 296); bring-up debug config sets bit 1 to batch the game vector alongside |

Legacy mode (`type = kLsm6dsv16x`, the SFLP path kept for rollback) gets the
same skeleton with its current values **plus** `FIFO_CTRL3 = 0x00` and
`FIFO_CTRL4` DEC_TS_BATCH = 0, so that a board switched back from fusion
mode does not feed raw gyro/accel words to the tag-blind legacy reader.

Optional: high-accuracy ODR mode (OP_MODE = 001 in CTRL1/CTRL2, DS §6.5)
reduces the ODR tolerance to ±1 % with the gyro on, for 20 µA. Not required
because the clock is tracked (§5.3), and it must be toggled in power-down;
leave it out of v1 of the firmware.

Other datasheet facts the design leans on:

* FIFO word = tag byte (0x78) + 6 data bytes (0x79–0x7E), read in one
  7-byte transaction (DS §6.12.8). Tag byte: bits 7:3 sensor tag, bits 2:1
  `TAG_CNT` "2-bit counter which identifies sensor time slot", bit 0 unused
  (Table 217). Tags used: 0x01 gyro, 0x02 accel, 0x04 timestamp, 0x13 SFLP
  game vector during bring-up (Table 218). Timestamp word: the 32-bit
  counter, 1 LSB = 21.75 µs typ (Table 127) — byte layout inside the 6 data
  bytes to verify.
* `FIFO_STATUS1/2 (0x1B/0x1C)`: `DIFF_FIFO[8:0]` unread words;
  `FIFO_OVR_LATCHED` (bit 3 of 0x1C) "reset when this register is read" —
  read both bytes every poll, so an overrun is never missed (Table 79).
* `INTERNAL_FREQ_FINE (0x4F)`: signed 8-bit in 0.13 % steps;
  `ODR_actual = 7680·(1 + 0.0013·FREQ_FINE)/ODRcoeff`, ODRcoeff = 8 at
  960 Hz; the timestamp tick is `1/(46080·(1 + 0.0013·FREQ_FINE))` s
  (DS §9.52). 0.13 % is the register's *step*, not a stated accuracy, and it
  says nothing about the phase of the chip's sample clock relative to the
  STM32's — it is only the seed for the tracker in §5.3.
* Sensor specs (DS Table 3): gyro noise density 2.8 mdps/√Hz, zero-rate level
  ±1 dps, drift ±0.006 dps/°C; accel noise 60 µg/√Hz, zero-g offset ±12 mg,
  drift ±0.07 mg/°C. So a 149 Hz gyro bandwidth gives ~0.035 dps rms
  (6e‑4 rad/s), and the accel offset alone is a ~0.7° static tilt error per
  board (the SFLP has exactly the same offset today).
* I2C occupancy at one byte per ISR cycle: status read 5 cycles + gyro word
  10 cycles per 31.25-cycle gyro period, accel word 10 cycles per 8 gyro
  periods, timestamp word 10 cycles per 32 → ≈ 53 % of cycles carry a byte,
  ~2× headroom. Falling behind temporarily only delays words in the chip's
  queue (1.5 KB ≈ 219 words ≈ 200 ms at this rate); nothing is lost until the
  FIFO itself overruns, which §5.2 detects.

**Filter gains are not persistent config in v1.** Defaults (`kp`, `ki`,
`latency_comp_us`, gating thresholds) are compile-time constants; for bench
tuning they are settable at runtime through the existing `aux2` console
command path (`aux2 fusion kp 0.3`, not saved). This keeps the config-schema
change down to the enum value (§5.6). Once tuned, the values become the new
defaults in code; persistent fields can be added later with the compare-
script support described in §5.6.

### 5.2 ISR side (`AuxPort`, aux2)

Unchanged mechanics: `ISR_I2C_Update()` runs every control cycle, calls
`Stm32I2c::Poll()` (one byte per call) and checks completion. Everything new
is a state machine that issues **at most one I2C transaction at a time and
never waits**:

```
Init:   SelectMainBank -> Reset -> WaitResetClear (re-read CTRL3 each cycle, timeout 10 ms)
        -> Cfg[0..N] (one register write per transaction, table in §5.1)
        -> ReadWhoAmI -> ReadFreqFine -> Running            (~21 transactions ≈ 3 ms wall, 0 µs blocked)
Running: StatusRead (2 B at 0x1B)
        -> count = DIFF_FIFO; ovr = FIFO_OVR_LATCHED
        -> count > 0 ? WordRead (7 B at 0x78) : StatusRead
        WordRead done: gyro words get seq = ++gyro_seq_ (counted BEFORE the ring-full check);
        push {t = TIM3->CNT (fusion ticks, §5.7), seq, tag, x, y, z, flags} to the mailbox;
        --count; count > 0 ? WordRead : StatusRead
error:   existing Stm32I2c error path (NACK/timeout -> peripheral reset), then
        push a kResync entry and go back to Init (so the chip is reconfigured
        from the main bank and SW_RESET — a cable glitch cannot leave it
        half-configured or in the wrong register bank)
```

`poll_rate_us` is ignored in fusion mode: the next transaction starts on the
cycle after the previous one completes. `HandleConfigUpdate()` keeps its
`i2c_state_ = {}`; that now just restarts the non-blocking Init.

**Mailbox contract** (single producer = ISR, single consumer = main loop):

* Ring of **256 entries** × `{uint16_t t; uint16_t seq; int16_t v[3];
  uint8_t tag; uint8_t flags;}` = 3 KB, in the static block of §5.8 (`t` in
  TIM3 ticks; `seq` mod 2¹⁶ — a gap is `uint16_t(seq − prev)`, and any gap
  > 96 re-initializes anyway). At
  1 110 words/s (960 gyro + 120 accel + 30 timestamp) that is ≈ 230 ms,
  which covers a full `conf write` (≈ 100 ms, §1.4; ≈ 110 words) with
  margin. Shedding starts at 192 entries (≈ 173 ms of backlog) and the
  first gyro drop only after ≈ 240 ms — a 150 ms stall exercises neither
  (§7 test 4).
* Publication order: the producer writes the payload, then stores the head
  index with `std::atomic<uint16_t>::store(memory_order_release)`; the
  consumer loads it with `memory_order_acquire`. (Volatile indices alone do
  not order the payload writes.)
* Load shedding: above 75 % full, accel and timestamp words are dropped
  first (they are not integrated; the filter tolerates their absence);
  gyro words are dropped only when the ring is completely full.
* **Every gyro word carries the producer sequence number**, assigned when
  the word is received from the chip, *before* the ring-full decision. A
  drop is therefore always visible to the consumer as a gap in `seq`, with
  its exact size; `mailbox_overflow` counts drops. The FIFO on the chip keeps
  queuing meanwhile.
* Chip FIFO overrun (`FIFO_OVR_LATCHED` seen in a status read) sets the
  `fifo_overrun` flag on the next pushed entry; `fifo_overruns` counts. Since
  the ISR pump never stops while the firmware runs, a chip overrun can only
  follow an I2C error/resync — it is treated as a catastrophic gap (§5.3).
* Draining: `AuxPort::Poll()` (main loop) drains at most 16 entries per call
  (≤ ~32 µs) so a backlog cannot starve CAN servicing; `StartFrame()` drains
  everything that is queued (worst case 256 × ~2 µs ≈ 0.5 ms, only right
  after a main-loop stall) so the reply is current. The gyro `seq` advance
  since the previous drain is the **stall detector** (§5.7): if it exceeds
  96 (> 100 ms) the pass is a *stall pass* — `stall_passes` counts, and the
  frame being processed **plus every frame still queued in the hardware RX
  FIFO** is marked `arrival_unknown` until consumed (§5.4).

### 5.3 Main loop: the filter (`AuxPort::Poll()`, called from `MoteusController::Poll()`)

**Sample-time reconstruction.** The I2C arrival time of a word is *not* its
sample time: it lags by the poll phase, the transfer, and any backlog, and
that lag varies by up to ~0.5 ms from word to word (more after a stall). For
a reply that is finite-differenced on the Orin, a 1 ms wobble in the assumed
sample time at 2 rad/s is a 12 % blip. So sample instants come from a
**sample-clock model**, not from arrival times:

* Each gyro word with sequence number k is assigned `t_k = t_epoch + k·T`.
  `T` is seeded from `INTERNAL_FREQ_FINE` and, together with `t_epoch`,
  refined by a slow tracker (time constant ~10 s) on the arrival times:
  arrival − model has a positive floor (the transfer time); the tracker
  follows that floor, so backlog and phase noise are rejected, and only the
  chip clock's slow drift moves `T`. The phase is entirely the tracker's job
  — `FREQ_FINE` cannot provide it.
* Continuity has two independent witnesses: the producer `seq` (exact, for
  everything lost between chip and filter) and `TAG_CNT` (mod 4, a
  cross-check on the chip side). Every 32nd slot carries a chip timestamp
  word; it re-establishes the sample *index* `k` after a loss (a sample
  count in the chip's clock) and nothing else: DS Table 145 trims the ODR
  and the timestamp rate with the same `FREQ_FINE`, i.e. the same
  oscillator, so the word carries **no rate and no phase relative to the
  STM32**, and it arrives late over I2C by an unknown amount. Accel words
  are not counted; they are matched to the gyro slot they arrive in.
* Gap rules:
  * *Mailbox drop* (seq jumps by n > 1): the missing n−1 samples are
    integrated with the last known ω over (n−1)·T (rectangular), the
    trapezoid's "previous sample" is reset to the first word after the gap,
    `gyro_gaps`/`gap_slots` count. Allowed up to 100 ms of gap; longer
    → re-initialize.
  * *Chip FIFO overrun* or *resync*: the number of lost samples is unknown,
    so no integration is attempted. The filter **re-initializes** from the
    next accel word: tilt from gravity, **heading carried over from the last
    estimate** (continuous in the common case that the board was still
    during the gap; if it was moving, the heading is off by the unobserved
    integral, which is no worse than resetting it), `kp` boost. The
    orientation history is **cleared**, so no query can interpolate across
    the re-initialization. The sample-clock tracker restarts: the next chip
    timestamp word (≤ 33 ms away) re-anchors the sample index `k` exactly,
    but the **STM32 phase is not corrected by it** — that comes only from
    the arrival-floor tracker, whose uncertainty is ≈ one word transfer
    (~0.5 ms) at restart and shrinks to the ISR granularity (~33 µs) as the
    floor is re-established over ~100 ms of words; it is tracked
    explicitly as `phase_unc_us`. Output is the **sentinel** until the
    filter has converged (0.2-1 s, see "Early convergence" below), `k` is re-anchored and `phase_unc_us` <
    100 µs — **and until at least three sentinel replies have been sent**
    since the re-initialization (`sentinel_replies`, counted when a
    quaternion reply is produced — not per CAN frame, §5.4). Counting
    replies, not samples or time, is what makes the transition visible to
    the host under §1.5's assumptions: a hold measured in samples can be
    consumed by one burst drain after a stall, and a hold measured in time
    can elapse while nobody polls. Three replies survive a lost frame or
    two; the host latches them at parse time (§6). `reinits` counts. None of this is
    expected in normal operation (it needs an I2C error or a > 240 ms
    main-loop stall); on the robot the host's `stale` handling is the
    safety net while it lasts.
  * While a gap is being dead-reckoned or the tracker is re-anchoring, the
    published state carries `timing_degraded`; telemetry only (there is no
    spare wire bit).

**Gyro word (960 Hz):** `ω = raw · 70 mdps → rad/s`; `ω_c = ω − b`;
trapezoid `ω̄ = ½(ω_c,prev + ω_c)`; `q ← q ⊗ exp(½·(ω̄ + kp·e)·T)`;
normalize; append `(t_k, q, ω_c)` to the **orientation history** (ring of
64 ≈ 67 ms, consumed by §5.4).

**Accel word (120 Hz):** `a_n = a/|a|`; predicted world-up in the body
frame `g_b = (2(xz − wy), 2(wx + yz), 1 − 2(x² + y²))` (third row of R(q));
`e = a_n × g_b`, then **gated and clamped** — magnitude alone is not
enough: a 2 m/s² lateral acceleration changes |a| by 2 % but the apparent
gravity direction by 11.5°, and a limb IMU 0.3 m from a joint swinging at
5 rad/s sees 7.5 m/s² of centripetal acceleration:

* weight `w_a = f(| |a|/g − 1 |) · f(|ω|)`: 1 when |a| is within 5 % of g
  and |ω| < 0.5 rad/s, falling to 0 at 20 % / 3 rad/s (piecewise linear);
* the correction magnitude per accel word is clamped (|e| ≤ sin 5°), so a
  bad sample can only nudge;
* `kp` is small (default 0.3 → tilt time constant ~3 s), so periodic limb
  accelerations average out and only the slow gyro-bias residual is
  corrected;
* **bias learning** `b ← b − ki·e·dt_a` (ki default 0.02, |b| ≤ 0.05 rad/s)
  runs only while the board has been quasi-static (w_a = 1) for ≥ 100 ms.
  This sees only the tilt axes; the bias about the vertical is invisible
  to the accelerometer and integrates into heading (measured: 0.16°/s at
  rest on the bench board while the SFLP held 0.002°/s).
* **stationary bias learning, all axes** (added 2026-09-22): while every
  bias-corrected gyro axis reads within 0.5 dps, |a| is within 2 % of g
  and the gravity direction has moved less than 0.2° since the candidate
  window began, for 1 s, the residual rate *is* the bias error and is
  blended in with a ~10 s time constant (`stat_*` params). Walking never
  qualifies; standing and the crane do. Known limit: a pure heading
  rotation slower than 0.5 dps has no gravity signature and would be
  absorbed at the 10 s rate (bounded by the threshold). Telemetry:
  `stationary`, `stationary_words`; in comparison mode the SFLP's own
  bias estimate (`sflp_gbias_*`, FIFO tag 0x16; int16 at the ±125 dps
  sensitivity, 4.375 mdps/LSB, not float16 as the register description
  suggests) is published next to ours. First comparison on the bench:
  SFLP (−0.499, +0.149, −0.022) dps vs ours (−0.500, learning, −0.028) —
  the tilt axes agree within 0.006 dps and ST's vertical-axis value equals
  the yaw drift we had measured. The window resets only after 8
  consecutive gyro samples above 1 dps, so table vibration does not starve
  it. Result on the bench: yaw drift 0.16 → 0.006°/s (SFLP 0.0005), our
  bias (−0.502, 0.160, −0.020) dps vs ST's (−0.508, 0.162, −0.018).
* **startup capture:** a cold start begins with zero bias, and in a PI
  filter a 0.5 dps tilt-axis bias error is a 1.7° tilt error that decays
  only as fast as the bias is learned (~45 s with the innovation learner
  alone — seen on the bench as a 0.5° tilt drift in a run started seconds
  after a reboot). The first 2 s of qualified stationary time after a cold
  start therefore use a fast gain (`stat_gain_fast`, ~0.1 s), which on the
  crane removes the transient before the policy starts. Verified from a
  cold boot on the bench: bias captured within 1 s (−0.506 dps on x at
  t = 1 s, 0.165 dps on the vertical axis by 3 s), stationary from t = 1 s,
  tilt within 0.045° of the first sample over the following 30 s.

This is the standard 6-axis trade-off: sustained acceleration is
fundamentally indistinguishable from tilt; the SFLP has the same limit, and
ST's own spec for it is 1.6°/1.2° pitch/roll under high dynamics (DS
Table 1). That is the yardstick for §7.

*Init:* first accel word sets `q` from tilt (rotation taking `a_n` to
(0, 0, 1), yaw 0), `b = 0`; `kp = 10` for the first 2 s, then the configured
gains. *Published state* (main-loop owned; the CAN path runs in the same main
loop, so no lock): the orientation history (cleared at every
re-initialization), `ω_c` of the last gyro word,
`initialized`, `timing_degraded`, counters. *Frame conventions:* `q` maps
body vectors to world; world Z up; heading zero at init — the same semantics
as the game vector. **Handedness and axis signs of the raw gyro/accel versus
the SFLP frame must be verified on the bench (§7).** Cost: ~1–2 µs per gyro
word in the main loop.

### 5.4 Reply path (`MoteusController::Impl`)

**Arrival timestamp.** The reply must describe the orientation at the moment
the request *arrived*, not when the main loop got to it (hundreds of µs later,
jittering with whatever else the loop does; up to 1.1 ms spread across the
boards of one bus). The FDCAN peripheral's *internal* timestamp counter is
**not** usable for this: with bit-rate switching its time base follows the
current bit time, which differs between arbitration and data phases, so ST
requires the external counter for CAN FD (RM0440 §44.4.8). The peripheral
does, however, write a 16-bit timestamp **into every received FIFO element,
in hardware, at start of frame** (`RxTimestamp`, "captured on start of frame
reception", `stm32g4xx_hal_fdcan.h:227`; filled by `HAL_FDCAN_GetRxMessage`,
`.c:2266`), and with `FDCAN_TIMESTAMP_EXTERNAL` that value is the external
counter, which on the STM32G4 is **TIM3**.

**Mechanism (the only one planned): hardware SOF stamps from TIM3.**

* TIM3 runs as a free-running 16-bit up-counter with a **4 µs tick**
  (PSC 679 from the 170 MHz timer clock; 262 ms wrap) — the same counter
  the ISR stamps mailbox entries with (§5.7). Its clock is already
  enabled at boot (`fw/moteus.cc:117`). `FDCAN_TSCC` is a protected
  register, so the external source is selected once, in `FDCan`'s
  constructor before `HAL_FDCAN_Start` (`fw/fdcan.cc:291`), on every board;
  with TIM3 stopped the stamps are simply unused.
* **Timer inventory** (why TIM3 is available): TIM15 is the µs timer
  (16-bit, `WORKSPACE:96-100`); TIM5 is the motor PWM timer
  (`PA_0_ALT0`… → TIM5_CH1–3, `PeripheralPins.c:201-205`,
  `fw/bldc_servo.cc:468-476`); TIM2/TIM3 are the aux-pin timers, claimed
  only by `kPwmOutput` (`fw/aux_port.h:1572-1590`), hardware quadrature
  (`quad_->hwtimer()`) and BiSS-C (`fw/bissc.h:171`) on pins PB_4/PA_7
  (`fw/moteus_controller.cc:411-413`). No board in the robot configures any
  of those: `custom_sensor.cfg` has only the aux2 I2C lines, and the bench
  motor board reads `aux1.pins.{0,1,2}.mode = 0`, `aux1.spi.mode = 0`
  (2026-09-21). Confirmed on the robot in §7.2.
* **Ownership registry** (`fw/timer_ownership.h`: `Claim(TIM, owner)` /
  `Release`): the fusion claims TIM3 when its mode is enabled; `kPwmOutput`
  on a TIM3 pin, hardware quadrature and BiSS-C consult it (the existing
  quadrature-vs-PWM check at `fw/aux_port.h:1576` is the template) and fail
  with `kPwmPinError` if TIM3 is taken. If TIM3 is already owned when the
  fusion is enabled, the fusion mode **does not start**: `init_state =
  kTimerUnavailable`, a console message, the sentinel on the wire. TIM3 is
  the fusion's clock (§5.7), so there is no degraded mode to maintain.
* **Nothing to pair.** TIM3 is also the fusion time base (§5.7), so
  `RxTimestamp` is already in history units: `t_req = RxTimestamp +
  latency_comp`. Ambiguity rule: a 16-bit difference is meaningful only
  within ±131 ms, so a request that waited in the FIFO across a longer
  main-loop stall would alias. The stall is detected by **sample count, not
  by a timer**: if the gyro `seq` advanced by more than 96 (> 100 ms) since
  the previous drain, every frame delivered in that pass is flagged
  `arrival_unknown` → sentinel. Its reply would be ≥ 100 ms late in any
  case, worthless to the Orin. The mark covers the frame being processed
  **and every frame still queued in the hardware FIFO** at that moment —
  `pending_unknown = 1 + RXF0S.F0FL` (≤ 4), consumed one per frame —
  because the transport delivers one frame per `Poll()`
  (`fw/fdcan_micro_server.h:94`) and the FIFO holds three
  (`SRAMCAN_RF0_NBR`) in the HAL's default blocking mode: a pre-stall
  request consumed two iterations later would otherwise be judged by a
  per-pass detector that has gone quiet again, and after a stall longer
  than 131 ms its stamp could alias into the current history. The same
  mark is applied whenever TIM3 is started or changes owner. Nothing new
  runs in interrupt context; the motor ISR is untouched. (Whether the
  *data* is still fresh is not judged here at all: the wire toggle of §4.2
  lets the host see a stopped acquisition within three polls, so a history
  that has gone stale needs no firmware-side expiry.)
* Precision: 4 µs tick, captured at SOF, so the SOF-vs-EOF offset of v3 no
  longer exists. All boards on a bus stamp a broadcast request at the same
  SOF to within their own clock offsets.

*Rejected alternative — an RX-FIFO0 interrupt stamping in software* (v3's
primary). `RF0N` is a level flag, not an event queue: if two frames arrive
before the handler runs, stamping `F0PI − 1` mis-stamps the older one;
`RF0L` does not help; the FIFO has **three** elements (`SRAMCAN_RF0_NBR`,
`stm32g4xx_hal_fdcan.c:214`) and wraps. A correct version would have to
drain the hardware FIFO **inside the handler** into a software queue (single
owner of the FIFO), stamp each element at handler entry, flag every element
but the newest as `ambiguous` whenever the fill level at entry is > 1, and
have its entry latency *measured*, not assumed (the control ISR's ~25 µs is
one contributor; every other interrupt and masked section adds to it, so
"≤ 25 µs" was a target, not a property). Note that `fw/fdcan.cc:295`
already activates the RX-FIFO0 notification at the peripheral — nothing
enables `FDCAN1_IT0_IRQn` in the NVIC, so it is pending and harmless today
and would fire the instant the NVIC line were enabled. Not needed while
TIM3 is free.

`FDCan::Poll` passes the element's `RxTimestamp` up with each frame;
`MultiTransportDatagramServer` keeps the last one and `StartFrame()` reads
it together with the stall detector's verdict. A broadcast telemetry request then gives every
board on the bus the **same target instant**. What that does *not* give: an
exact orientation (the filter's estimate has its own error), or identity with
the Orin's user-space send time (the Orin's pacer jitter and driver latency
sit between its clock and the SOF; if the Orin wants exact spacing for its
finite difference it should difference over its own measured send times, or
over the boards' reply timing, not the nominal 1/120 s).

**Query.** `StartFrame()` (once per received CAN frame):

1. drain the mailbox (so the history is current),
2. `t_req = RxTimestamp + latency_comp` (ticks; the constant sensor
   pipeline delay, configured as `latency_comp_us`, measured in §7.2),
3. evaluate the orientation **at `t_req` from the history** — never "the
   newest state, clamped":
   * `t_req` inside the history → nlerp between the two bracketing samples
     (sign-continuous; 1 ms apart, so nlerp ≈ slerp to < 1e‑6 rad);
   * `t_req` after the newest sample by ≤ **8 ms** → extrapolate with the
     newest `ω_c`: `q_out = q_new ⊗ exp(½·ω_c·(t_req − t_new))`;
   * otherwise (newest sample more than 8 ms behind the request, or the
     request older than the history, or `arrival_unknown`) → **sentinel**;
     the history is cleared at every re-initialization (§5.3), so no
     interpolation can straddle one. This single rule
     replaces v2's separate 5 ms clamp and 50 ms staleness threshold; an
     orientation is either evaluated within the stated tolerance or not
     reported at all,
4. `words = encode_quat48(q_out)`, or the sentinel if `!initialized`, the
   filter is re-converging after a resync (until three quaternion replies
   carrying the sentinel have gone out, §5.3), the frame is in a
   `pending_unknown` set, or rule 3 fell through,
5. cache the three words. The **first `Read()` of 0x06d–0x06f in this
   frame** does the per-reply bookkeeping — `if (newest_seq !=
   last_reply_seq) { toggle ^= 1; last_reply_seq = newest_seq; }`,
   `sentinel_replies++` if the cached words are the sentinel — and ORs the
   toggle into word 0x06f; later reads in the same frame return the same
   words. `StartFrame()` itself keeps no such state, so a motor command, a
   diagnostic/config frame or a broadcast that does not read the quaternion
   (all of which run `StartFrame()` too) consumes neither the toggle nor a
   sentinel count.

All times above are TIM3 ticks (§5.7); every comparison is `int16_t(a − b)`,
never a raw `<`. The counter wraps every 262 ms, so a 10-minute bench run
crosses it ≈ 2 300 times — a wrap bug cannot hide (§7 test 10).

Why the history matters: after any processing delay (a long telemetry
emission, a `conf write`), the newest fused sample is *later* than the
request. Reporting it would reintroduce exactly the processing-time jitter
the arrival stamp removes; interpolating back to `t_req` does not.

This also replaces the broken `quaternion_cache_valid_` reset in `Read()`.
The legacy type-3 path keeps its behaviour, with the cache reset moved into
`StartFrame()` as well.

### 5.5 Telemetry

Extend the aux status with a fusion block: `q[4]`, `bias[3]`, `omega[3]`,
`odr_actual_hz`, `gyro_words`, `accel_words`, `gyro_gaps`, `gap_slots`,
`mailbox_overflow`, `fifo_overruns`, `resyncs`, `timing_degraded`,
`init_state`, `history_age_us`, `phase_unc_us`, `stall_passes`,
`arrival_unknown`, `pending_unknown`, `sentinel_replies`, `reinits`,
`storage_owner`; and in `servo_stats` an
`isr_max_cycles`
max-hold (DWT cycle count from ISR entry to exit, reset on read) so §7 can
measure the ISR instead of inferring it.

### 5.6 Config and flash safety — read this before flashing anything

Adding a value to the device `type` enum changes the schema of every group
that contains it. `IsEnum<aux::I2C::DeviceConfig::Type>` is specialized
(`fw/aux_common.h:650`), mjlib's schema archive serializes the enum's
value/name list (`BinarySchemaArchive::VisitEnumeration`, mjlib
`telemetry/binary_write_archive.h:246-256`), and `PersistentConfig` keys each
stored group by a CRC of that schema. `aux1` and `aux2` both contain
`aux::I2C::Config`, so **`kLsm6dsv16xFusion` changes the schema CRC of both
`aux1` and `aux2`**. Consequences:

* A bare SWD flash of this firmware makes the boot loader discard **both**
  groups: `aux2` (IMU config) *and* `aux1` — which on the motor boards holds
  the SPI encoder configuration. The motor would come up without its
  position source. `motor` and `motor_position` (the calibration) are not
  touched, but "calibration survives" is not sufficient here.
* `moteus_tool --flash` over CAN captures every group as text before
  flashing and replays it afterwards, so the normal
  `update_robot_firmware.py` procedure is safe **provided its compare step
  is tightened first**: today it only counts added keys
  (`motor_scripts/update_robot_firmware.py:334-335`). It must fail on any
  unexpected added key and on any expected added key that is not at its
  default. With v1 adding no persistent fields, the expected-added set is
  empty and every `aux1.*` / `aux2.*` value must compare equal.
* If persistent fusion fields are added later, remember that `DeviceConfig`
  is shared: each new field appears under **both ports and all three device
  slots** (`aux1.i2c.devices.{0,1,2}.*` and `aux2.i2c.devices.{0,1,2}.*`),
  and the compare script's expected-added map must list all of them with
  their defaults.
* The bench board gets flashed the same way — no SWD shortcuts for this
  change, and verify `aux1.*` and `aux2.*` after every flash.

Mode selection: new enum value `kLsm6dsv16xFusion` (= 5, appended before
`kNumTypes`, name `"lsm6dsv16xFusion"` in the `IsEnum` map). Legacy type 3
keeps working so that flashing and switching are separate steps (§8).
Register names/docs: `docs/protocol/registers.md` 0x06d–0x06f must be
rewritten for the new meaning; `lib/python/moteus/protocol.py` and
`moteus_protocol.h` keep the register numbers.

### 5.7 Time base: TIM3 ticks, 16 bits, and why that is enough

The mbed microsecond timer is overridden to **TIM15**, 16 bits wide
(`WORKSPACE:96-100`: `MBED_US_TIMER_TIM = TIM15`, `TIM_MST_BIT_WIDTH = 16`),
so `MillisecondTimer::read_us()` wraps every 65.536 ms
(`fw/millisecond_timer.h:39-60`); every existing consumer forms short deltas
only (`fw/stm32_i2c.h:207`, `fw/moteus.cc:359`, `fw/aux_port.h:777`). v4
reacted by building a 32-bit software clock. That was unnecessary:
**nothing in this design measures a long interval.** The board never times
the Orin's poll spacing; each reply compares that request's own stamp with
fused samples at most 8 ms away. Every long duration — the 100 ms gap
limit, the 2 s warm-up, the 1 s convergence, the
100 ms quasi-static window — is a **sample count**, not a timer reading.

**The clock.** TIM3, free-running 16-bit up-counter, **4 µs tick, 262 ms
wrap**. It must run anyway as the FDCAN external timestamp counter (§5.4),
so using it everywhere means the CAN stamp arrives in history units with
nothing to pair. The ISR stamps mailbox entries with `TIM3->CNT` (one APB
read, in place of the `read_us()` it does today at `fw/aux_port.h:721`);
the history stores the same ticks. Resolution: 4 µs against an ISR
granularity of 33 µs and a 0.002° effect at 10 rad/s — nothing lost against
TIM15's 1 µs.

**Wrap rules.** Every comparison is `int16_t(a − b)`, never a raw `<`; the
result is meaningful only while the true difference is within **±131 ms**.
There are exactly three comparisons, and each is bounded:

1. *Tracker residual*: a mailbox stamp minus the model's sample time for
   the same `seq`. The ISR takes the stamp at I2C completion, ~0.3–0.8 ms
   after the sample, no matter when the main loop gets around to the entry
   — a main-loop stall does not change it. Always < 1 ms.
2. *Request vs history*: bounded by the **stall detector**, the gyro `seq`
   advance since the previous drain. ≤ 96 (≤ 100 ms) means the request is
   at most ~101 ms old and the difference is exact; > 96 means the frame
   is `arrival_unknown` → sentinel (its reply would be ≥ 100 ms late in any
   case). No timer is involved in detecting the stall.
3. *Extrapolation*: ≤ 8 ms by rule.

One **latch** and one **wire signal** complete the argument, so that
validity is never established by a timestamp comparison alone (a wrapped
counter can make an old value *look* recent; it cannot clear a latch or
flip a toggle):

* `pending_unknown` (§5.4): every CAN frame queued at the moment a stall
  pass is detected stays unknown until consumed, so a pre-stall request
  processed two loop iterations later is not judged by a per-pass detector
  that has since gone quiet.
* The **freshness toggle** (§4.2): if gyro words stop, the newest history
  sample stops changing, the toggle stops flipping, and the host flags the
  board stale within three polls — whatever the payload says. In normal
  operation the IMU streams continuously and the toggle flips on every
  reply; it exists for a chip reset, a stuck state machine or a
  configuration mistake. v6 handled that case with an ISR no-data watchdog
  that also re-initialized the chip; v7 drops it as a complicated answer
  to a case we are not designing for (a chip that silently reset stays
  stale until `conf set aux2.…` or a power cycle re-runs Init; resync on
  I2C errors is unchanged).

With these, comparison 2 only ever runs for a frame that arrived after the
previous non-stall drain (≤ 100 ms), and the host only acts on a reply
while the history behind it is advancing — under §1.5's assumptions, every
difference the host acts on is < 131 ms. The residual: in the dead-acquisition case the *wire* may
carry a stale but valid-looking payload once the 16-bit comparison aliases
(≥ 262 ms after the last word); every consumer goes through the one parser
that honours the toggle, so nothing acts on it. (Should a clean wire ever
be wanted regardless, a main-loop count of iterations since the last
drained gyro word — two lines, no ISR change — can force the sentinel after
~20 ms. Not in v7.)

The sample-clock model lives in sample-index space: `t_k = t_anchor +
(k − k_anchor)·T`, `T` in Q16 ticks (≈ 260.4167), the anchor re-based every
256 samples (`t_anchor += 256·T`, carrying the fraction), so `t_k mod 2¹⁶`
always comes from a recent anchor — the phase-accumulator code any wrapping
clock needs.

**What the 16-bit choice removes.** The software clock extension, the
seqlock and the dual-counter pairing of v4; three quarters of the history
(64 samples suffice, since it no longer has to reach back across a `conf
write`, §5.8); and the 71-minute rollover test — the wrap now happens ~4
times a second on the bench, so a wrap bug shows up in the first minute
(§7 test 10) — though normal wraps exercise only the arithmetic, not the
latch and the toggle, which get their own tests (§7 tests 12–13). Existing users of
TIM15 are untouched.

(Aside, pre-existing and out of scope: the main-loop watchdog at
`fw/moteus.cc:359` forms its delta on the 16-bit TIM15, so a stall of
66 ms reads as ≈ 0.5 ms and evades the 4 ms fault.)

### 5.8 Memory plan

Measured on the current build (`bazel-bin/fw/moteus.elf`, 2026-09-17,
DWARF) and on bench board 2 (2026-09-21):

| item | size |
|---|---|
| object pool (`fw/moteus.cc:206`) | 24 000 B; **1 184 B available** (`system_info.pool_available`) |
| `MoteusController::Impl` (pool; contains both `AuxPort`s and `MotorPosition`) | 13 272 B |
| `AuxPort` (× 2) | 5 016 B each |
| `BldcServo::Impl` (pool) | 2 624 B |
| `main()` stack frame (the pool is a local of `main`) | 27 524 B |
| `.data` + `.bss` | 4 992 B |
| `.ccmram` (ISR code, in the 32 KB CCM SRAM) | 31 072 B — 1 696 B spare |
| main SRAM (`STM32G474XE.ld`: 0x20000200, 96k − 0x200) | 97 792 B |

New buffers: mailbox 256 × 12 B = **3 072 B**; orientation history 64 ×
20 B (`uint16_t t` + `float q[4]`, padded; `ω_c` is kept only for the
newest sample) = **1 280 B**; filter + tracker state ≈ 200 B. Total ≈
**4.5 KB**. They can be neither `AuxPort` members (two ports → 9 KB, and
`Impl` would no longer fit the pool at all) nor pool allocations (1.2 KB left; `Pool::Allocate`
asserts on exhaustion, `mjlib/micro/pool_ptr.cc:29`, i.e. the board would
not boot). They live in **one static block** (`fw/imu_fusion_storage.cc`,
`.bss`), claimed at config time by whichever port enables the fusion
(`FusionStorage::Claim(port)`; a second claimant gets `kI2cDeviceError` and
a console message; `Release` on mode change). `AuxPort` gains a pointer and
< 200 B of state, so `Impl` grows < 400 B and `pool_available` drops to
≈ 800 B — that is the one pool cost, and it is checked in §7 test 3. After
the change `.bss` ≈ 9.5 KB; heap + stack keep ≈ 86 KB, of which `main`'s
frame is 27.5 KB; the rest is newlib/mbed heap and ISR/nested-call stack,
whose high-water mark is measured once with a stack paint in a debug build
(fill from `__HeapLimit` to the boot SP, scan after the full §7 run).

---

## 6. Orin changes (`humanoid3`)

The wire bytes do not change (`0x14 + 3, 0x6d`; the pending 0x72 → 0x6d
edit still applies), but the change is **not** confined to the decoder:

* **CAN parser** (`orin/can_utils.py:796-807`): today each of the three
  register words is converted to a half-float *at parse time*
  (`_f16_from_u16`) and stored as a float in the joint-state array; the
  driver then replaces non-finite values with 0
  (`orin/rl_api/driver.py:312`). A packed word such as `0x7fff` is NaN as
  fp16 and would be destroyed before any decoder sees it. Change: store the
  three **raw unsigned 16-bit words** (exactly representable as float32, so
  the array layout can stay) and do no conversion here.
* **Driver** (`orin/rl_api/driver.py:get_imu_board_quats_wxyz`): assemble the
  48-bit value from the three words and `decode_quat48` (vectorized numpy
  port of the reference implementation); the all-zero sentinel → hold the
  previous quaternion and raise a per-board `stale` flag instead of
  zeroing; latch a per-board `reinit` flag on every stale→valid transition,
  cleared when the consumer reads it. Remove the `isfinite → 0` replacement
  on this path.
* **Discontinuity rule (required, not optional).** A firmware
  re-initialization changes the reference frame (tilt re-seeded; after a
  long gap the heading is arbitrary) while the host holds the previous
  quaternion through the sentinel. On resume, the finite difference in
  `orin/deploy/canmonitor_runner.py:658-671` would read the frame change as
  rotation: a 90° change over one 120 Hz interval is **188 rad/s** before
  the α = 0.5 low-pass (the shortest-arc reduction in
  `ang_vel_from_quat_pair` clips at π per step = 377 rad/s; it does not
  remove it). So for every board whose `reinit` flag is set the obs builder
  resets that board's derivative baseline (`prev_board_quats[i] =
  board_quats[i]`, FD = 0) and its low-pass state (`ang_vel_filt[i] = 0`),
  exactly as `_post_reset_builds` already does for all boards after an
  episode reset. The firmware holds the sentinel until **three quaternion
  replies carrying it have gone out** (§5.3) — counted in replies, not
  time, so it is neither consumed by a burst drain nor elapsed while the
  host is not polling; the host sees it unless all three are lost (§1.5),
  and the outage reset below is the backstop for that. Two host-side rules
  complete it: (1) **latch at parse time** — the CAN parser sets per-board
  `stale_seen` and, on a stale→valid transition, `reinit`, and only the obs
  builder clears them, so a later valid reply cannot overwrite the event
  before it is consumed; (2) **outage reset** — a board whose last valid
  reply is older than two poll periods on the host's monotonic clock (lost
  CAN frames, `docs/can_frame_loss_2026-08-21.md`; a paused host process;
  a board that re-initialized while nobody polled) gets the same baseline
  and low-pass reset. The FD then never spans an outage, whatever the
  firmware did during it.
* **Sign convention.** `decode_quat48` returns the quaternion with its
  largest-magnitude component positive, so consecutive samples flip sign
  whenever the largest component changes. `ang_vel_from_quat_pair` already
  negates `q_cur` when `dot(q_prev, q_cur) < 0` (`orin/imu_common.py:257`),
  and gravity/rot6d are sign-invariant; any other consumer that differences
  raw quaternions must do the same.
* **Freshness toggle (bit 47).** The parser masks bit 47 before decoding
  and keeps, per board, the last bit and a repeat count over *valid*
  replies. Three identical bits in a row (≈ 25 ms at 120 Hz) → `stale`,
  handled exactly like the sentinel; a sentinel clears the toggle state.
  Why three and not two: the toggle is per board, not per poller, so a
  second consumer polling the same board (canmonitor beside the driver)
  flips it between two driver polls, and a single lost reply frame also
  makes two consecutive bits equal; neither can produce three. With one
  poller (§1.5) a halted acquisition is flagged within three polls with no
  firmware logic beyond one XOR, and `stale`→fresh raises `reinit` like any
  other stale→valid transition. The outage reset stays in place regardless.
* **Sim backends** (`orin/mj_can_utils.py`, `orin/fastphys_can_utils.py`,
  `_get_orientation_xyz_for_actuator`): produce the three raw words with
  `encode_quat48` so the emulated CAN path is bit-identical to hardware.
* **canmonitor** (`orin/canmonitor.py:_gravity_from_quat_xyz` and the
  display columns): decode the words before computing gravity.
* **`orin/imu_common.py`**: `encode_quat48` / `decode_quat48` replace
  `encode_game_quat_xyz` / `decode_game_quat_xyz` as the wire round trip
  used by the trainer's emulation; the fp16 stage and the `w ≥ 0`
  canonicalization go away. The trainer's `SFLP_ODR_RANGE_HZ` ZOH stage goes
  away once the firmware evaluates the orientation at the request instant.
* `ang_vel_from_quat_pair` and the α = 0.5 low-pass stay for now (payload is
  orientation-only). Expect a much cleaner rate: no fp16 steps, no 0.52×
  blips. The remaining rate jitter is the Orin's own request spacing; if it
  matters, difference over measured send times instead of the nominal dt.
* Unit tests on the §4.4 vectors in `orin/tests/test_imu_common.py`, plus a
  parser test that `0x7fff` survives. Contract version bump.
* Sensor-model constants in `orin/robot_metadata.py` (fp16/ZOH/delay) need a
  re-measurement pass after bring-up (§7).
* Later phase, if wanted: gyro on the wire as 3 × int16 at 0x065–0x067 (free
  below 0x80, its own short-opcode subframe), scale 0.001 rad/s per LSB;
  then `imu_ang_vel` comes from the gyro and the FD path is retired.

---

## 7. Validation plan

### 7.1 Bench (motor id 2, board IMU on aux2; nothing on the robot)

1. FIFO bring-up: tag values as expected, 960 ± 1 % gyro words/s against
   `odr_actual_hz`, 120 accel words/s, timestamp words every 32 slots,
   `error_count 0`, `fifo_overruns 0`, `mailbox_overflow 0` over 1 h.
2. **ISR timing by measurement, not inference** (`isr_max_cycles`, DWT):
   in zero-torque position mode (`d pos nan 0 0`) it must stay below the v1
   baseline (25.5 µs ≈ 4 335 cycles) plus the byte-pump cost, and it must
   not move during: a mode switch (`conf set aux2.i2c.devices.0.type 5`,
   re-init while running), an induced I2C error (set a wrong `address`, then
   restore), and a runtime gain change. `servo.timing_fault` may be on for
   these, since none of them stall the main loop.
3. **Persistence, separately**: `conf write` during operation. The main-loop
   watchdog (`fw/moteus.cc:359-366`) *will* fault the servo if
   `servo.timing_fault` is on — that is pre-existing and not this design's
   concern — so run it with `timing_fault` off. Requirements: `isr_max_cycles`
   unchanged (the ISR keeps pumping through the stall), the mailbox holds
   the whole save (`mailbox_overflow 0`, expected ≈ 110 words queued), no
   gyro gap (`gyro_gaps 0`), and the reply during/after the stall obeys §5.4
   (sentinel while the history cannot cover the request, then a continuous
   orientation with no jump > 0.1° at rest; `stall_passes` 0 or 1 — a
   ≈ 100 ms save straddles the 96-word threshold and either is correct).
   Memory (§5.8): `pool_available`
   must drop by < 400 B from the pre-change 1 184 B, `.bss` must grow by
   ≈ 4.5 KB (`arm-none-eabi-size`), and the debug-build stack paint must
   show ≥ 8 KB of headroom.
4. Gap contract, three parts. (a) *Exact drops*: debug command
   `aux2 fusion drop N` makes the producer skip N gyro words while still
   advancing `seq` — the consumer must dead-reckon exactly N slots and
   count them; orientation error across the gap at rest < 0.1°. (b)
   *Shedding and overflow*: stall the consumer for ≥ **400 ms** (a 150 ms
   stall queues only ≈ 167 words — below both the 192-entry shedding point
   and the 256-entry capacity — and would exercise nothing): expect a
   stall pass (`stall_passes` + 1, any CAN frame delivered in it
   `arrival_unknown`), accel/timestamp shedding first, gyro drops with exact `seq` gaps after
   ≈ 240 ms, then a re-initialization per §5.3 — heading carried over
   (< 0.5° change at rest), history cleared, three sentinel replies,
   `reinits` = 1. (c) *Init recovery*: force an I2C error mid-init (wrong
   address after the bank switch) — recovery must start with
   `FUNC_CFG_ACCESS = 0` and reach `Running` (checked via `init_state`).
5. **Accuracy against an independent reference, away from the SFLP's
   singularity.** Static: the board on a machinist's angle block / levelled
   fixture at 0°, ±30°, 90° tilt in two axes — projected-gravity error
   < 0.5° after convergence (the accel zero-g spec alone allows ~0.7°; log
   the per-board offset). SFLP comparison only where the SFLP is trustworthy
   (|w_SFLP| > 0.3), after aligning the two arbitrary headings, expecting
   < 0.5° agreement in tilt; near w ≈ 0 the SFLP is *expected* to disagree
   by up to ~1.7° (§1.2) and ours must not.
6. Quantization: slow hand rotation through the old worst-case orientations
   shows steps ≤ 0.01°, no 3° jumps.
7. Wire: Python reference round trip within the 0.0086° bound on 1 M random
   orientations; **firmware float32 encoder** unit test (`fw/test`) producing
   the §4.4 words bit-exactly and a round trip through the Python decoder
   within the bound on the same random set; Orin parser test that `0x7fff`
   survives and that bit 47 is masked and tracked (three-in-a-row rule);
   `check_robot_imus.py` updated and run against the bench board.
8. Drift: board still for 10 min — tilt stable to < 0.2°, yaw drift rate
   logged (unobservable, informational). Then the "swing" test: the board on
   a ~0.3 m arm swung by hand at ~1 Hz for 30 s — tilt error afterwards
   < 1.5° (ST's own high-dynamic spec), bias must not have moved by more
   than 0.2 dps.
9. Request-time evaluation and the stamp itself. With the TIM3 SOF stamps
   in, request the quaternion at deliberately irregular intervals (and with
   an artificial main-loop delay in a debug build) while the board is
   still, then while hand-rotated — the reported orientation must depend
   only on the SOF instant, never on the processing delay; the FD rate must
   be free of the poll-phase blips. Verify the stamp: with BRS **off** the
   FDCAN *internal* counter is valid, so log `RxTimestamp` from both sources
   over 10⁴ frames and expect a constant offset ± 1 tick; with BRS on, a
   scope on CAN-RX against a GPIO toggled in `Poll()` bounds the offset.
   Then the stall detector: a forced 150 ms main-loop stall (Δ`seq` > 96)
   must produce `arrival_unknown` and the sentinel for the frames delivered
   in that pass, never a wrong orientation; a 50 ms stall must not.
10. Clock wrap. Nothing to force: TIM3 wraps every 262 ms, so a 10-minute
    run at 120 Hz crosses it ≈ 2 300 times with requests landing on both
    sides. Assert zero sentinels and no jump > 0.05° at rest over the run,
    and that the tracker's `T` estimate is continuous across wraps. Plus a
    host unit test of the query/tracker arithmetic with tick values
    straddling 0xffff.
11. Recovery end to end, from the Orin-side code path against the bench
    board at 120 Hz: force a re-initialization; the host must see ≥ 3
    sentinel replies, latch `reinit`, reset the FD baseline and low-pass,
    and show no |ω| spike > 1 rad/s at rest. Repeat with the host paused
    for 200 ms around the re-initialization (outage reset) and with the
    re-initialization forced *during* a 150 ms main-loop stall (burst
    drain) — same assertions.
12. Acquisition stopped — a toggle test, not an arithmetic one. Debug
    command `aux2 fusion halt_acq` stops the ISR's word reads while I2C
    keeps answering. Polling at 120 Hz, the host must flag the board
    `stale` within three polls and **must not accept any reply as fresh**
    for the whole halt — 300 ms and 600 ms, past 262.144 ms, where the
    firmware's 16-bit comparison may alias and the payload can look valid
    again; the toggle must not flip once during the halt. On resume:
    toggling resumes, `reinit` is raised, the FD baseline resets, heading
    continuous with before the halt. Repeat with the halt produced by
    switching the chip's `FIFO_CTRL4` to bypass through a raw-I2C debug
    command — the case of a chip that reset to defaults — and confirm that
    `conf set aux2.i2c.devices.0.type 5` (re-running Init) recovers it.
13. Queued requests across a stall. With the main loop held by a debug
    command for 140 ms, 270 ms, 131.072 ± 2 ms and 262.144 ± 2 ms, the
    bench script sends three back-to-back requests 1 ms into the stall and
    a fourth 1 ms after it ends. All three pre-stall replies must be
    sentinels (`pending_unknown` 3 → 0) and the fourth valid; no reply may
    ever carry an orientation older than its request. Also after a
    `conf set aux2.…type 5` re-enable (TIM3 restart) with two requests
    queued.

### 7.2 Robot (on the crane — requires an explicit go-ahead first)

12. `check_robot_imus.py` on all 48 boards (read-only), plus `aux1.*` /
    `aux2.*` config verification on every board after flashing (§5.6) —
    which also confirms that no board has PWM output or quadrature on aux
    pins 1–2, i.e. that TIM3 is free everywhere (§5.4).
13. Latency vs output encoder with the existing chirp rig
    (`orin/calib/imu_lag_fit.py`, motor-driven — robot moves): expect ~1 ms
    residual instead of 7.3; set `latency_comp_us` from it.
14. Re-fit the trainer's IMU sensor model (gyro-based dynamics, delay) on
    delay-corrected captures, including a walking-speed limb-swing capture
    for the accelerometer-gating parameters.

### 7.3 Things that must be verified on hardware, not assumed

* Timestamp-word byte layout; that a 7-byte read from 0x78 returns one word
  and advances (today's code reads 0x79–0x7E only); behaviour of reading an
  empty FIFO (should never be needed, but must be harmless).
* `SW_RESET` clear time. That `FUNC_CFG_ACCESS` is reachable from the other
  banks is supported by ST's documentation, not only by its driver: the
  datasheet's own page-write procedure (DS p. 145, step 7) writes
  `EMB_FUNC_REG_ACCESS = 0` in `FUNC_CFG_ACCESS` *while the embedded bank
  is selected*, and AN5763 (pp. 68, 81–82) exits both the embedded and the
  sensor-hub bank the same way. The bench test checks our implementation
  of it, not the chip.
* Raw gyro/accel axis conventions relative to the SFLP frame.
* Actual ODR vs `INTERNAL_FREQ_FINE` (the register is a trim value in
  0.13 % steps, not an accuracy spec — measure the residual).
* The FDCAN external timestamp source is closed from documentation: RM0440
  §44.4.8 names it as "TIM3 value (tim3_cnt[0:15])". The bench still
  verifies the `TSCC` configuration, the 4 µs tick, the SOF capture, and
  the constant offset against the internal counter with BRS off (§7 test 9).
* That TIM3's kernel clock is 170 MHz on this board (PSC 679 → 4.000 µs);
  a 10-minute comparison of TIM3 against the chip's ODR must show only the
  two crystals' difference (the tracker's `T` stable to < 1e-4).
* Accel ±4 g adequacy under foot impacts (gating handles saturation, but log it).
* Filter gains on the real limbs.

---

## 8. Rollout

1. Tighten the update script's compare step (§5.6), then land the firmware
   with both paths (type 3 legacy, type 5 fusion) and the §5.8 memory checks
   passed. Flash all boards with
   `update_robot_firmware.py` per
   `humanoid3/motor_scripts/README_firmware_update.md` — **over CAN only,
   because both aux groups change schema** — and verify every `aux1.*` /
   `aux2.*` value unchanged. Nothing changes on the wire yet.
2. Land the Orin parser/driver/sim changes with a switch keyed on the
   board's mode (or a contract version), tested on the bench board.
3. Switch boards to `type 5` bus by bus (`conf set aux2.i2c.devices.0.type 5`,
   `conf write`, from a small script) with the Orin decode switched at the
   same time; `check_robot_imus.py` after each bus. Rollback is
   `conf set … type 3` (the legacy init now resets the chip, so this is
   clean) — no reflash.
4. Only then retrain with the new sensor model.

---

## 9. Expected latency after the change

| stage | today (measured 7.3 ms total) | new (estimate) |
|---|---|---|
| chip fusion / filter group delay | ≈4 ms (SFLP) | ~1–1.5 ms (gyro LPF1 149 Hz + LPF2 342 Hz at 960 Hz; constant, compensable via `latency_comp_us`) |
| sample age (ZOH) | 2.1 ms mean | 0 — the orientation is evaluated at the request instant from the history |
| polling / transfer | 0–4 ms (2 ms two-step) | not on the critical path once the sample clock is modelled (bounded ≤ 8 ms extrapolation, else sentinel) |
| request → snapshot | processing latency, up to ~1 ms across a bus | 4 µs tick, captured by hardware at SOF (all boards on a bus target the same SOF of a broadcast request) |
| main-loop fusion | – | drained at `StartFrame()`, so 0 |
| residual, uncompensated | 7.3 ms ± 2 | ≈ estimation error + Orin send-time jitter; to be measured, §7.2 |

---

## 10. Review log

### v1 → v2 (2026-09-21)

| # | Finding on v1 | Verified | Change in v2 |
|---|---|---|---|
| 1 | Arrival-time stamps are not sample times; `StartFrame()` is processing time | yes | Sample-clock model; anchor on frame arrival |
| 2 | Adding the enum value changes the `aux1` and `aux2` schema CRCs | yes — `IsEnum` map is serialized into the schema | §5.6 rewritten; CAN-only flashing, `aux1` verification |
| 3 | Mode switches leave stale chip state (SFLP on / raw batching on) | yes | Init starts with `SW_RESET` and writes every mode-dependent register in both modes |
| 4 | The Orin parser fp16-converts at parse time; `0x7fff` → NaN → 0 | yes | §6 moves the change to the parser; raw words carried through |
| 5 | Blocking init is re-run on config changes; `final_timer` cannot show whole-period overruns | yes | Non-blocking init; `isr_max_cycles` DWT measurement |
| 6 | No mailbox/FIFO loss contract; 16 entries ≈ 15 ms; volatile ≠ publication order | yes | Larger ring, release/acquire, overflow/overrun flags, bounded draining |
| 7 | |a|-only gating passes lateral acceleration | yes | |a| and |ω| weighting, innovation clamp, low kp, quasi-static bias learning |
| 8 | Script's SFLP model un-normalized; "0.0073°" was a sampled max | yes | Script fixed; analytic bound as the acceptance figure |

### v2 → v3 (2026-09-21)

| # | Finding on v2 | Verified | Change in v3 |
|---|---|---|---|
| 1 | The FDCAN internal timestamp counter is not a constant-rate clock under CAN FD BRS; ST requires the external counter (TIM3), which aux1 pin modes and BiSS-C can own | TIM3 claims confirmed in `moteus_controller.cc:411-413`, `bissc.h:387` | §5.4: RX-interrupt stamp with the 32-bit µs timer as the primary mechanism; TIM3 as the alternative with an ownership check |
| 2 | Draining then clamping Δt ≥ 0 reports a state *after* the request; 5 ms clamp vs 50 ms staleness were inconsistent; SOF is a common instant, not an exact orientation nor the Orin's send time | yes | §5.4: orientation history, interpolation at the request instant, ≤ 8 ms extrapolation, single sentinel rule; claims reworded |
| 3 | The `conf write` test cannot pass with `timing_fault` on (main-loop watchdog at 4 ms, `moteus.cc:359-366`); the erase is two pages (44–49 ms) plus programming; 65 ms counter wrap | yes | §1.4 budget ≈ 100 ms per save; §5.2 ring 256; §7 persistence test separated from ISR timing and run with `timing_fault` off; no 16-bit counter in the primary path |
| 4 | `SW_RESET` at 0x12 is `EMB_FUNC_STATUS` if the embedded bank is selected | yes (DS Table 262) | §5.1 step 0: `FUNC_CFG_ACCESS = 0` first, on every init and recovery |
| 5 | Mod-4 `TAG_CNT` cannot see a loss of 4 words; a sticky flag gives no count; timestamp words may be 33 ms away | yes | §5.2 producer sequence numbers assigned before the drop decision; §5.3 exact dead-reckoning for mailbox drops, re-init + sentinel for chip overruns/resyncs |
| 6 | New `DeviceConfig` fields appear under both ports and all slots; the update script only counts added keys | yes (`update_robot_firmware.py:334-335`) | §5.1/§5.6: no persistent fields in v1 (runtime console override for tuning); compare step must enforce expected-added keys with defaults |
| 7 | SFLP agreement < 0.5° cannot be required near w = 0 (its own error there is ~1.7°) | yes (script: 1.68° for (0, 1/√2, 1/√2, 0)) | §7 test 5: independent static reference; SFLP comparison only for |w_SFLP| > 0.3 after heading alignment |
| 8 | The bound's derivation used a first-order step as strict and had the chord/angle inequality reversed | yes | Script: rigorous finite-perturbation derivation, `θ ≤ 2·asin(|e|/(1−|e|))`; 0.0086° stands (independent value 0.0085674°); firmware float32 gets its own test |

### v3 → v4 (2026-09-21)

| # | Finding on v3 | Verified | Change in v4 |
|---|---|---|---|
| 1 | The RX-interrupt stamp is unreliable as written: `RF0N` is a level flag, two frames before the handler mis-stamp the older one; the 3-element FIFO wraps; "≤ 25 µs" was an unverified target | yes — and a larger premise error surfaced while checking it: the firmware has **no 32-bit µs timer** (TIM15, 16-bit, wraps every 65.5 ms; `WORKSPACE:96-100`) | §5.4: hardware SOF stamps from TIM3 as the FDCAN external counter — race-free, no new interrupt, ownership registry, timer inventory verified on the bench board; the software-interrupt design is now a rejected alternative with the protocol it would need. §5.7: `FusionClock`, a 32-bit extension of TIM15 kept by the control ISR |
| 2 | Recovery resets the reference frame while the host holds the old quaternion; the host's finite difference then reports up to 188 rad/s; the history could interpolate across a reset | yes (`canmonitor_runner.py:658-671`, `imu_common.py:257`) | §5.3: heading carried over, history cleared, sentinel held ≥ 50 ms; §6: mandatory `reinit` flag with per-board baseline and low-pass reset; §7 test 11 |
| 3 | 12 KB of buffers with no allocation plan; the pool is 24 000 B and `Impl` 13 272 B; `AuxPort` members would be allocated twice | yes — and the bench board has only 1 184 B of pool left | §5.8: one static block outside the pool, claimed by the fusion port; sizes and headroom measured; checks in §7 test 3 |
| 4 | A chip timestamp word does not correct the STM32 phase | yes (DS Table 145: the ODR and the timestamp rate share one trimmed oscillator; delivery over I2C is late by an unknown amount) | §5.3: the word re-anchors the sample index only; phase from the arrival-floor tracker with explicit `phase_unc_us` |
| 5 | The 150 ms overflow test cannot overflow (≈ 167 words < 192 shedding < 256 capacity) | yes | §7 test 4: injected drops plus a ≥ 400 ms stall |
| 6 | "The 32-bit clock wraps every 71.6 min" | the premise was shared with v3: the hardware clock wraps every 65.5 ms; 71.6 min applies to the new `FusionClock` | §5.7: signed 32-bit differences; §7 test 10 |
| – | `fw/moteus.cc:171` concerns a 1 ms `Ticker`, not a ban on CAN interrupts; the bank exit through `FUNC_CFG_ACCESS` is documented (DS p. 145, AN5763) | yes | Neither is cited as a constraint any more; §7.3 updated; no interrupt is added anyway |

### v4 → v5 (2026-09-21)

| # | Question on v4 | Verified | Change in v5 |
|---|---|---|---|
| 1 | Why a 32-bit clock at all? Wrapping has to be handled either way, and nothing times an interval near the 16-bit range (Otavio) | yes: every long duration is a sample count; the only long interval is a request or stamp waiting across a `conf write`, and the right treatment is a sample-count stall detector plus the sentinel, not a bigger clock | §5.7 rewritten: TIM3 at 4 µs is the one 16-bit clock (ISR stamps, history, CAN stamps — nothing to pair); `int16_t` differences with three bounded comparisons; `FusionClock`, seqlock and dual-counter pairing deleted; history 64 samples; static block 4.5 KB (§5.8); fusion mode requires TIM3 instead of carrying a degraded mode; tests 3, 4, 9, 10 and §7.3 adjusted |

### v5 → v6 (2026-09-21)

| # | Finding on v5 | Assessment | Change in v6 |
|---|---|---|---|
| 1 | The sample-count stall detector proves nothing when gyro words stop; after 262.144 ms a stale history aliases back into "valid" | Correct as arithmetic, but the case only arises when acquisition dies — and that is not a wrap problem, it is a **missing self-healing rule**: a chip that answers I2C but has reset to defaults, or a stuck state machine, would deliver a stale orientation forever. Otavio's point stands for normal operation: with a streaming IMU the detector is exact | §5.2: ISR no-data watchdog (`cycles_since_gyro_` > 300 → resync + re-init); `acquisition_live` latch gates every comparison (§5.4, §5.7). ~5 lines; also makes a misconfigured FIFO loud during development |
| 2 | A stall pass invalidates only the frame delivered in it; the transport reads one frame per `Poll()` and the FIFO holds three, so a pre-stall frame consumed later escapes the check | Correct, and independent of IMU regularity — a real hole in v5's per-pass rule, reachable for stalls > 131 ms (a long `conf write`) | §5.4: mark the frame being processed plus every frame queued in the hardware FIFO (`pending_unknown = 1 + RXF0S.F0FL`), kept until consumed; same at TIM3 start / owner change. ~3 lines |
| 3 | A 50 ms sentinel hold does not guarantee host observation: a sample-counted hold is consumed by a burst drain, a blocked loop sends nothing, CAN frames get lost; the host also needs a reset after its own outages, with events latched at parse time | Correct; the sample-counted hold was a v5 regression | §5.3: hold until three sentinel **replies** have been sent; §6: latch at parse time, outage reset after two missed poll periods; §7 tests 11–13 |
| – | Normal wraps do not test these cases; RM0440 §44.4.8 names the external source as `tim3_cnt[0:15]` | yes | §7 tests 12–13 target the latches around 131.072 / 262.144 ms; §7.3 closes the source question and keeps the bench check of configuration and tick |

### v6 → v7 (2026-09-21)

| # | Point on v6 | Assessment | Change in v7 |
|---|---|---|---|
| 1 | The ISR no-data watchdog is a complicated answer to what a **toggle in the spare bit** does: flip it on each send, and if the host ever sees the same bit twice it is an error (Otavio) | Right. A toggle carries liveness, which neither a static flag nor a firmware-side freshness check can (the latter needs a clock the firmware cannot trust across a wrap). Two refinements: flip on replies that *saw a newer sample* rather than on every send, so the bit reports data freshness and not merely frame delivery; and let the host require three identical bits, not two, because the toggle is per board (a second poller or a lost frame makes two equal). Dropped with the watchdog: automatic re-init of a chip that silently reset — a case we are not designing for | §4.2 bit 47 defined; §4.3 sentinel on the low 47 bits; §5.2 watchdog removed; §5.4 step 4 sets the toggle; §5.7 argument re-based on latch + toggle; §6 host rule; §7 tests 7 and 12; script `encode_quat48(q, toggle)` / decoder masks bit 47 |

### v7 → v8 (2026-09-21)

| # | Point on v7 | Assessment | Change in v8 |
|---|---|---|---|
| 3 | The toggle and the sentinel counter were updated in `StartFrame()`, i.e. on every incoming frame; motor commands and configuration requests would consume those events, which is ordinary traffic, not a failure case | Correct — a real defect in ordinary operation: one config frame between two polls would make the driver see a repeated bit, and a sentinel hold could be spent on frames the host never decodes | §4.2, §5.3, §5.4: bookkeeping moves to the first `Read()` of 0x06d–0x06f in a frame, i.e. only when a quaternion reply is produced |
| – | Proceed with the scope: one Orin poller, continuously streaming IMU, 16-bit TIM3, the toggle as a practical progress check; correct the doc's absolute guarantees instead of expanding the implementation; keep the host outage reset | Agreed | §1.5 states the assumptions and the not-covered list once; §5.3, §5.7, §6 reference it instead of claiming guarantees; the outage reset is named as the backstop |

---

## 11. Implementation notes (firmware, 2026-09-22)

What was built against this design, and where it deviates.  Files:

| file | role |
|---|---|
| `fw/quat48.h` | float32 `EncodeQuat48` / `DecodeQuat48`, sentinel and toggle helpers (§4) |
| `fw/imu_fusion.h` | hardware-free core: `FusionMailbox` (256 × 12 B, release/acquire, shedding), `FusionControl` (sticky ISR→main counters), `ImuFusion` (tracker, filter, 64-entry history, `BeginFrame`/`Reply` bookkeeping), `FusionStorage` |
| `fw/imu_fusion_storage.{h,cc}` | the one static block, `ClaimFusionStorage`, the timer ownership registry, `StartFusionTimer` (TIM3, PSC 679) |
| `fw/lsm6dsv16x_fusion.h` | ISR driver: non-blocking init (`FUNC_CFG_ACCESS=0`, `SW_RESET`, poll, 15 config writes, `WHO_AM_I`, `FREQ_FINE`), then the status/word loop |
| `fw/aux_port.h` | device type `lsm6dsv16xFusion` (= 5): claim storage + TIM3 in `HandleConfigUpdate`, ISR start/complete plumbing, main-loop drain, `aux2 fusion …` console commands |
| `fw/fdcan.{h,cc}`, `fw/fdcan_micro_server.h`, `fw/multi_transport_datagram_server.h` | external timestamp counter selected before `Start`; per-frame `RxTimestamp` and RX FIFO fill level exposed to the controller; transport of the current frame recorded |
| `fw/moteus_controller.cc` | `StartFrame` resets the quaternion cache and runs `BeginFrame`; first read of 0x06d–0x06f produces the reply (`Reply(stamp, hw)`) |
| `fw/bldc_servo.{cc,h}`, `fw/bldc_servo_structs.h` | `servo_stats.isr_max_cycles`: longest control cycle (DWT) of the previous second |
| `fw/test/quat48_test.cc`, `fw/test/imu_fusion_test.cc` | host tests: §4.4 vectors bit-exact, 200 k round trips within 1.2× the bound; a simulated chip feeding the core through the mailbox — static init, constant rotation (≤ 0.02° at request times inside and 7 ms beyond the history), tracker learning +1500 ppm to < 100 ppm with 320 µs jitter, 10-word gap dead-reckoned, 200-word gap → re-init with heading kept, tick wrap every 262 ms, stall marking of queued frames, toggle semantics, resync/overrun from the sticky counters, shedding/overflow, bias learning |

Deviations from the text above (the design sections were left as written):

* **Tracker (§5.3).** Not a leaky floor with a 10 s time constant: per
  window (first 128 samples, then 1024) the minimum residual of each of
  8 sub-windows is kept and a straight line fitted through them; the
  value at the window end corrects the phase, the slope the rate. A
  single windowed minimum cannot see a *rising* floor (it sits at the
  window start) and its noise bias leaks into the rate — found by the
  host test. `phase_unc_us` = 4 µs × |last phase correction| (+500 µs
  before the first window).
* **Chip timestamp words** are batched and counted (`ts_words`) but not
  used: every loss the filter can act on is exact via `seq` (mailbox) or
  catastrophic (chip overrun → re-init). `TAG_CNT` travels in
  `FusionWord::flags` and is not acted on.
* **Legacy type 3 init** stays blocking (inside the ISR, as before); it
  gained `FUNC_CFG_ACCESS = 0` first and `FIFO_CTRL3 = 0` (raw batching
  off), but no `SW_RESET` + wait, which would add blocking time to the
  ISR. Switching type 5 → 3 is still clean because the fusion's
  registers are all rewritten by the legacy path or turned off here.
* **Errors:** storage or TIM3 already taken → `aux2.error = kUnsupported`
  with `fusion.init_state` 253 (storage) / 254 (timer); only one fusion
  device per board.
* **Sentinel hold** is armed by a re-initialization or a cold start,
  and *not* re-armed when the accelerometer completes the re-init
  (that would cost the host a fourth sentinel).
* **UART-transport frames** (fdcanusb tunnel) have no hardware stamp:
  the reply uses the current TIM3 tick and sets `timing_degraded`.
* **Debug/tuning console commands** (runtime only, nothing persistent):
  `aux2 fusion gains <kp> <ki>`, `aux2 fusion latency <us>`,
  `aux2 fusion halt <0|1>` (stop word reads, §7 test 12),
  `aux2 fusion drop <n>` (skip n gyro words, test 4a),
  `aux2 fusion stall <ms>` (hold the main loop, tests 9/13).
* `isr_max_cycles` is a rolling previous-second maximum rather than a
  reset-on-read value, so tview and scripts need no reset command.
* BiSS-C does not yet consult the timer registry (not used on this
  robot); PWM output on a TIM3 pin does.
* **Telemetry lives in its own record** (`aux1_fusion` / `aux2_fusion`),
  not inside `aux2`. Appending the fusion block to `AuxStatus` pushed the
  `aux2` schema past the 2 KB telemetry output buffer
  (`fw/moteus.cc:246`), and mjlib's `BufferWriteStream` asserts on
  overflow, which halts the board: the first bench flash died the moment
  `tel schema aux2` was requested (2026-09-22). Rule for future work:
  never grow an existing record; add a new one, and check `tel schema`
  of anything touched before flashing.

Host side (humanoid3, 2026-09-22, uncommitted on `main`): the register
renumbering 0x72 → 0x6d that the merge required was still pending on the
host — the driver was reading upstream's PWM-input registers — and is
included. `orin/imu_common.py` gains `encode_quat48` / `decode_quat48`
(vectorized) and `QUAT48_STALE_REPEATS`; `orin/can_backend.py` requests
`0x14+3, 0x6d`; `orin/can_utils.py` stores the raw words and latches
`imu_reply_count` / `imu_sentinel_seen` per board at parse time;
`orin/rl_api/driver.py` decodes, holds the previous quaternion on a
sentinel, applies the three-identical-toggles rule, treats two polls
without a reply as an outage, and exposes `get_imu_board_flags()` →
(stale, reinit); `orin/deploy/canmonitor_runner.py` resets the FD
baseline and low-pass of any board with `reinit`; the MuJoCo and fastphys
backends emit quat48 words with a per-board toggle that flips when sim
time advanced; `orin/canmonitor.py` decodes for its gravity columns;
`motor_scripts/check_robot_imus.py` auto-detects fusion vs legacy and
reports toggle flips and sentinels; `update_robot_firmware.py` fails on
any added config key outside `EXPECTED_ADDED` (empty) and accepts sensor
type 3 or 5. Tests: `test_imu_common.py` (+4), `test_can_quat48_parse.py`,
`test_driver_imu_freshness.py`. The workstation has neither pytest nor
python-can; these ran through a stub runner, and the torch/warp-dependent
observation-builder tests must run on the Orin.

### Bench results (board 2, 2026-09-22)

| test (§7.1) | result |
|---|---|
| 1 FIFO bring-up | init to `Running` in one pass; 938 Hz gyro words (`FREQ_FINE` −15), 121 Hz accel, timestamp word every 32 slots; 0 I2C errors, 0 overruns, 0 gaps over the session; mailbox depth 1 in steady state |
| 2 ISR timing | `isr_max_cycles` idle 2 747 (16.2 µs), zero-torque position mode 4 137–4 161 (24.3–24.5 µs) with the fusion streaming — the pre-fusion measurement was 25.5 µs worst; the byte pump adds nothing measurable. Fault code 102 shown in that mode is `kLimitMaxTorque` (max torque 0), a limit indicator, not a fault |
| 3 persistence | `conf write` round trip 97 ms, no mailbox drops. But each page erase timed out the I2C transaction in flight (2 errors per save): while a page erases the ISR cannot fetch the (flash-resident) I2C pump code, and `isr_max_cycles` shows 86 µs stalls from double-word programming. The first driver treated any error as a chip resync (2 re-inits per save); fixed to retry isolated errors while running and resync only after three in a row — verified: one retried error per save, 0 resyncs, 0 re-inits, 0 gaps, mailbox peak 72. The fusion also starts by itself at boot from the persisted `type 5` |
| 4a drop injection | `aux2 fusion drop 10` → `gyro_gaps` +1, `gap_slots` +10, no re-init, 0.005° change at rest |
| 9 / 13 stall + queued requests | 150 ms stall with three requests queued in the RX FIFO: round trip 154 ms, one stall pass, three frames marked unknown, three sentinels; 50 ms stall: no stall pass, all three answered validly at their SOF stamps |
| 10 wrap | 10 s at 120 Hz (≈ 38 counter wraps): 0 sentinels, 0 toggle repeats, 0.012° max step at rest |
| 12 halt | 0.6 s halt: 51 sentinels and 16 valid-looking replies around the 262 ms aliasing points, every one with a repeated toggle (exactly the predicted residual); on resume one re-init, ~1 s of sentinel, no heading jump |
| 120 Hz polling | 553 replies: the three initial sentinels, then all valid, toggle alternating on every reply, 0.010° steps at rest; `phase_unc_us` 12–28 |
| drift at rest, steady state (2026-09-22, after the stationary learner and startup capture) | Two 5-minute windows after a 150 s warm-up, game vector batched alongside: yaw drift fusion −0.0001 and −0.0000°/s vs SFLP −0.0006 and −0.0011°/s; tilt jitter fusion 0.005–0.006° rms vs SFLP 0.024–0.028°; learned bias (−0.511, 0.167, −0.021) dps vs SFLP gbias (−0.512, 0.166, −0.018), i.e. within the gbias resolution of 4.4 mdps. The earlier "0.0056 vs 0.0005°/s" figure came from a single 90 s window that still contained the learner's convergence from boot. Mean tilt offset fusion vs SFLP 0.39° in the flat pose = the accelerometer bias correction (predicted 0.36° from the six-position bias), which the SFLP does not have. 0 I2C errors, resyncs or re-inits during the 12 minutes; the 21 876 errors on the counters had accumulated while the board was handled for the six-position poses (test-point wires) |

Bench-tool notes: the Python `Stream` defers diagnostic writes until the
channel is read, so a console command that must take effect at a known
instant (the stall hook) goes out as `[diagnostic write, diagnostic read,
queries…]` in one transport `cycle`; and a script must `flush_read()`
*before* its first command, or stale binary telemetry from the previous
session stalls `read_until_OK` (seen as intermittent 5 s timeouts).

| axis conventions (§7.3) | **Match.** With the chip's SFLP game vector batched alongside (`aux2 fusion sflp 1`, tag 0x13 at 120 Hz) and the board hand-rotated about all three axes: body-frame up vectors within 0.3° of the game vector at every tilt (accelerometer axes), heading difference constant within ±0.5° through a ±40° rotation about the vertical (gyro signs, handedness). Raw gyro/accel axes map 1:1 onto the SFLP frame; no remapping needed. Repeated with the test-point wires kept clear: 119 samples (68 moving), 0.46° max / 0.11° median up-vector disagreement, heading offset constant within 1.0°, and 0 I2C errors, resyncs, re-inits, gaps or overflows for the whole run |
| link dropouts (unplanned) | Loose test-point wires on the bench board shorted the I2C lines while it was handled (confirmed: a repeat run with the wires clear had zero errors): 3 793 I2C errors and 3 757 resyncs in one session, in 8 episodes. Every episode recovered by itself (re-init with heading carried over; mailbox never overflowed). Cost per episode: the rotation during the dropout is lost from the heading (one ~0.3 s episode shifted it 7.7°) plus ~1 s of sentinel. A "soft resync" — verify the chip's configuration registers and drain its 512-word FIFO backlog instead of `SW_RESET` — would make dropouts shorter than ~0.4 s lossless; proposed, not implemented |

**Early convergence (2026-09-22).** The fixed 1 s convergence count was
the whole cold-start time (bench board 2, `d reset` to first valid reply:
1 076-1 080 ms over three resets; CAN answers after 41 ms). It is not a
thermal warm-up: the first accelerometer word already sets the tilt, and
the time model locks within ~130 ms. Replies now become valid after
`converge_words_min` (192 words, 0.2 s) once the last `converge_calm_words`
(12, 0.1 s) accelerometer words each agreed with the estimated tilt to
`converge_innovation_max` (0.2°) at full correction weight; a board that
starts moving (down-weighted correction) still waits the 1 s
`converge_words`. Host tests: still start valid in < 0.35 s within 0.05°;
0.1 g of linear acceleration in the first accel word (5.7° initial error)
holds the sentinel until the tilt is within 0.25°; 1 rad/s of rotation
falls back to 1 s. Bench, five resets: first valid reply 263-395 ms, eight
alternating toggles 40 ms later at 200 Hz polling; the tilt at the first
valid reply was within 0.035-0.052° of the tilt 4.6 s later (three
resets). Tool: `utils/imu_fusion_bench/cold_start.py`.

Still open on the bench: test 5 against a fixture.

Corrections and additions, 2026-09-22 evening:

* **Test metric bug.** The host tests' quaternion angle helper had been
  rewritten to the numerically stable `2·atan2(|a−b|, |a+b|)`, which for
  unit quaternions is *half* the rotation angle (the chord is
  `2·sin(θ/4)`; the factor must be 4). Every threshold was re-checked
  with the corrected helper and all tests still pass, so the accuracy
  figures quoted above hold at true scale; the Python reference script's
  acos-based metric was always correct. Bench comparisons used vector
  angles and are unaffected.
* **Six-position accelerometer test (step 2 of the calibration plan):**
  telemetry `accel_raw_g` (1 s low-pass of the uncorrected accelerometer),
  runtime `aux2 fusion acal bx by bz [sx sy sz]` (g, unitless; not
  persistent) applied as `a = (raw − b)/s` before fusion, and
  `utils/imu_fusion_bench/six_point.py`, which fits an axis-aligned
  ellipsoid (`A x² + B y² + C z² + D x + E y + F z = 1`) to eight still
  poses by default (six axis-ish plus two tilted ones for redundancy; six
  is the bare minimum and leaves no residual), so no pose needs to be aligned with anything and the
  operator need not know which sensor axis is which: the prompts are
  physical (flat, flipped, on each edge), the script reports which sensor
  direction was up in each pose, tracks coverage of the six directions,
  and tells the operator what to do with the board if one is missing.
  Quality checks: fit residual, leave-one-out bias stability, and a
  motion warning from the spread of the averaged samples. Host test: a
  12 mg x offset gives 0.69° of tilt uncorrected and < 0.05° corrected;
  the script's self-test recovers a synthetic bias to 0.59 mg worst case
  with poses up to 15° off-axis.
  **Bench result (board 2, 2026-09-22, eight poses):** bias
  (+2.2, +8.7, +5.8) mg, scale (0.9974, 0.9973, 0.9979); fit residual
  0.23 mg rms, leave-one-out bias stability 0.13 mg, every pose within
  7.2° of an axis. The bias alone would tilt the estimate by up to 0.61°
  (0.36° in the flat pose), i.e. this single chip's zero-g offset is the
  largest static orientation error in the system by an order of
  magnitude, which is the case for persisting the correction. The
  uniform 0.26 % scale deficit is harmless (a common scale cancels in the
  direction; local g is itself ~0.1 % below standard) and the per-axis
  spread of 0.06 % is worth < 0.03° of tilt. Sensor-axis map of this
  board: +Y = normal of the face laid on the table, ±X along the long
  edges, ±Z along the short edges. Applied at runtime only:
  `aux2 fusion acal 0.00223 0.00865 0.00584 0.99741 0.99733 0.99786`
  (that console command has since been replaced by the config group below).
* **Persistent calibration: the `imu_cal` config group** (`fw/imu_cal.h`).
  A new top-level group, registered by `MoteusController` and handed to
  both aux ports, so it belongs to the board rather than to a port and the
  aux1/aux2 schemas (and their CRCs) are untouched. Keys:
  `imu_cal.accel_bias.0..2` (g, default 0) and `imu_cal.accel_scale.0..2`
  (default 1; a value outside 0.5–2 is treated as 1). The port that owns
  the fusion copies the group into the fusion parameters when the fusion
  attaches and again on every load or `conf set` (mjlib fires the group's
  callback in both cases), so a change takes effect immediately and
  `conf write` persists it. The console `aux2 fusion acal` command is
  gone; `six_point.py --apply` now issues the six `conf set`s and
  `--persist` adds the `conf write`. Robot rollout: the updater's
  expected-added list names these six keys with their defaults; any other
  added key still fails the post-flash compare. Adding fields to this
  group later changes only its own CRC, and `moteus_tool --flash` replays
  the old keys by name, so motor calibration is never involved.
  Cost: about 1.8 KB of flash for the group's serializers; the image now
  ends 2 240 bytes below the config page (checked from the ELF *load*
  addresses with `readelf -l`, since `.data` and `.ccmram` live in RAM but
  are stored in flash after `.text`; a section-address check misses them).
  The linker cap remains the real guard.
  Bench verification (board 2, 2026-09-22): flashed over CAN in 2.5 min;
  post-flash compare 0 changed / 0 removed / 6 added (`imu_cal`, at
  defaults), all 327 calibration values identical; a 20 mg `conf set`
  test bias tilted the estimate 1.21° and clearing it returned to 0.045°;
  the six-position values were stored with `conf set` + `conf write`,
  survived a hard reset (`conf enumerate` shows them), and the fusion came
  up using them: 0.38° tilt offset against the uncorrected SFLP, the
  predicted effect of the stored bias. Note for scripts: `conf get`
  answers with the value line only, no `OK`, so it must be read with a
  plain `readline`, not `Stream.command()` (which waits for `OK` and
  times out).
* **Flashing lesson.** One bench flash was killed by a 600 s shell
  timeout while still *verifying* (it ran far slower than the usual 3 min,
  consistent with CAN retries on the bench's loose wiring); the board was
  left without a `lock`/`reset` and unresponsive until power-cycled.
  Never wrap `moteus_tool --flash` in a short timeout, and watch the
  `flash:` progress lines rather than filtering them out.
* **Flash layout, the real cause of the failed flash.** The persistent
  config occupies the last two flash pages (0x0807f000–0x08080000), but
  the linker's application region was `MBED_APP_SIZE = 0x70000` from
  0x08010000 — it *includes* those pages. The fusion firmware grew the
  image (text plus the flash copies of `.data` and `.ccmram`) past
  0x0807f000 by 136 bytes; flashing it erased the config page, so the
  board booted with defaults (id 1, calibration gone), and its next
  `conf write` would have erased the image's own tail. mjlib's per-field
  telemetry serializers (schema, binary, text) cost ~300 bytes per field,
  which is where the growth came from. Fixes: `MBED_APP_SIZE` capped at
  0x6f000 so an oversized image fails to link, and the fusion record
  trimmed (six SFLP scalars packed into two arrays; `ts_words`,
  `sflp_words`, `mailbox_shed`, `pending_unknown`, `last_floor_ticks`,
  `history_count` dropped). Recovery on the bench: flash a fitting image,
  then `moteus_tool --restore-config` with the config the tool had
  captured. Check `readelf -S` section ends against 0x0807f000 before
  every flash. After trimming, the image ended only 792 bytes below the config
  page; the main-loop fusion code is now compiled `-Os` on the target
  (`FUSION_MAINLOOP`, speed is irrelevant there) and the ISR driver kept
  out of CCM RAM (its I2C peer already runs from flash): 4 024 bytes of
  flash margin and 1 488 bytes of CCM free.
* **Recovery outcome and the restore pitfall.** The fitting image was
  flashed at id 1 with `--no-restore-config`, then the captured config was
  replayed. A `--restore-config` issued at the *old* id hangs part-way:
  a dump lists `id.id` on its second-to-last line, so the replay switches
  the board to its real id mid-stream and every later command times out
  (the tool waits forever with no output and the adapter goes quiet).
  Restore at the id the board answers on *now* (`-t 2` here, where the
  `id.id 2` line is then a no-op). Verified afterwards with a hard reset
  (`d reset` on the diagnostic stream = `NVIC_SystemReset`): the board
  came back at id 2 with all 327 calibration values identical to the
  capture, the fusion driver running, and zero I2C errors or resyncs. Note
  that `conf load` alone proves nothing (mjlib's loader silently skips a
  blank or mismatched flash page and leaves RAM as is); only a reset does.
  Also observed: writing any `aux2.*` key, even to its current value,
  re-initializes the port, so the fusion storage is reclaimed and every
  counter restarts from zero (not counted as a resync); on the robot this
  only happens during configuration, never while polling.

### Review fixes (2026-09-22)

* Failed FIFO status reads are still retried, but failed FIFO word reads
  now trigger resync and re-initialization: the chip may have consumed a
  sample that never received a producer sequence number. Neither the
  arrival tracker nor the unused TAG_CNT field recovers that rotation.
  Consequently, a config save that interrupts a word read can cause a
  convergence interval of sentinel replies; the earlier persistence
  measurements above describe the previous retry policy.
* The ISR maximum is collected with an atomic exchange in the main loop,
  so publishing and clearing it cannot erase a peak from an intervening
  control interrupt. The ISR uses only relaxed loads and stores.
* Non-finite accelerometer calibration biases use zero; invalid scales
  continue to use one. Persistent config keys and their schema are unchanged.
* Fusion requires an I2C clock of at least 400 kHz. Slower configurations
  fail with `aux.error = kUnsupported` before claiming fusion resources.
