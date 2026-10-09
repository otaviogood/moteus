// Copyright 2026 Otavio Good.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>

#include "fw/aux_common.h"
#include "fw/quat48.h"

/// On-board orientation fusion for the LSM6DSV16X.  Design:
/// docs/imu_orientation_redesign.md.
///
/// Hardware-free: the ISR-side driver (fw/lsm6dsv16x_fusion.h) pushes
/// raw FIFO words into FusionMailbox and bumps the sticky counters in
/// FusionControl; ImuFusion (main loop) drains them, runs the filter,
/// keeps a short orientation history and answers register reads.
///
/// Time base: TIM3 ticks of 4 us, 16 bits, wrapping every 262 ms.  Every
/// comparison is a signed 16-bit difference (§5.7).

namespace moteus {

// The main-loop fusion code has no speed requirement (a few microseconds
// per sample either way) but flash is tight: the application region ends
// at the persistent-config page.  Size-optimize it on the target; the host
// toolchain (clang) does not know the attribute.
#if defined(TARGET_STM32G4)
#define FUSION_MAINLOOP __attribute__((noinline, optimize("Os")))
#else
#define FUSION_MAINLOOP
#endif

constexpr float kFusionTickUs = 4.0f;
constexpr float kFusionGyroOdrHz = 960.0f;
constexpr float kFusionAccelOdrHz = 120.0f;
// +-2000 dps: 70 mdps/LSB -> rad/s.
constexpr float kFusionGyroScale = 70.0e-3f * 3.14159265f / 180.0f;
// +-4 g: 4 / 32768 g/LSB.
constexpr float kFusionAccelScale = 4.0f / 32768.0f;
constexpr float kFusionNominalTicks =
    1.0e6f / kFusionGyroOdrHz / kFusionTickUs;  // 260.4167
// The chip's timestamp counter (TIMESTAMP0-3, FIFO tag 0x04) and its ODR
// run from one oscillator: 46080 / 960 = 48 counter LSB per gyro sample,
// whatever INTERNAL_FREQ_FINE says (DS Table 145).
constexpr int32_t kFusionChipTicksPerSample = 48;

struct FusionWord {
  uint16_t t = 0;       // TIM3 ticks at I2C completion
  uint16_t seq = 0;     // gyro sequence number (assigned before drop decision)
  int16_t v[3] = {};    // raw sensor words
  uint8_t tag = 0;      // FusionTag
  uint8_t flags = 0;    // TAG_CNT: the chip's 2-bit time-slot counter
};
static_assert(sizeof(FusionWord) == 12);

enum FusionTag : uint8_t {
  kFusionTagGyro = 0x01,
  kFusionTagAccel = 0x02,
  kFusionTagTimestamp = 0x04,
};

/// Single producer (ISR) / single consumer (main loop) ring.
class FusionMailbox {
 public:
  static constexpr uint16_t kSize = 256;
  static constexpr uint16_t kShedThreshold = 192;

  // ISR side.  Accel and timestamp words are shed above
  // kShedThreshold; gyro words only when completely full.  Returns
  // false if the word was dropped.
  bool Push(const FusionWord& word) {
    const uint16_t head = head_.load(std::memory_order_relaxed);
    const uint16_t tail = tail_.load(std::memory_order_acquire);
    const uint16_t fill = static_cast<uint16_t>(head - tail);
    if (fill >= kSize) {
      overflow_++;
      return false;
    }
    if (fill >= kShedThreshold && word.tag != kFusionTagGyro) {
      shed_++;
      return false;
    }
    buf_[head % kSize] = word;
    head_.store(static_cast<uint16_t>(head + 1), std::memory_order_release);
    if (fill + 1 > max_depth_) { max_depth_ = fill + 1; }
    return true;
  }

  // Main loop side.
  bool Pop(FusionWord* out) {
    const uint16_t tail = tail_.load(std::memory_order_relaxed);
    const uint16_t head = head_.load(std::memory_order_acquire);
    if (head == tail) { return false; }
    *out = buf_[tail % kSize];
    tail_.store(static_cast<uint16_t>(tail + 1), std::memory_order_release);
    return true;
  }

  uint16_t size() const {
    return static_cast<uint16_t>(head_.load(std::memory_order_acquire) -
                                 tail_.load(std::memory_order_relaxed));
  }

  void Reset() {
    head_.store(0);
    tail_.store(0);
    overflow_ = 0;
    shed_ = 0;
    max_depth_ = 0;
  }

  uint32_t overflow() const { return overflow_; }
  uint32_t shed() const { return shed_; }
  uint16_t max_depth() const { return max_depth_; }

 private:
  std::array<FusionWord, kSize> buf_ = {};
  std::atomic<uint16_t> head_{0};
  std::atomic<uint16_t> tail_{0};
  uint32_t overflow_ = 0;
  uint32_t shed_ = 0;
  uint16_t max_depth_ = 0;
};

/// Sticky ISR -> main loop signals that must not be lost even when the
/// ring is full.  The consumer keeps its own copy and acts on changes.
struct FusionControl {
  std::atomic<uint16_t> resync_count{0};   // chip re-init started
  std::atomic<uint16_t> running_count{0};  // init finished (freq_fine valid)
  std::atomic<uint16_t> overrun_count{0};  // FIFO_OVR_LATCHED seen
  std::atomic<int16_t> freq_fine{0};       // INTERNAL_FREQ_FINE, signed
  std::atomic<uint8_t> init_state{0};      // driver state, for telemetry
  // Transfer-delay probe: the chip's live timestamp counter
  // (TIMESTAMP0-3) read right after a FIFO timestamp word, and the TIM3
  // ticks when that read completed.  probe_count changes last.
  std::atomic<uint32_t> probe_chip{0};
  std::atomic<uint16_t> probe_t{0};
  std::atomic<uint16_t> probe_count{0};

  void Reset() {
    resync_count.store(0);
    running_count.store(0);
    overrun_count.store(0);
    freq_fine.store(0);
    init_state.store(0);
    probe_chip.store(0);
    probe_t.store(0);
    probe_count.store(0);
  }
};

struct FusionHistoryEntry {
  uint16_t t = 0;      // model sample time, ticks
  float q[4] = {};
};

namespace fusion_math {

inline void QuatMul(const float* a, const float* b, float* out) {
  out[0] = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
  out[1] = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
  out[2] = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
  out[3] = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
}

inline void Normalize(float* q) {
  const float n2 = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
  if (n2 <= 0.0f) {
    q[0] = 1.0f; q[1] = q[2] = q[3] = 0.0f;
    return;
  }
  const float inv = 1.0f / std::sqrt(n2);
  for (int i = 0; i < 4; i++) { q[i] *= inv; }
}

/// q <- q (x) exp(theta / 2), theta a body-frame rotation vector.
/// Exact for the angle, so it is also used for reply extrapolation.
inline void Rotate(float* q, const float* theta) {
  const float a2 = theta[0] * theta[0] + theta[1] * theta[1] +
      theta[2] * theta[2];
  float d[4];
  if (a2 < 1.0e-8f) {
    d[0] = 1.0f - a2 * 0.125f;
    const float s = 0.5f * (1.0f - a2 / 24.0f);
    d[1] = theta[0] * s;
    d[2] = theta[1] * s;
    d[3] = theta[2] * s;
  } else {
    const float a = std::sqrt(a2);
    const float half = 0.5f * a;
    const float s = std::sin(half) / a;
    d[0] = std::cos(half);
    d[1] = theta[0] * s;
    d[2] = theta[1] * s;
    d[3] = theta[2] * s;
  }
  float out[4];
  QuatMul(q, d, out);
  std::memcpy(q, out, sizeof(out));
  Normalize(q);
}

/// Predicted world-up expressed in the body frame: R(q)^T (0, 0, 1).
inline void BodyUp(const float* q, float* g) {
  const float w = q[0], x = q[1], y = q[2], z = q[3];
  g[0] = 2.0f * (x * z - w * y);
  g[1] = 2.0f * (w * x + y * z);
  g[2] = 1.0f - 2.0f * (x * x + y * y);
}

/// Heading of the body x axis in the world xy plane.
inline float Yaw(const float* q) {
  const float w = q[0], x = q[1], y = q[2], z = q[3];
  const float r00 = 1.0f - 2.0f * (y * y + z * z);
  const float r10 = 2.0f * (x * y + w * z);
  return std::atan2(r10, r00);
}

inline void Nlerp(const float* a, const float* b, float alpha, float* out) {
  float dot = 0.0f;
  for (int i = 0; i < 4; i++) { dot += a[i] * b[i]; }
  const float sign = dot < 0.0f ? -1.0f : 1.0f;
  for (int i = 0; i < 4; i++) {
    out[i] = (1.0f - alpha) * a[i] + alpha * sign * b[i];
  }
  Normalize(out);
}

}  // namespace fusion_math

class ImuFusion {
 public:
  struct Params {
    float kp = 0.3f;                 // accel correction gain, 1/s
    float ki = 0.02f;                // gyro bias learning gain
    float kp_init = 10.0f;           // gain during the first init_words
    uint16_t init_words = 1920;      // 2 s at 960 Hz
    uint16_t converge_words = 960;   // 1 s: replies valid regardless
    // Early convergence (a board that starts still): replies become valid
    // after converge_words_min once the last converge_calm_words accel
    // words each agreed with the estimated tilt to converge_innovation_max
    // at full correction weight.  A board that starts moving waits for
    // converge_words.
    uint16_t converge_words_min = 192;        // 0.2 s
    float converge_innovation_max = 0.00349f; // sin 0.2 deg
    uint8_t converge_calm_words = 12;         // accel words (0.1 s)
    float bias_max = 0.05f;          // rad/s
    float innovation_max = 0.0872f;  // sin 5 deg
    float acc_tol_full = 0.05f;      // | |a|/g - 1 | for full weight
    float acc_tol_zero = 0.20f;      // ... for zero weight
    float omega_full = 0.5f;         // |omega| rad/s for full weight
    float omega_zero = 3.0f;         // ... for zero weight
    uint8_t quasi_static_words = 12; // accel words (100 ms) before bias learning
    uint16_t stall_entries = 100;    // mailbox depth at drain => stall pass
    uint16_t extrapolate_max_ticks = 2000;  // 8 ms
    uint16_t gap_max_words = 96;     // dead-reckon up to this, else re-init
    uint8_t sentinel_hold_replies = 3;
    // Orientation compensation: the reply is evaluated this much after
    // the request stamp (gyro extrapolation), cancelling the chip's
    // filter delay, the transfer and the fusion time base.  MEASURED
    // 2026-10-07 on the robot (hand-rocked joint vs its encoder,
    // orin/calib/STATUS.md): with CTRL6 at the 310 Hz setting the
    // orientation is ~2.15 ms behind the request (3.35 at 149 Hz); 2.0 ms
    // is taken out here, the ~0.15 ms left is modelled by the simulators
    // (kept positive: they interpolate history, not extrapolate).  A
    // prediction with the newest, itself ~2 ms old, gyro rate: exact for
    // smooth motion, a gain error ~L*D*w^2 at high frequency.  Console
    // `aux2 fusion latency <us>` overrides it (not saved).
    uint16_t latency_comp_ticks = 500;  // 2000 us
    uint16_t tracker_window_first = 128;
    uint16_t tracker_window = 1024;
    // No gyro word for this long (main-loop milliseconds): replies are
    // sentinels.  The 16-bit tick stamps wrap every 262 ms, so the
    // tick-based extrapolation limit alone would accept a stale history
    // again once a stopped acquisition has lasted a whole wrap.
    uint16_t stale_ms = 50;
    // Stationary gyro-bias learning, all three axes (the accelerometer
    // innovation cannot see the bias about the vertical).  While every
    // axis reads within stat_omega_max of the current bias, |a| is within
    // stat_acc_tol of g and the gravity direction has not moved by more
    // than stat_dir_deg since the window began, for stat_qualify_words
    // samples, the residual rate is the bias error and is blended in
    // with per-sample gain stat_gain (time constant ~10 s at 960 Hz).
    // A 6-axis IMU cannot tell a steady yaw from a bias, so a corrected
    // rate above stat_omega_max counts as motion -- which also means a
    // bias error above it can never be unlearned here: the learner tracks
    // any rotation slower than its time constant, and when that rotation
    // stops the corrected rate jumps by the absorbed amount (8 of 48
    // robot boards sat at a false 0.6-1.2 dps, 2026-10-07).  The way out
    // is Relearn(), requested by the host when the robot is known to be
    // still (console `aux2 fusion relearn`): for its wall-time deadline
    // the gate is steadiness only (each axis within stat_omega_max of its
    // own stat_lp_alpha average), at the fast gain.
    float stat_omega_max = 0.0174533f;   // 1 dps
    float stat_lp_alpha = 1.0f / 240.0f; // 0.25 s at 960 Hz
    uint8_t stat_over_samples = 8;       // consecutive over-threshold samples that reset the window
    float stat_acc_tol = 0.02f;
    float stat_dir_cos = 0.99999391f;    // cos(0.2 deg)
    uint16_t stat_qualify_words = 960;   // 1 s
    float stat_gain = 1.0e-4f;
    // The fast gain (~0.1 s) runs only during a host-requested Relearn():
    // a cold-start fast phase used to take in whatever the board saw in
    // its first two seconds, and the robot is often moving then (the
    // joint power switch is on the robot; a hanging robot twists after
    // it is touched), which is how 7 of 48 boards came to carry a false
    // 0.6-1.2 dps for hours (2026-10-07).  At boot the bias starts from
    // the persisted gyro_bias0 (imu_cal.gyro_bias, saved by the host) and
    // the slow learner refines it; the runner's startup relearn takes
    // care of the rest before the policy runs.
    float stat_gain_fast = 1.0e-2f;
    // Accelerometer calibration, applied to the raw reading before use:
    // a = (raw - accel_bias) / accel_scale, in g.  Copied from the
    // persistent `imu_cal` config group by the aux port (fw/imu_cal.h).
    float accel_bias[3] = {0.0f, 0.0f, 0.0f};
    float accel_scale[3] = {1.0f, 1.0f, 1.0f};
    // The gyro bias to start from at power-up (imu_cal.gyro_bias, rad/s),
    // seeded once at the first initialization.
    float gyro_bias0[3] = {0.0f, 0.0f, 0.0f};
  };

  enum ReinitReason : uint8_t {
    kReinitNone = 0,
    kReinitResync = 1,
    kReinitOverrun = 2,
    kReinitGap = 3,
  };

  static constexpr size_t kHistorySize = 64;
  using History = std::array<FusionHistoryEntry, kHistorySize>;

  // Transfer-delay probe, bench builds only (flash is tight):
  //   tools/bazel build --config=target //:target --copt=-DMOTEUS_TS_PROBE
  // The probe's 4-byte read of TIMESTAMP0-3 at 400 kHz: the chip's value
  // is taken as latched at the first data byte, 4 x 9 bits = 90 us = 22
  // ticks before the transfer ends.  (The ISR sees both this completion
  // and a word's arrival up to one control cycle late, which cancels.)
  // Systematic uncertainty of the delay below: about +-50 us.
  static constexpr uint16_t kProbeLatchTicks = 22;
  // The measured gyro word is this many slots after the timestamp slot:
  // the slot right after it is itself delayed by the probe's own read
  // (bench 2026-10-06: its transfer floor read 519 us while the fusion
  // clock sat 398 us behind the chip), two slots on the bus is quiet again.
  static constexpr uint16_t kProbeSlotOffset = 2;

  ImuFusion() {}

  void Attach(FusionMailbox* mailbox, FusionControl* control,
              History* history, aux::ImuFusionStatus* status) {
    mailbox_ = mailbox;
    control_ = control;
    history_ = history;
    status_ = status;
    Reset();
  }

  Params* params() { return &params_; }

  /// Host-requested gyro-bias relearn (console `aux2 fusion relearn <s>`):
  /// the robot is known to be still, so a steady corrected rate of any
  /// size is bias error, taken in at the fast gain for learn_ms of
  /// elapsed time after the stationary window has qualified again (one
  /// second); the accelerometer gate is not consulted.  The assurance of
  /// stillness does not outlive the request: the deadline is wall time
  /// (PollMillisecond), and an acquisition outage (stale words) or a
  /// re-initialization cancels it.
  void Relearn(uint16_t learn_ms) {
    const uint32_t qualify_ms = static_cast<uint32_t>(
        params_.stat_qualify_words * 1000.0f / kFusionGyroOdrHz + 0.5f);
    relearn_ms_ = static_cast<uint16_t>(std::min<uint32_t>(65535, learn_ms + qualify_ms));
    stat_words_ = 0;
    stat_lp_reset_ = true;
  }

  /// Main loop, once per millisecond: ages the newest gyro word (see
  /// Params::stale_ms) and runs the relearn deadline.
  void PollMillisecond() {
    if (ms_since_gyro_ < 65535) { ms_since_gyro_++; }
    if (relearn_ms_ > 0) {
      relearn_ms_--;
      if (ms_since_gyro_ > params_.stale_ms) { relearn_ms_ = 0; }
    }
  }

  FUSION_MAINLOOP void Reset() {
    initialized_ = false;
    converged_ = false;
    q_[0] = 1.0f; q_[1] = q_[2] = q_[3] = 0.0f;
    for (auto& v : bias_) { v = 0.0f; }
    bias_seeded_ = false;
    for (auto& v : omega_c_) { v = 0.0f; }
    for (auto& v : omega_prev_) { v = 0.0f; }
    for (auto& v : corr_) { v = 0.0f; }
    have_prev_omega_ = false;
    quasi_static_count_ = 0;
    converge_calm_count_ = 0;
    words_since_init_ = 0;
    sentinel_hold_ = 0;
    keep_heading_ = false;
    heading_old_ = 0.0f;
    have_seq_ = false;
    last_seq_ = 0;
    model_t_ = 0.0f;
    T_ticks_ = kFusionNominalTicks;
    dT_ = 0.0f;
    ResetTrackerWindow(true);
    windows_done_ = 0;
    last_floor_ = 0;
    hist_head_ = 0;
    hist_count_ = 0;
    newest_seq_ = 0;
    have_last_reply_seq_ = false;
    last_reply_seq_ = 0;
    toggle_ = false;
    pending_unknown_ = 0;
    frame_unknown_ = false;
    stall_latched_ = false;
    seen_resync_ = control_ ? control_->resync_count.load() : 0;
    seen_running_ = control_ ? control_->running_count.load() : 0;
    seen_overrun_ = control_ ? control_->overrun_count.load() : 0;
    counters_ = {};
    stat_words_ = 0;
    stat_over_ = 0;
    relearn_ms_ = 0;
    for (auto& v : omega_lp_) { v = 0.0f; }
    stat_lp_reset_ = false;
    stat_active_ = false;
    stat_accel_ok_ = false;
    stat_have_ref_ = false;
    stationary_words_ = 0;
    for (auto& v : accel_raw_lp_) { v = 0.0f; }
    last_request_age_ticks_ = 0;
    ms_since_gyro_ = 0;
#ifdef MOTEUS_TS_PROBE
    have_last_gyro_ = false;
    ts_wait_gyro_ = false;
    next_armed_ = false;
    pair_valid_ = false;
    probe_valid_ = false;
    seen_probe_ = control_ ? control_->probe_count.load() : 0;
    ts_pairs_ = 0;
    ts_rejects_ = 0;
    ts_delay_min_us_ = 0;
    ts_delay_mean_us_ = 0.0f;
    ts_model_mean_us_ = 0.0f;
#endif
    if (status_) { *status_ = {}; }
  }

  /// Main loop: process up to max_entries mailbox words.
  FUSION_MAINLOOP int Drain(int max_entries) {
    if (!mailbox_) { return 0; }
    const uint16_t depth = mailbox_->size();
    if (depth >= params_.stall_entries) { stall_latched_ = true; }
    CheckControl();

    int n = 0;
    FusionWord word;
    while (n < max_entries && mailbox_->Pop(&word)) {
      Process(word);
      n++;
    }
    UpdateStatus();
    return n;
  }

  /// Called once per received CAN frame before any register read.
  /// rx_fifo_fill: elements still queued in the hardware RX FIFO.
  FUSION_MAINLOOP void BeginFrame(uint32_t rx_fifo_fill) {
    Drain(FusionMailbox::kSize);
    if (stall_latched_) {
      stall_latched_ = false;
      counters_.stall_passes++;
      const uint32_t pending = 1 + rx_fifo_fill;
      pending_unknown_ = pending > 255 ? 255 : static_cast<uint8_t>(pending);
    }
    frame_unknown_ = false;
    if (pending_unknown_ > 0) {
      pending_unknown_--;
      frame_unknown_ = true;
      counters_.arrival_unknown++;
    }
  }

  /// Called when a quaternion reply is produced (first read of
  /// 0x06d-0x06f in a frame).  stamp_ticks: the frame's TIM3 SOF stamp,
  /// or the current tick count if hardware_stamp is false.
  FUSION_MAINLOOP Quat48Words Reply(uint16_t stamp_ticks, bool hardware_stamp) {
    bool valid = initialized_ && converged_ && sentinel_hold_ == 0 &&
        !frame_unknown_ && hist_count_ > 0;
    float q_out[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    if (valid) {
      const uint16_t t_req = static_cast<uint16_t>(
          stamp_ticks + params_.latency_comp_ticks);
      valid = EvaluateAt(t_req, q_out);
    }
    if (status_) { status_->timing_degraded = !hardware_stamp; }

    if (!valid) {
      counters_.sentinel_replies++;
      if (sentinel_hold_ > 0) { sentinel_hold_--; }
      if (status_) { UpdateReplyStatus(); }
      return Quat48Words{};
    }

    if (!have_last_reply_seq_ || newest_seq_ != last_reply_seq_) {
      toggle_ = !toggle_;
      last_reply_seq_ = newest_seq_;
      have_last_reply_seq_ = true;
    }
    counters_.valid_replies++;
    if (status_) { UpdateReplyStatus(); }
    return EncodeQuat48(q_out[0], q_out[1], q_out[2], q_out[3], toggle_);
  }

  /// The bias-corrected gyro rate of the newest sample, rad/s about
  /// body axis 0-2 (the quaternion's body frame), for a read of
  /// 0x065-0x067 in a frame stamped stamp_ticks.  NaN until the filter
  /// has converged, or when the newest sample is older than the
  /// quaternion's extrapolation limit.  No side effects: the freshness
  /// toggle and the sentinel hold advance on quaternion replies only.
  FUSION_MAINLOOP float RateAt(uint16_t stamp_ticks, int axis) const {
    if (!initialized_ || !converged_ || hist_count_ == 0 ||
        ms_since_gyro_ > params_.stale_ms) {
      return std::numeric_limits<float>::quiet_NaN();
    }
    const auto& newest = (*history_)[Index(hist_count_ - 1)];
    const int16_t d = static_cast<int16_t>(
        stamp_ticks + params_.latency_comp_ticks - newest.t);
    if (d > static_cast<int16_t>(params_.extrapolate_max_ticks)) {
      return std::numeric_limits<float>::quiet_NaN();
    }
    return omega_c_[axis];
  }

  // Accessors for tests and telemetry.
  bool initialized() const { return initialized_; }
  bool converged() const { return converged_; }
  bool stationary() const { return stat_active_; }
  const float* q() const { return q_; }
  const float* bias() const { return bias_; }
  const float* omega() const { return omega_c_; }
  float period_ticks() const { return T_ticks_ + dT_; }
  int32_t last_floor() const { return last_floor_; }
  uint16_t windows_done() const { return windows_done_; }
  bool toggle() const { return toggle_; }
  uint8_t pending_unknown() const { return pending_unknown_; }
  uint16_t history_count() const { return hist_count_; }
  uint16_t newest_seq() const { return newest_seq_; }

  struct Counters {
    uint32_t gyro_words = 0;
    uint32_t accel_words = 0;
    uint32_t ts_words = 0;
    uint32_t other_words = 0;
    uint16_t gyro_gaps = 0;
    uint32_t gap_slots = 0;
    uint16_t resyncs = 0;
    uint16_t overruns = 0;
    uint16_t reinits = 0;
    uint16_t stall_passes = 0;
    uint32_t arrival_unknown = 0;
    uint32_t sentinel_replies = 0;
    uint32_t valid_replies = 0;
  };
  const Counters& counters() const { return counters_; }

  /// Evaluate the orientation at t_req (ticks) from the history.
  /// Returns false when the request is outside the covered window.
  FUSION_MAINLOOP bool EvaluateAt(uint16_t t_req, float* q_out) const {
    if (hist_count_ == 0 || ms_since_gyro_ > params_.stale_ms) { return false; }
    const auto& newest = (*history_)[Index(hist_count_ - 1)];
    const int16_t d = static_cast<int16_t>(t_req - newest.t);
    if (d > static_cast<int16_t>(params_.extrapolate_max_ticks)) {
      return false;
    }
    if (d >= 0) {
      std::memcpy(q_out, newest.q, sizeof(newest.q));
      const float dt_s = static_cast<float>(d) * kFusionTickUs * 1.0e-6f;
      const float theta[3] = {omega_c_[0] * dt_s, omega_c_[1] * dt_s,
                              omega_c_[2] * dt_s};
      fusion_math::Rotate(q_out, theta);
      return true;
    }
    // Walk back to the bracketing pair.
    for (uint16_t i = hist_count_ - 1; i > 0; i--) {
      const auto& a = (*history_)[Index(i - 1)];
      const auto& b = (*history_)[Index(i)];
      const int16_t da = static_cast<int16_t>(t_req - a.t);
      if (da >= 0) {
        const int16_t span = static_cast<int16_t>(b.t - a.t);
        const float alpha = span > 0 ?
            static_cast<float>(da) / static_cast<float>(span) : 0.0f;
        fusion_math::Nlerp(a.q, b.q, alpha, q_out);
        return true;
      }
    }
    return false;  // older than the history
  }

 private:
  uint16_t Index(uint16_t logical) const {
    // logical 0 = oldest, hist_count_-1 = newest.
    return static_cast<uint16_t>(
        (hist_head_ + kHistorySize - hist_count_ + logical) % kHistorySize);
  }

  static constexpr int kSubWindows = 8;

  void ResetTrackerWindow(bool first) {
    win_count_ = 0;
    win_len_ = first ? params_.tracker_window_first : params_.tracker_window;
    if (win_len_ < kSubWindows) { win_len_ = kSubWindows; }
    sub_len_ = static_cast<uint16_t>(win_len_ / kSubWindows);
    win_len_ = static_cast<uint16_t>(sub_len_ * kSubWindows);
    sub_idx_ = 0;
    for (auto& m : sub_min_) { m = INT32_MAX; }
  }

  FUSION_MAINLOOP void FitTrackerWindow() {
    // x_j = (j + 0.5) * sub_len_ (sample offset of the sub-window
    // centre), y_j = sub_min_[j].  With x centred, slope =
    // sum((j - 3.5) y_j) / (42 sub_len_) and the value at the window
    // end is mean(y) + slope * 4 sub_len_.
    float sum_y = 0.0f;
    float sum_xy = 0.0f;
    for (int j = 0; j < kSubWindows; j++) {
      const float y = static_cast<float>(sub_min_[j]);
      sum_y += y;
      sum_xy += (static_cast<float>(j) - 3.5f) * y;
    }
    const float mean_y = sum_y / static_cast<float>(kSubWindows);
    const float slope = sum_xy / (42.0f * static_cast<float>(sub_len_));
    const float m_end = mean_y + slope * 4.0f * static_cast<float>(sub_len_);

    model_t_ += m_end;
    WrapModel();
    dT_ += slope;
    const float limit = 0.02f * T_ticks_;
    if (dT_ > limit) { dT_ = limit; }
    if (dT_ < -limit) { dT_ = -limit; }
    last_floor_ = static_cast<int32_t>(std::lround(m_end));
    windows_done_++;
    ResetTrackerWindow(false);
  }

  FUSION_MAINLOOP void CheckControl() {
    if (!control_) { return; }
    const uint16_t running = control_->running_count.load();
    if (running != seen_running_) {
      seen_running_ = running;
      const float ff = static_cast<float>(control_->freq_fine.load());
      const float odr = kFusionGyroOdrHz * (1.0f + 0.0013f * ff);
      T_ticks_ = 1.0e6f / odr / kFusionTickUs;
      dT_ = 0.0f;
    }
    const uint16_t resync = control_->resync_count.load();
    if (resync != seen_resync_) {
      counters_.resyncs += static_cast<uint16_t>(resync - seen_resync_);
      seen_resync_ = resync;
      ReInit(kReinitResync);
    }
    const uint16_t overrun = control_->overrun_count.load();
    if (overrun != seen_overrun_) {
      counters_.overruns += static_cast<uint16_t>(overrun - seen_overrun_);
      seen_overrun_ = overrun;
      ReInit(kReinitOverrun);
    }
#ifdef MOTEUS_TS_PROBE
    uint16_t probe = control_->probe_count.load();
    if (probe != seen_probe_) {
      // A consistent snapshot: the ISR stores chip and t, then bumps
      // count; re-read while count moves under us.
      uint32_t chip = 0;
      uint16_t t = 0;
      for (int i = 0; i < 3; i++) {
        chip = control_->probe_chip.load();
        t = control_->probe_t.load();
        const uint16_t again = control_->probe_count.load();
        if (again == probe) { break; }
        probe = again;
      }
      seen_probe_ = probe;
      probe_chip_ = chip;
      probe_t_ = t;
      probe_valid_ = true;
      EvaluateProbe();
    }
#endif
  }

  FUSION_MAINLOOP void Process(const FusionWord& w) {
    switch (w.tag) {
      case kFusionTagGyro: { ProcessGyro(w); break; }
      case kFusionTagAccel: { ProcessAccel(w); break; }
      case kFusionTagTimestamp: {
        counters_.ts_words++;
        ProcessTimestamp(w);
        break;
      }
      default: { counters_.other_words++; break; }
    }
  }

  FUSION_MAINLOOP void ProcessGyro(const FusionWord& w) {
    counters_.gyro_words++;
    ms_since_gyro_ = 0;

    if (!have_seq_) {
      have_seq_ = true;
      last_seq_ = w.seq;
      model_t_ = static_cast<float>(w.t);
      ResetTrackerWindow(true);
    } else {
      const uint16_t gap = static_cast<uint16_t>(w.seq - last_seq_);
      if (gap == 0) { return; }
      if (gap > 1) {
        const uint16_t lost = static_cast<uint16_t>(gap - 1);
        if (lost > params_.gap_max_words) {
          ReInit(kReinitGap);
          model_t_ = static_cast<float>(w.t);
          ResetTrackerWindow(true);
        } else {
          counters_.gyro_gaps++;
          counters_.gap_slots += lost;
          DeadReckon(lost);
          model_t_ += static_cast<float>(lost) * (T_ticks_ + dT_);
        }
      }
      last_seq_ = w.seq;
      model_t_ += T_ticks_ + dT_;
    }
    WrapModel();

    // Arrival-floor tracker (docs §5.3).  The residual of each arrival
    // against the model has a positive floor (the transfer time) plus
    // jitter.  Per window, keep the minimum residual of each of
    // kSubWindows sub-windows and fit a straight line through them: the
    // slope is the rate error, the value at the window end the phase
    // error.  (A single minimum per window cannot see a rising floor,
    // and its noise bias would leak into the rate.)
    {
      const uint16_t model_u16 = static_cast<uint16_t>(
          static_cast<uint32_t>(model_t_ + 0.5f) & 0xffff);
      const int32_t r = static_cast<int16_t>(w.t - model_u16);
      if (r < sub_min_[sub_idx_]) { sub_min_[sub_idx_] = r; }
      win_count_++;
      if (win_count_ % sub_len_ == 0 && sub_idx_ + 1 < kSubWindows) {
        sub_idx_++;
        sub_min_[sub_idx_] = INT32_MAX;
      }
      if (win_count_ >= win_len_) { FitTrackerWindow(); }
    }
    // This word's sample time on the fusion's clock (the history stamp).
    const uint16_t model_t16 = static_cast<uint16_t>(
        static_cast<uint32_t>(model_t_ + 0.5f) & 0xffff);
    ProbeGyro(w, model_t16);

    float omega[3];
    for (int i = 0; i < 3; i++) {
      omega[i] = static_cast<float>(w.v[i]) * kFusionGyroScale - bias_[i];
    }

    if (initialized_) {
      const float dt_s = (T_ticks_ + dT_) * kFusionTickUs * 1.0e-6f;
      float theta[3];
      for (int i = 0; i < 3; i++) {
        const float avg = have_prev_omega_ ?
            0.5f * (omega_prev_[i] + omega[i]) : omega[i];
        theta[i] = (avg + corr_[i]) * dt_s;
      }
      fusion_math::Rotate(q_, theta);
      words_since_init_++;
      if (words_since_init_ >= params_.converge_words ||
          (words_since_init_ >= params_.converge_words_min &&
           converge_calm_count_ >= params_.converge_calm_words)) {
        converged_ = true;
      }

      auto& e = (*history_)[hist_head_];
      e.t = model_t16;
      std::memcpy(e.q, q_, sizeof(q_));
      hist_head_ = static_cast<uint16_t>((hist_head_ + 1) % kHistorySize);
      if (hist_count_ < kHistorySize) { hist_count_++; }
      newest_seq_ = w.seq;
    }

    std::memcpy(omega_prev_, omega, sizeof(omega));
    std::memcpy(omega_c_, omega, sizeof(omega));
    have_prev_omega_ = true;

    // Stationary bias learning (all axes).  omega is already bias
    // corrected, so at rest it is the remaining bias error.
    if (initialized_) {
      if (stat_lp_reset_) {
        // A relearn's steadiness reference starts from its first sample:
        // the running average still remembers a rotation that stopped
        // within the last second or so, and until that had decayed every
        // sample would count as unsteady, eating into (or outlasting)
        // the deadline.
        stat_lp_reset_ = false;
        std::memcpy(omega_lp_, omega, sizeof(omega_lp_));
      }
      float dev = 0.0f;
      float wmax = 0.0f;
      for (int i = 0; i < 3; i++) {
        omega_lp_[i] += params_.stat_lp_alpha * (omega[i] - omega_lp_[i]);
        dev = std::max(dev, std::abs(omega[i] - omega_lp_[i]));
        wmax = std::max(wmax, std::abs(omega[i]));
      }
      // A brief bump (table vibration) neither resets the window nor
      // feeds the learner; sustained motion resets it.  Motion is a rate
      // above stat_omega_max, or, during a host-requested relearn (the
      // robot is known still), an unsteady rate of any size.
      const bool relearn = relearn_ms_ > 0;
      const bool over = relearn ? (dev > params_.stat_omega_max)
                                : (wmax > params_.stat_omega_max);
      stat_over_ = over ? static_cast<uint8_t>(std::min(255, stat_over_ + 1)) : 0;
      // The host vouching for stillness also stands in for the
      // accelerometer gate: a board whose |a| calibration is off by more
      // than stat_acc_tol never passes it (robot board can3/34 read
      // 0.975 g, 2026-10-07) and could never learn otherwise.
      if (stat_over_ >= params_.stat_over_samples || (!relearn && !stat_accel_ok_)) {
        stat_words_ = 0;
        stat_active_ = false;
      } else if (!over) {
        if (stat_words_ < 65535) { stat_words_++; }
        if (stat_words_ >= params_.stat_qualify_words) {
          stat_active_ = true;
          stationary_words_++;
          const float gain = relearn ? params_.stat_gain_fast : params_.stat_gain;
          for (int i = 0; i < 3; i++) {
            bias_[i] += gain * omega[i];
            if (bias_[i] > params_.bias_max) { bias_[i] = params_.bias_max; }
            if (bias_[i] < -params_.bias_max) { bias_[i] = -params_.bias_max; }
          }
        }
      }
    }
  }

#ifdef MOTEUS_TS_PROBE
  // Transfer-delay probe.  Every 32nd slot carries the chip's timestamp
  // counter at that slot's data-ready (FIFO tag 0x04, chip clock); the
  // driver then reads the live counter (FusionControl::probe_*), which
  // ties the chip clock to TIM3 at one instant.  The word's slot counter
  // (TAG_CNT) picks out the gyro word of the same slot whatever order the
  // chip writes them in (bench 2026-10-06: the timestamp word comes
  // first), and the measured word is the gyro word kProbeSlotOffset slots
  // later (chip time + 48 per slot, DS Table 145): a slot with nothing
  // but a gyro word, the kind the arrival-floor tracker's floor comes
  // from, far enough from the probe's own read.  Two results per
  // word: its arrival stamp minus its FIFO write (the transfer,
  // ts_delay_*) and its history stamp minus its FIFO write (the fusion
  // clock's offset from the chip's data-ready, ts_model_*).
  FUSION_MAINLOOP void ProbeGyro(const FusionWord& w, uint16_t model_t16) {
    bool armed_now = false;
    if (ts_wait_gyro_) {
      // The slot's timestamp word came first; this is its gyro word if
      // the slot counters agree.
      ts_wait_gyro_ = false;
      if (w.flags == ts_wait_cnt_) {
        ArmNext(ts_wait_value_, w.seq);
        armed_now = true;
      }
    }
    if (next_armed_ && !armed_now) {
      // The gyro word kProbeSlotOffset slots after the timestamp slot.
      const uint16_t ahead = static_cast<uint16_t>(w.seq - next_seq_);
      if (ahead >= kProbeSlotOffset) {
        next_armed_ = false;
      }
      if (ahead == kProbeSlotOffset) {
        pair_chip_ = next_chip_;
        pair_t_ = w.t;
        pair_model_t_ = model_t16;
        pair_valid_ = true;
        EvaluateProbe();
      }
    }
    last_gyro_t_ = w.t;
    last_gyro_seq_ = w.seq;
    last_gyro_cnt_ = w.flags;
    have_last_gyro_ = true;
  }

  FUSION_MAINLOOP void ProcessTimestamp(const FusionWord& w) {
    const uint32_t value =
        static_cast<uint32_t>(static_cast<uint16_t>(w.v[0])) |
        (static_cast<uint32_t>(static_cast<uint16_t>(w.v[1])) << 16);
    if (have_last_gyro_ && last_gyro_cnt_ == w.flags) {
      ArmNext(value, last_gyro_seq_);
    } else {
      ts_wait_gyro_ = true;
      ts_wait_cnt_ = w.flags;
      ts_wait_value_ = value;
    }
  }

  FUSION_MAINLOOP void ArmNext(uint32_t chip, uint16_t gyro_seq) {
    next_chip_ = chip + kProbeSlotOffset * kFusionChipTicksPerSample;
    next_seq_ = gyro_seq;
    next_armed_ = true;
  }

  FUSION_MAINLOOP void EvaluateProbe() {
    if (!pair_valid_ || !probe_valid_) { return; }
    // The live read follows the timestamp slot's burst, so its latch is
    // ~1 ms after that slot's data-ready and the measured slot (+2 x 48)
    // is written within ~2 ms of it either way.  A byte of the live
    // counter rolling over during its 4-byte read reads 256 LSB high
    // (dchip 5.6 ms too low): outside this window, rejected.
    const int32_t dchip = static_cast<int32_t>(pair_chip_ - probe_chip_);
    if (dchip > 3 * kFusionChipTicksPerSample ||
        dchip < -2 * kFusionChipTicksPerSample) {
      // Records from different cycles: only the older one is spent.  A
      // main-loop stall queues the words of one or more cycles behind
      // the latest live read, and if that read were spent on the first
      // stale pair, every later pair would meet a read one cycle ahead
      // of it, for good.
      if (dchip < 0) { pair_valid_ = false; } else { probe_valid_ = false; }
      ts_rejects_++;
      return;
    }
    pair_valid_ = false;
    probe_valid_ = false;
    const float ticks_per_lsb =
        (T_ticks_ + dT_) / static_cast<float>(kFusionChipTicksPerSample);
    // FIFO write of the measured sample, in TIM3 ticks relative to the
    // probe's latch.
    const float write_ticks = static_cast<float>(dchip) * ticks_per_lsb;
    const uint16_t latch = static_cast<uint16_t>(probe_t_ - kProbeLatchTicks);
    const int32_t us = static_cast<int32_t>(std::lround(
        (static_cast<float>(static_cast<int16_t>(pair_t_ - latch)) - write_ticks) *
        kFusionTickUs));
    const int32_t model_us = static_cast<int32_t>(std::lround(
        (static_cast<float>(static_cast<int16_t>(pair_model_t_ - latch)) - write_ticks) *
        kFusionTickUs));
    if (us < -1000 || us > 8000) {   // not a transfer time at all
      ts_rejects_++;
      return;
    }
    if (ts_pairs_ == 0) {
      ts_delay_min_us_ = us;
      ts_delay_mean_us_ = static_cast<float>(us);
      ts_model_mean_us_ = static_cast<float>(model_us);
    } else {
      if (us < ts_delay_min_us_) { ts_delay_min_us_ = us; }
      ts_delay_mean_us_ += (static_cast<float>(us) - ts_delay_mean_us_) / 16.0f;
      ts_model_mean_us_ += (static_cast<float>(model_us) - ts_model_mean_us_) / 16.0f;
    }
    ts_pairs_++;
  }
#else
  void ProbeGyro(const FusionWord&, uint16_t) {}
  void ProcessTimestamp(const FusionWord&) {}
#endif

  FUSION_MAINLOOP void DeadReckon(uint16_t lost) {
    if (!initialized_) { return; }
    const float dt_s = static_cast<float>(lost) * (T_ticks_ + dT_) *
        kFusionTickUs * 1.0e-6f;
    const float theta[3] = {omega_c_[0] * dt_s, omega_c_[1] * dt_s,
                            omega_c_[2] * dt_s};
    fusion_math::Rotate(q_, theta);
    // The trapezoid's previous sample is no longer adjacent.
    have_prev_omega_ = false;
  }

  void WrapModel() {
    while (model_t_ >= 65536.0f) { model_t_ -= 65536.0f; }
    while (model_t_ < 0.0f) { model_t_ += 65536.0f; }
  }

  static float Weight(float x, float full, float zero) {
    if (x <= full) { return 1.0f; }
    if (x >= zero) { return 0.0f; }
    return (zero - x) / (zero - full);
  }

  FUSION_MAINLOOP void ProcessAccel(const FusionWord& w) {
    counters_.accel_words++;
    float a[3];
    for (int i = 0; i < 3; i++) {
      const float raw = static_cast<float>(w.v[i]) * kFusionAccelScale;
      // Uncorrected, low-passed (~1 s) for the six-position calibration.
      accel_raw_lp_[i] += (1.0f / kFusionAccelOdrHz) * (raw - accel_raw_lp_[i]);
      a[i] = (raw - params_.accel_bias[i]) / params_.accel_scale[i];
    }
    const float an = std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    if (an < 0.1f) {
      // Free fall: no gravity reference.  Drop the held correction
      // (it is applied on every gyro word) and the stationarity state.
      for (auto& v : corr_) { v = 0.0f; }
      quasi_static_count_ = 0;
      converge_calm_count_ = 0;
      stat_accel_ok_ = false;
      stat_words_ = 0;
      stat_active_ = false;
      return;
    }
    float a_n[3];
    for (int i = 0; i < 3; i++) { a_n[i] = a[i] / an; }

    if (!initialized_) {
      InitFromAccel(a_n);
      return;
    }

    // Stationarity, accelerometer side: magnitude within tolerance and
    // the gravity direction within stat_dir_deg of where the candidate
    // window began (a slow tilt passes the gyro threshold but not this).
    {
      bool ok = std::abs(an - 1.0f) < params_.stat_acc_tol;
      if (ok && stat_have_ref_ && stat_words_ > 0) {
        const float dot = a_n[0] * stat_ref_[0] + a_n[1] * stat_ref_[1] +
            a_n[2] * stat_ref_[2];
        ok = dot > params_.stat_dir_cos;
      }
      if (!ok || stat_words_ == 0) {
        std::memcpy(stat_ref_, a_n, sizeof(stat_ref_));
        stat_have_ref_ = true;
      }
      if (!ok && relearn_ms_ == 0) {   // a host-requested relearn vouches for stillness
        stat_words_ = 0;
        stat_active_ = false;
      }
      stat_accel_ok_ = ok;
    }

    float g_b[3];
    fusion_math::BodyUp(q_, g_b);
    float e[3] = {
      a_n[1] * g_b[2] - a_n[2] * g_b[1],
      a_n[2] * g_b[0] - a_n[0] * g_b[2],
      a_n[0] * g_b[1] - a_n[1] * g_b[0],
    };
    const float en = std::sqrt(e[0] * e[0] + e[1] * e[1] + e[2] * e[2]);
    if (en > params_.innovation_max) {
      const float s = params_.innovation_max / en;
      for (auto& v : e) { v *= s; }
    }

    const float omega_n = std::sqrt(
        omega_c_[0] * omega_c_[0] + omega_c_[1] * omega_c_[1] +
        omega_c_[2] * omega_c_[2]);
    const float w_a =
        Weight(std::abs(an - 1.0f), params_.acc_tol_full, params_.acc_tol_zero) *
        Weight(omega_n, params_.omega_full, params_.omega_zero);
    if (en < params_.converge_innovation_max && w_a >= 0.999f) {
      if (converge_calm_count_ < 255) { converge_calm_count_++; }
    } else {
      converge_calm_count_ = 0;
    }
    const float kp = words_since_init_ < params_.init_words ?
        params_.kp_init : params_.kp;
    for (int i = 0; i < 3; i++) { corr_[i] = kp * w_a * e[i]; }

    if (w_a >= 0.999f) {
      if (quasi_static_count_ < 255) { quasi_static_count_++; }
    } else {
      quasi_static_count_ = 0;
    }
    if (quasi_static_count_ >= params_.quasi_static_words) {
      const float dt_a = 1.0f / kFusionAccelOdrHz;
      for (int i = 0; i < 3; i++) {
        bias_[i] -= params_.ki * e[i] * dt_a;
        if (bias_[i] > params_.bias_max) { bias_[i] = params_.bias_max; }
        if (bias_[i] < -params_.bias_max) { bias_[i] = -params_.bias_max; }
      }
    }
  }

  FUSION_MAINLOOP void InitFromAccel(const float* a_n) {
    if (!bias_seeded_) {
      // Cold start: begin from the bias the host saved (imu_cal.gyro_bias)
      // rather than zero.  Once only -- a re-initialization keeps what has
      // been learned since, which is better than the saved value.
      for (int i = 0; i < 3; i++) { bias_[i] = params_.gyro_bias0[i]; }
      bias_seeded_ = true;
    }
    // Tilt quaternion: R(q_tilt) a_n = z.
    float q_tilt[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    const float c = a_n[2];  // a_n . z
    float axis[3] = {a_n[1], -a_n[0], 0.0f};  // a_n x z
    const float s = std::sqrt(axis[0] * axis[0] + axis[1] * axis[1]);
    if (s > 1.0e-6f) {
      const float angle = std::atan2(s, c);
      const float half = 0.5f * angle;
      const float sh = std::sin(half) / s;
      q_tilt[0] = std::cos(half);
      q_tilt[1] = axis[0] * sh;
      q_tilt[2] = axis[1] * sh;
      q_tilt[3] = 0.0f;
    } else if (c < 0.0f) {
      // Upside down: 180 degrees about x.
      q_tilt[0] = 0.0f; q_tilt[1] = 1.0f; q_tilt[2] = 0.0f; q_tilt[3] = 0.0f;
    }

    const float target = keep_heading_ ? heading_old_ : 0.0f;
    const float psi = target - fusion_math::Yaw(q_tilt);
    const float q_z[4] = {std::cos(0.5f * psi), 0.0f, 0.0f, std::sin(0.5f * psi)};
    fusion_math::QuatMul(q_z, q_tilt, q_);
    fusion_math::Normalize(q_);

    for (auto& v : corr_) { v = 0.0f; }
    have_prev_omega_ = false;
    quasi_static_count_ = 0;
    converge_calm_count_ = 0;
    words_since_init_ = 0;
    converged_ = false;
    if (!keep_heading_) {
      // Cold start.  After a re-initialization ReInit() armed the hold
      // already; re-arming here would cost the host an extra sentinel.
      sentinel_hold_ = params_.sentinel_hold_replies;
    }
    hist_count_ = 0;
    hist_head_ = 0;
    initialized_ = true;
    keep_heading_ = false;
  }

  FUSION_MAINLOOP void ReInit(ReinitReason reason) {
    if (initialized_) {
      heading_old_ = fusion_math::Yaw(q_);
      keep_heading_ = true;
      counters_.reinits++;
    }
    initialized_ = false;
    converged_ = false;
    hist_count_ = 0;
    hist_head_ = 0;
    have_prev_omega_ = false;
    have_seq_ = false;
    sentinel_hold_ = params_.sentinel_hold_replies;
    last_reinit_reason_ = reason;
    relearn_ms_ = 0;   // the host's assurance of stillness does not outlive a restart
  }

  FUSION_MAINLOOP void UpdateStatus() {
    if (!status_) { return; }
    auto& s = *status_;
    s.initialized = initialized_;
    s.converged = converged_;
    s.toggle = toggle_ ? 1 : 0;
    for (int i = 0; i < 4; i++) { s.q[i] = q_[i]; }
    for (int i = 0; i < 3; i++) {
      s.bias[i] = bias_[i];
      s.omega[i] = omega_c_[i];
    }
    s.odr_actual_hz = 1.0e6f / ((T_ticks_ + dT_) * kFusionTickUs);
    s.freq_fine = control_ ? static_cast<int8_t>(control_->freq_fine.load()) : 0;
    s.init_state = control_ ? control_->init_state.load() : 0;
    s.gyro_words = counters_.gyro_words;
    s.accel_words = counters_.accel_words;
    s.stationary = stat_active_;
    s.stationary_words = stationary_words_;
    for (int i = 0; i < 3; i++) { s.accel_raw_g[i] = accel_raw_lp_[i]; }
    s.gyro_gaps = counters_.gyro_gaps;
    s.gap_slots = counters_.gap_slots;
    s.mailbox_overflow = mailbox_ ? mailbox_->overflow() : 0;
    s.mailbox_max_depth = mailbox_ ? mailbox_->max_depth() : 0;
    s.fifo_overruns = counters_.overruns;
    s.resyncs = counters_.resyncs;
    s.reinits = counters_.reinits;
    s.last_reinit_reason = last_reinit_reason_;
    s.stall_passes = counters_.stall_passes;
    s.arrival_unknown = counters_.arrival_unknown;
    s.phase_unc_us = static_cast<uint32_t>(
        std::abs(last_floor_) * 4 + (windows_done_ == 0 ? 500 : 0));
#ifdef MOTEUS_TS_PROBE
    s.ts_delay_min_us = ts_delay_min_us_;
    s.ts_delay_mean_us = ts_delay_mean_us_;
    s.ts_model_mean_us = ts_model_mean_us_;
    s.ts_pairs = ts_pairs_;
    s.ts_rejects = ts_rejects_;
#endif
  }

  FUSION_MAINLOOP void UpdateReplyStatus() {
    auto& s = *status_;
    s.sentinel_replies = counters_.sentinel_replies;
    s.valid_replies = counters_.valid_replies;
    s.toggle = toggle_ ? 1 : 0;
    s.arrival_unknown = counters_.arrival_unknown;
    s.stall_passes = counters_.stall_passes;
  }

  FusionMailbox* mailbox_ = nullptr;
  FusionControl* control_ = nullptr;
  History* history_ = nullptr;
  aux::ImuFusionStatus* status_ = nullptr;
  Params params_;

  // Filter state.
  bool initialized_ = false;
  bool converged_ = false;
  float q_[4] = {1.0f, 0.0f, 0.0f, 0.0f};
  float bias_[3] = {};
  bool bias_seeded_ = false;
  float omega_c_[3] = {};
  float omega_prev_[3] = {};
  float corr_[3] = {};
  bool have_prev_omega_ = false;
  uint8_t quasi_static_count_ = 0;
  uint8_t converge_calm_count_ = 0;
  uint32_t words_since_init_ = 0;
  uint8_t sentinel_hold_ = 0;
  bool keep_heading_ = false;
  float heading_old_ = 0.0f;
  ReinitReason last_reinit_reason_ = kReinitNone;

  // Sample clock.
  bool have_seq_ = false;
  uint16_t last_seq_ = 0;
  float model_t_ = 0.0f;
  float T_ticks_ = kFusionNominalTicks;
  float dT_ = 0.0f;
  int32_t sub_min_[kSubWindows] = {};
  uint16_t sub_len_ = 16;
  uint8_t sub_idx_ = 0;
  uint16_t win_count_ = 0;
  uint16_t win_len_ = 128;
  uint16_t windows_done_ = 0;
  int32_t last_floor_ = 0;
  uint16_t ms_since_gyro_ = 0;

  // History ring.
  uint16_t hist_head_ = 0;
  uint16_t hist_count_ = 0;
  uint16_t newest_seq_ = 0;

  // Reply bookkeeping.
  bool have_last_reply_seq_ = false;
  uint16_t last_reply_seq_ = 0;
  bool toggle_ = false;
  uint8_t pending_unknown_ = 0;
  bool frame_unknown_ = false;
  bool stall_latched_ = false;
  int32_t last_request_age_ticks_ = 0;

  uint16_t seen_resync_ = 0;
  uint16_t seen_running_ = 0;
  uint16_t seen_overrun_ = 0;
#ifdef MOTEUS_TS_PROBE
  // Transfer-delay probe state (EvaluateProbe).
  uint16_t seen_probe_ = 0;
  bool have_last_gyro_ = false;
  uint16_t last_gyro_t_ = 0;
  uint16_t last_gyro_seq_ = 0;
  uint8_t last_gyro_cnt_ = 0;
  bool ts_wait_gyro_ = false;
  uint8_t ts_wait_cnt_ = 0;
  uint32_t ts_wait_value_ = 0;
  bool next_armed_ = false;
  uint32_t next_chip_ = 0;
  uint16_t next_seq_ = 0;
  bool pair_valid_ = false;
  uint32_t pair_chip_ = 0;
  uint16_t pair_t_ = 0;
  uint16_t pair_model_t_ = 0;
  bool probe_valid_ = false;
  uint32_t probe_chip_ = 0;
  uint16_t probe_t_ = 0;
  uint32_t ts_pairs_ = 0;
  uint16_t ts_rejects_ = 0;
  int32_t ts_delay_min_us_ = 0;
  float ts_delay_mean_us_ = 0.0f;
  float ts_model_mean_us_ = 0.0f;
#endif

  float accel_raw_lp_[3] = {};

  // Stationary bias learning state.
  float omega_lp_[3] = {};
  bool stat_lp_reset_ = false;
  uint16_t relearn_ms_ = 0;
  uint16_t stat_words_ = 0;
  uint8_t stat_over_ = 0;
  bool stat_active_ = false;
  bool stat_accel_ok_ = false;
  bool stat_have_ref_ = false;
  float stat_ref_[3] = {};
  uint32_t stationary_words_ = 0;

  Counters counters_;
};

/// Everything the fusion needs that must not live in the pool or in
/// each AuxPort: one static instance, claimed by the port that runs the
/// fusion (docs §5.8).
struct FusionStorage {
  FusionMailbox mailbox;
  FusionControl control;
  ImuFusion::History history;
  ImuFusion fusion;
};

}
