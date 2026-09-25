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

struct FusionWord {
  uint16_t t = 0;       // TIM3 ticks at I2C completion
  uint16_t seq = 0;     // gyro sequence number (assigned before drop decision)
  int16_t v[3] = {};    // raw sensor words
  uint8_t tag = 0;      // FusionTag
  uint8_t flags = 0;    // reserved
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

  void Reset() {
    resync_count.store(0);
    running_count.store(0);
    overrun_count.store(0);
    freq_fine.store(0);
    init_state.store(0);
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
    uint16_t latency_comp_ticks = 0; // added to the request stamp
    uint16_t tracker_window_first = 128;
    uint16_t tracker_window = 1024;
    // Stationary gyro-bias learning, all three axes (the accelerometer
    // innovation cannot see the bias about the vertical).  While every
    // axis reads within stat_omega_max of the current bias, |a| is within
    // stat_acc_tol of g and the gravity direction has not moved by more
    // than stat_dir_deg since the window began, for stat_qualify_words
    // samples, the residual rate is the bias error and is blended in
    // with per-sample gain stat_gain (time constant ~10 s at 960 Hz).
    float stat_omega_max = 0.0174533f;   // 1 dps
    uint8_t stat_over_samples = 8;       // consecutive over-threshold samples that reset the window
    float stat_acc_tol = 0.02f;
    float stat_dir_cos = 0.99999391f;    // cos(0.2 deg)
    uint16_t stat_qualify_words = 960;   // 1 s
    float stat_gain = 1.0e-4f;
    // Startup capture: for the first stat_fast_words of qualified
    // stationary time after a cold start the bias is taken in with a fast
    // gain (~0.1 s), so a board that boots at rest (the crane) does not
    // spend ~45 s with a 1.7 deg tilt error while the slow learners
    // discover a 0.5 dps bias.
    float stat_gain_fast = 1.0e-2f;
    uint16_t stat_fast_words = 1920;     // 2 s
    // Accelerometer calibration, applied to the raw reading before use:
    // a = (raw - accel_bias) / accel_scale, in g.  Copied from the
    // persistent `imu_cal` config group by the aux port (fw/imu_cal.h).
    float accel_bias[3] = {0.0f, 0.0f, 0.0f};
    float accel_scale[3] = {1.0f, 1.0f, 1.0f};
  };

  enum ReinitReason : uint8_t {
    kReinitNone = 0,
    kReinitResync = 1,
    kReinitOverrun = 2,
    kReinitGap = 3,
  };

  static constexpr size_t kHistorySize = 64;
  using History = std::array<FusionHistoryEntry, kHistorySize>;

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

  FUSION_MAINLOOP void Reset() {
    initialized_ = false;
    converged_ = false;
    q_[0] = 1.0f; q_[1] = q_[2] = q_[3] = 0.0f;
    for (auto& v : bias_) { v = 0.0f; }
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
    stat_active_ = false;
    stat_accel_ok_ = false;
    stat_have_ref_ = false;
    stationary_words_ = 0;
    for (auto& v : accel_raw_lp_) { v = 0.0f; }
    last_request_age_ticks_ = 0;
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
    if (!initialized_ || !converged_ || hist_count_ == 0) {
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
    if (hist_count_ == 0) { return false; }
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
  }

  FUSION_MAINLOOP void Process(const FusionWord& w) {
    switch (w.tag) {
      case kFusionTagGyro: { ProcessGyro(w); break; }
      case kFusionTagAccel: { ProcessAccel(w); break; }
      case kFusionTagTimestamp: { counters_.ts_words++; break; }
      default: { counters_.other_words++; break; }
    }
  }

  FUSION_MAINLOOP void ProcessGyro(const FusionWord& w) {
    counters_.gyro_words++;

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
      e.t = static_cast<uint16_t>(static_cast<uint32_t>(model_t_ + 0.5f) & 0xffff);
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
      const float wmax = std::max(std::abs(omega[0]),
                                  std::max(std::abs(omega[1]), std::abs(omega[2])));
      // A brief bump (table vibration) neither resets the window nor
      // feeds the learner; sustained motion resets it.
      const bool over = wmax > params_.stat_omega_max;
      stat_over_ = over ? static_cast<uint8_t>(std::min(255, stat_over_ + 1)) : 0;
      if (stat_over_ >= params_.stat_over_samples || !stat_accel_ok_) {
        stat_words_ = 0;
        stat_active_ = false;
      } else if (!over) {
        if (stat_words_ < 65535) { stat_words_++; }
        if (stat_words_ >= params_.stat_qualify_words) {
          stat_active_ = true;
          stationary_words_++;
          const float gain = stationary_words_ <= params_.stat_fast_words ?
              params_.stat_gain_fast : params_.stat_gain;
          for (int i = 0; i < 3; i++) {
            bias_[i] += gain * omega[i];
            if (bias_[i] > params_.bias_max) { bias_[i] = params_.bias_max; }
            if (bias_[i] < -params_.bias_max) { bias_[i] = -params_.bias_max; }
          }
        }
      }
    }
  }

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
      if (!ok) {
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

  float accel_raw_lp_[3] = {};

  // Stationary bias learning state.
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
