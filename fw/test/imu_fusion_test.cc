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

#include "fw/imu_cal.h"
#include "fw/imu_fusion.h"

#include <cmath>
#include <limits>
#include <memory>
#include <random>

#include <boost/test/auto_unit_test.hpp>

using namespace moteus;
namespace fm = fusion_math;

namespace {

double AngleDeg(const float* a, const float* b) {
  // Stable for tiny angles (acos(dot) is not, in float or double).
  double av[4], bv[4];
  double na = 0.0, nb = 0.0;
  for (int i = 0; i < 4; i++) {
    av[i] = a[i]; bv[i] = b[i];
    na += av[i] * av[i]; nb += bv[i] * bv[i];
  }
  na = std::sqrt(na); nb = std::sqrt(nb);
  double dot = 0.0;
  for (int i = 0; i < 4; i++) {
    av[i] /= na; bv[i] /= nb;
    dot += av[i] * bv[i];
  }
  const double sign = dot < 0.0 ? -1.0 : 1.0;
  double dm = 0.0, dp = 0.0;
  for (int i = 0; i < 4; i++) {
    dm += (av[i] - sign * bv[i]) * (av[i] - sign * bv[i]);
    dp += (av[i] + sign * bv[i]) * (av[i] + sign * bv[i]);
  }
  // |a-b| = 2 sin(theta/4), |a+b| = 2 cos(theta/4) for unit quaternions.
  return 4.0 * std::atan2(std::sqrt(dm), std::sqrt(dp)) * 180.0 / M_PI;
}

double VecAngleDeg(const float* a, const float* b) {
  double d = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  if (d > 1.0) { d = 1.0; }
  if (d < -1.0) { d = -1.0; }
  return std::acos(d) * 180.0 / M_PI;
}

/// Simulated chip + ISR: generates FIFO words for a rigid body with a
/// constant body-frame angular velocity, exactly as the ISR would push
/// them, and lets tests query the fusion like the CAN path does.
struct Sim {
  Sim() : storage(new FusionStorage()) {
    fusion.Attach(&storage->mailbox, &storage->control, &storage->history,
                  &status);
    // These tests check the reply against the truth at the request
    // itself; the production default compensates the sensor chain
    // (FusionLatencyCompensation covers that).
    fusion.params()->latency_comp_ticks = 0;
    q_true[0] = 1.0f; q_true[1] = q_true[2] = q_true[3] = 0.0f;
  }

  std::unique_ptr<FusionStorage> storage;
  ImuFusion fusion;
  aux::ImuFusionStatus status;

  float q_true[4];
  float omega[3] = {0.0f, 0.0f, 0.0f};    // rad/s, body frame
  float omega_prev[3] = {0.0f, 0.0f, 0.0f};
  float gyro_bias[3] = {0.0f, 0.0f, 0.0f};  // rad/s added to the raw gyro
  float accel_bias[3] = {0.0f, 0.0f, 0.0f}; // g added to the raw accel
  double T_true = kFusionNominalTicks;      // ticks per sample
  double t_sample = 0.0;                    // true time of the last sample
  double dmin = 0.0;                        // constant transfer delay, ticks
  double jitter = 0.0;                      // uniform extra delay, ticks
  uint32_t chip_epoch = 0x12345678;         // chip timestamp counter at sample 0
  bool ts_first = false;                    // timestamp word before the slot's gyro word (the real chip)
  double ms_acc = 0.0;                      // the main loop's millisecond poll, driven by Step()
  uint16_t seq = 0;
  int k = 0;
  bool drain_each = true;
  bool freefall = false;                    // accel reads zero
  std::mt19937 rng{7};

  void SetTilt(float axis_x, float axis_y, float axis_z, float angle_rad) {
    const float n = std::sqrt(axis_x * axis_x + axis_y * axis_y + axis_z * axis_z);
    const float theta[3] = {axis_x / n * angle_rad, axis_y / n * angle_rad,
                            axis_z / n * angle_rad};
    q_true[0] = 1.0f; q_true[1] = q_true[2] = q_true[3] = 0.0f;
    fm::Rotate(q_true, theta);
  }

  static int16_t Clamp16(double v) {
    if (v > 32767.0) { return 32767; }
    if (v < -32768.0) { return -32768; }
    return static_cast<int16_t>(std::lround(v));
  }

  uint16_t Arrival() {
    std::uniform_real_distribution<double> u(0.0, jitter);
    const double t = t_sample + dmin + (jitter > 0.0 ? u(rng) : 0.0);
    return static_cast<uint16_t>(static_cast<uint64_t>(std::llround(t)) & 0xffff);
  }

  // One gyro sample period.  drop: the ISR could not push this gyro
  // word (mailbox full); seq still advances.
  void Step(bool drop = false) {
    // The true rate is piecewise linear between samples (a step in
    // omega ramps over one interval), which is what the trapezoid
    // integrator assumes and what a band-limited gyro delivers.
    // The gyro reports whole LSBs; the truth is what the gyro reports
    // (the test measures the filter, not the sensor's quantization).
    FusionWord g;
    float omega_q[3];
    for (int i = 0; i < 3; i++) {
      g.v[i] = Clamp16((omega[i] + gyro_bias[i]) / kFusionGyroScale);
      // The bias the gyro can represent (whole LSBs), so the truth does
      // not drift by the quantization remainder.
      const float bias_q = static_cast<float>(Clamp16(gyro_bias[i] / kFusionGyroScale)) * kFusionGyroScale;
      omega_q[i] = static_cast<float>(g.v[i]) * kFusionGyroScale - bias_q;
    }
    const double dt_s = T_true * kFusionTickUs * 1.0e-6;
    const float theta[3] = {
      static_cast<float>(0.5 * (omega_prev[0] + omega_q[0]) * dt_s),
      static_cast<float>(0.5 * (omega_prev[1] + omega_q[1]) * dt_s),
      static_cast<float>(0.5 * (omega_prev[2] + omega_q[2]) * dt_s)};
    fm::Rotate(q_true, theta);
    std::memcpy(omega_prev, omega_q, sizeof(omega_q));
    t_sample += T_true;
    seq++;
    k++;

    g.t = Arrival();
    g.seq = seq;
    g.tag = kFusionTagGyro;
    g.flags = static_cast<uint8_t>(k & 3);   // TAG_CNT
    if (k % 32 == 0 && ts_first) { PushTimestamp(g); }
    if (!drop) { storage->mailbox.Push(g); }

    if (k % 8 == 0) {
      float up[3];
      fm::BodyUp(q_true, up);
      FusionWord a;
      a.t = g.t;
      a.seq = seq;
      a.tag = kFusionTagAccel;
      for (int i = 0; i < 3; i++) {
        a.v[i] = freefall ? 0 :
            Clamp16((up[i] + accel_bias[i]) / kFusionAccelScale);
      }
      storage->mailbox.Push(a);
    }
    if (k % 32 == 0 && !ts_first) { PushTimestamp(g); }
    if (drain_each) { fusion.Drain(16); }
    ms_acc += 1000.0 / kFusionGyroOdrHz;
    while (ms_acc >= 1.0) {
      fusion.PollMillisecond();
      ms_acc -= 1.0;
    }
  }

  // Acquisition stops for `seconds`: time passes with no words at all,
  // then the driver's restart re-initializes the fusion (resync_count).
  void Outage(double seconds) {
    for (int ms = 0; ms < static_cast<int>(seconds * 1000); ms++) { fusion.PollMillisecond(); }
    t_sample += seconds * kFusionGyroOdrHz * T_true;
    storage->control.resync_count.fetch_add(1);
    fusion.Drain(16);
  }

  // The chip's timestamp counter at this slot's data-ready, 48 LSB per
  // sample (DS Table 145).  The real chip writes it before the slot's
  // gyro word (bench 2026-10-06): ts_first.
  void PushTimestamp(const FusionWord& g) {
    FusionWord ts;
    ts.t = g.t;
    ts.seq = seq;
    ts.tag = kFusionTagTimestamp;
    ts.flags = g.flags;
    const uint32_t chip = chip_epoch + static_cast<uint32_t>(k) * kFusionChipTicksPerSample;
    ts.v[0] = static_cast<int16_t>(chip & 0xffff);
    ts.v[1] = static_cast<int16_t>(chip >> 16);
    storage->mailbox.Push(ts);
  }

  void Run(double seconds) {
    const int n = static_cast<int>(seconds * kFusionGyroOdrHz);
    for (int i = 0; i < n; i++) { Step(); }
  }

  // The driver's live read of TIMESTAMP0-3 (the transfer-delay probe):
  // the counter latched at true time t_latch (ticks from sample 0), the
  // read seen complete kProbeLatchTicks later.  chip_error: a corrupted
  // read (a byte rolling over mid-transfer reads 256 LSB high).
  void Probe(double t_latch, int32_t chip_error = 0) {
    const double lsb = t_latch * kFusionChipTicksPerSample / T_true;
    storage->control.probe_chip.store(
        chip_epoch + static_cast<uint32_t>(std::llround(lsb) + chip_error));
    storage->control.probe_t.store(static_cast<uint16_t>(
        static_cast<uint64_t>(std::llround(t_latch + ImuFusion::kProbeLatchTicks)) & 0xffff));
    storage->control.probe_count.fetch_add(1);
    fusion.Drain(FusionMailbox::kSize);   // like BeginFrame: everything queued
  }

  // Truth at an arbitrary true time (constant omega propagation from
  // the last sample; exact for constant omega, negative dt allowed).
  void TruthAt(double t_ticks, float* q) const {
    std::memcpy(q, q_true, sizeof(q_true));
    const double dt_s = (t_ticks - t_sample) * kFusionTickUs * 1.0e-6;
    const float theta[3] = {
      static_cast<float>(omega_prev[0] * dt_s),
      static_cast<float>(omega_prev[1] * dt_s),
      static_cast<float>(omega_prev[2] * dt_s)};
    fm::Rotate(q, theta);
  }

  // A CAN request whose SOF is at true time t_ticks: the hardware
  // stamp is that instant in the same tick clock.
  Quat48Words Request(double t_ticks, uint32_t fill = 0) {
    fusion.BeginFrame(fill);
    return fusion.Reply(
        static_cast<uint16_t>(static_cast<uint64_t>(std::llround(t_ticks)) & 0xffff),
        true);
  }

  // Error (deg) of a valid reply against the truth at its request time.
  double ReplyErrorDeg(double t_ticks) {
    const auto words = Request(t_ticks);
    BOOST_TEST_REQUIRE(!Quat48IsSentinel(words));
    float q[4];
    DecodeQuat48(words, q);
    float truth[4];
    TruthAt(t_ticks, truth);
    return AngleDeg(q, truth);
  }
};

}  // namespace

BOOST_AUTO_TEST_CASE(FusionStaticInit) {
  Sim sim;
  sim.SetTilt(0.0f, 1.0f, 0.0f, 20.0f * M_PI / 180.0f);
  sim.Run(0.1);
  BOOST_TEST(sim.fusion.initialized());
  BOOST_TEST(!sim.fusion.converged());
  // Not converged: sentinel.
  BOOST_TEST(Quat48IsSentinel(sim.Request(sim.t_sample)));
  sim.Run(1.5);
  BOOST_TEST(sim.fusion.converged());

  float up_est[3];
  float up_true[3];
  fm::BodyUp(sim.fusion.q(), up_est);
  fm::BodyUp(sim.q_true, up_true);
  BOOST_TEST(VecAngleDeg(up_est, up_true) < 0.05);
  BOOST_TEST(std::abs(fm::Yaw(sim.fusion.q())) < 1e-3f);
  BOOST_TEST(AngleDeg(sim.fusion.q(), sim.q_true) < 0.05);

  // The three-reply sentinel hold, then valid.
  // The three-reply hold counts the sentinel sent before convergence.
  int sentinels = 0;
  for (int i = 0; i < 3; i++) {
    if (Quat48IsSentinel(sim.Request(sim.t_sample))) { sentinels++; }
  }
  BOOST_TEST(sentinels == 2);
  sim.Step();
  BOOST_TEST(sim.ReplyErrorDeg(sim.t_sample) < 0.05);
  BOOST_TEST(sim.status.valid_replies == 2);
  BOOST_TEST(sim.status.sentinel_replies == 3);
  BOOST_TEST(sim.status.gyro_gaps == 0);
  BOOST_TEST(sim.status.resyncs == 0);
  BOOST_TEST(sim.status.mailbox_overflow == 0);
  BOOST_TEST(sim.fusion.counters().ts_words > 0);
}

namespace {
// Steps until the filter reports converged; returns the time since the
// start of the simulation in seconds (or a negative value on timeout).
uint16_t Stamp(double t_ticks) {
  return static_cast<uint16_t>(static_cast<uint64_t>(std::llround(t_ticks)) & 0xffff);
}

double StepUntilConverged(Sim* sim, double limit_s) {
  const int n = static_cast<int>(limit_s * kFusionGyroOdrHz);
  for (int i = 0; i < n; i++) {
    sim->Step();
    if (sim->fusion.converged()) {
      return static_cast<double>(sim->k) / kFusionGyroOdrHz;
    }
  }
  return -1.0;
}

double TiltErrorDeg(const Sim& sim) {
  float up_est[3];
  float up_true[3];
  fm::BodyUp(sim.fusion.q(), up_est);
  fm::BodyUp(sim.q_true, up_true);
  return VecAngleDeg(up_est, up_true);
}
}  // namespace

BOOST_AUTO_TEST_CASE(FusionStillStartConvergesEarly) {
  // A board that boots at rest: the first accel word already gives the
  // tilt, so replies become valid at the 0.2 s minimum plus the 0.1 s
  // agreement window at most, not after the 1 s fallback.
  Sim sim;
  sim.SetTilt(1.0f, 0.3f, 0.0f, 25.0f * M_PI / 180.0f);
  const double t = StepUntilConverged(&sim, 1.5);
  BOOST_TEST(t >= 0.2);
  BOOST_TEST(t < 0.35);
  BOOST_TEST(TiltErrorDeg(sim) < 0.05);
}

BOOST_AUTO_TEST_CASE(FusionDisturbedStartWaitsForTilt) {
  // The first accel word carries 0.1 g of linear acceleration, so the
  // initial tilt is ~5.7 deg off.  The filter must not report converged
  // until the estimate agrees with gravity again, and must still do so
  // well before the 1 s fallback.
  Sim sim;
  sim.accel_bias[0] = 0.1f;
  for (int i = 0; i < 10; i++) { sim.Step(); }  // first accel word at 8
  BOOST_TEST(sim.fusion.initialized());
  BOOST_TEST(TiltErrorDeg(sim) > 5.0);
  sim.accel_bias[0] = 0.0f;
  const double t = StepUntilConverged(&sim, 1.5);
  BOOST_TEST(t > 0.2);
  BOOST_TEST(t < 0.8);
  BOOST_TEST(TiltErrorDeg(sim) < 0.25);
}

BOOST_AUTO_TEST_CASE(FusionMovingStartFallsBackToOneSecond) {
  // Rotating at 1 rad/s the accelerometer correction is down-weighted,
  // so the early path never qualifies and the 1 s count decides.
  Sim sim;
  sim.omega[0] = 1.0f;
  const double t = StepUntilConverged(&sim, 1.5);
  BOOST_TEST(t > 0.95);
  BOOST_TEST(t < 1.05);
}

BOOST_AUTO_TEST_CASE(FusionConstantRotation) {
  Sim sim;
  sim.Run(0.05);        // initialize level, heading 0
  sim.omega[2] = 2.0f;  // about body z, which stays world-up
  sim.Run(2.0);
  for (int i = 0; i < 3; i++) { sim.Request(sim.t_sample); }
  BOOST_TEST(sim.fusion.converged());

  double max_err = 0.0;
  for (int i = 0; i < 200; i++) {
    sim.Step();
    // Requests inside the history, and up to 7 ms after the newest.
    const double offsets[] = {-30.5, -3.25, -0.5, 0.0, 0.4, 100.0, 1700.0};
    for (const double off : offsets) {
      const double err = sim.ReplyErrorDeg(sim.t_sample + off);
      if (err > max_err) { max_err = err; }
    }
  }
  BOOST_TEST(max_err < 0.02);

  // 9 ms after the newest sample: outside the extrapolation limit.
  BOOST_TEST(Quat48IsSentinel(sim.Request(sim.t_sample + 2250.0)));
  // Older than the 64-sample history: sentinel too.
  BOOST_TEST(Quat48IsSentinel(sim.Request(sim.t_sample - 70.0 * kFusionNominalTicks)));
  // Just inside the history: valid.
  BOOST_TEST(!Quat48IsSentinel(sim.Request(sim.t_sample - 60.0 * kFusionNominalTicks)));
}

BOOST_AUTO_TEST_CASE(FusionTrackerLearnsRateAndPhase) {
  Sim sim;
  sim.T_true = kFusionNominalTicks * (1.0 + 0.0015);
  sim.dmin = 83.0;   // one 7-byte transfer
  sim.jitter = 80.0;  // up to one more
  sim.fusion.params()->latency_comp_ticks = 83;
  sim.Run(0.05);
  sim.omega[0] = 1.0f;
  sim.Run(5.0);
  const double period = sim.fusion.period_ticks();
  BOOST_TEST(std::abs(period - sim.T_true) / sim.T_true < 1e-4);
  BOOST_TEST(std::abs(sim.fusion.last_floor()) <= 2);
  BOOST_TEST(sim.fusion.windows_done() >= 4);
  for (int i = 0; i < 3; i++) { sim.Request(sim.t_sample); }
  double max_err = 0.0;
  for (int i = 0; i < 100; i++) {
    sim.Step();
    const double err = sim.ReplyErrorDeg(sim.t_sample + 50.0);
    if (err > max_err) { max_err = err; }
  }
  BOOST_TEST(max_err < 0.06);
  BOOST_TEST(sim.status.phase_unc_us <= 8);
}

BOOST_AUTO_TEST_CASE(FusionGapDeadReckoning) {
  Sim sim;
  sim.Run(0.05);
  sim.omega[2] = 1.0f;
  sim.Run(2.0);
  for (int i = 0; i < 3; i++) { sim.Request(sim.t_sample); }
  for (int i = 0; i < 10; i++) { sim.Step(true); }
  sim.Step();
  BOOST_TEST(sim.status.gyro_gaps == 1);
  BOOST_TEST(sim.status.gap_slots == 10);
  BOOST_TEST(sim.status.reinits == 0);
  BOOST_TEST(sim.ReplyErrorDeg(sim.t_sample) < 0.02);
}

BOOST_AUTO_TEST_CASE(FusionLongGapReinitKeepsHeading) {
  Sim sim;
  sim.Run(0.05);
  sim.omega[2] = 1.0f;
  sim.Run(1.0);           // heading now ~1 rad
  sim.omega[2] = 0.0f;
  sim.Run(1.5);
  for (int i = 0; i < 3; i++) { sim.Request(sim.t_sample); }
  const float heading_before = fm::Yaw(sim.fusion.q());
  BOOST_TEST(std::abs(heading_before - 1.0f) < 0.01f);

  for (int i = 0; i < 200; i++) { sim.Step(true); }  // > gap_max_words
  sim.Run(0.1);
  BOOST_TEST(sim.status.reinits == 1);
  BOOST_TEST(sim.status.last_reinit_reason == ImuFusion::kReinitGap);
  BOOST_TEST(sim.fusion.initialized());
  BOOST_TEST(!sim.fusion.converged());
  BOOST_TEST(Quat48IsSentinel(sim.Request(sim.t_sample)));
  sim.Run(1.5);
  int sentinels = 0;
  for (int i = 0; i < 4; i++) {
    if (Quat48IsSentinel(sim.Request(sim.t_sample))) { sentinels++; }
  }
  BOOST_TEST(sentinels == 2);  // one already consumed above
  BOOST_TEST(std::abs(fm::Yaw(sim.fusion.q()) - heading_before) < 0.01f);
  BOOST_TEST(AngleDeg(sim.fusion.q(), sim.q_true) < 0.05);
}

BOOST_AUTO_TEST_CASE(FusionTickWrap) {
  Sim sim;
  sim.t_sample = 65000.0;  // wraps within the first 0.5 s and every 262 ms
  sim.Run(0.05);
  sim.omega[1] = 0.7f;
  sim.Run(2.0);
  for (int i = 0; i < 3; i++) { sim.Request(sim.t_sample); }
  double max_err = 0.0;
  int sentinels = 0;
  for (int i = 0; i < 2000; i++) {  // ~2 s, ~8 wraps
    sim.Step();
    const auto words = sim.Request(sim.t_sample - 100.0);
    if (Quat48IsSentinel(words)) { sentinels++; continue; }
    float q[4];
    DecodeQuat48(words, q);
    float truth[4];
    sim.TruthAt(sim.t_sample - 100.0, truth);
    const double err = AngleDeg(q, truth);
    if (err > max_err) { max_err = err; }
  }
  BOOST_TEST(sentinels == 0);
  BOOST_TEST(max_err < 0.02);
  BOOST_TEST(sim.status.gyro_gaps == 0);
}

BOOST_AUTO_TEST_CASE(FusionStallMarksQueuedFrames) {
  Sim sim;
  sim.Run(2.0);
  for (int i = 0; i < 3; i++) { sim.Request(sim.t_sample); }
  BOOST_TEST(!Quat48IsSentinel(sim.Request(sim.t_sample)));

  // Main loop away: 150 words queue up, then three frames were waiting.
  sim.drain_each = false;
  for (int i = 0; i < 150; i++) { sim.Step(); }
  BOOST_TEST(sim.storage->mailbox.size() > 100);
  BOOST_TEST(Quat48IsSentinel(sim.Request(sim.t_sample, 2)));
  BOOST_TEST(sim.status.stall_passes == 1);
  BOOST_TEST(sim.fusion.pending_unknown() == 2);
  sim.drain_each = true;
  sim.Step();
  BOOST_TEST(Quat48IsSentinel(sim.Request(sim.t_sample)));
  sim.Step();
  BOOST_TEST(Quat48IsSentinel(sim.Request(sim.t_sample)));
  BOOST_TEST(sim.status.arrival_unknown == 3);
  sim.Step();
  BOOST_TEST(!Quat48IsSentinel(sim.Request(sim.t_sample)));
  BOOST_TEST(sim.status.stall_passes == 1);
  BOOST_TEST(sim.status.mailbox_overflow == 0);
}

BOOST_AUTO_TEST_CASE(FusionToggleFlipsOnlyWithNewSamples) {
  Sim sim;
  sim.Run(2.0);
  for (int i = 0; i < 3; i++) { sim.Request(sim.t_sample); }
  const auto a = sim.Request(sim.t_sample);
  const auto b = sim.Request(sim.t_sample);  // no new sample in between
  BOOST_TEST(!Quat48IsSentinel(a));
  BOOST_TEST(Quat48Toggle(a) == Quat48Toggle(b));
  sim.Step();
  const auto c = sim.Request(sim.t_sample);
  BOOST_TEST(Quat48Toggle(c) != Quat48Toggle(b));
  sim.Step();
  const auto d = sim.Request(sim.t_sample);
  BOOST_TEST(Quat48Toggle(d) != Quat48Toggle(c));
}

BOOST_AUTO_TEST_CASE(FusionGyroRate) {
  Sim sim;
  sim.gyro_bias[0] = 0.004f;
  sim.gyro_bias[2] = -0.01f;
  sim.Run(0.1);
  // Not converged yet: NaN.
  BOOST_TEST(!sim.fusion.converged());
  sim.fusion.BeginFrame(0);
  BOOST_TEST(std::isnan(sim.fusion.RateAt(Stamp(sim.t_sample), 0)));

  // At rest the stationary learner takes the bias out of the rate
  // (slowly: tau 10 s, no startup capture).
  sim.Run(60.0);
  for (int i = 0; i < 3; i++) { sim.Request(sim.t_sample); }
  for (int axis = 0; axis < 3; axis++) {
    BOOST_TEST(std::abs(sim.fusion.RateAt(Stamp(sim.t_sample), axis)) < 0.002f);
  }

  // Moving: the newest sample's rate, bias corrected, within one LSB.
  const float omega[3] = {0.3f, -0.7f, 2.0f};
  std::memcpy(sim.omega, omega, sizeof(omega));
  sim.Run(0.05);
  const auto before = sim.Request(sim.t_sample);
  sim.fusion.BeginFrame(0);
  for (int axis = 0; axis < 3; axis++) {
    BOOST_TEST(std::abs(sim.fusion.RateAt(Stamp(sim.t_sample + 100.0), axis) -
                        omega[axis]) < 1.5f * kFusionGyroScale);
  }
  // Reading the rate neither flips the toggle nor counts as a reply.
  const auto replies = sim.fusion.counters().valid_replies;
  const auto after = sim.Request(sim.t_sample);
  BOOST_TEST(Quat48Toggle(after) == Quat48Toggle(before));
  BOOST_TEST(sim.fusion.counters().valid_replies == replies + 1);

  // Past the quaternion's extrapolation limit: NaN.
  BOOST_TEST(std::isnan(sim.fusion.RateAt(Stamp(sim.t_sample + 2250.0), 0)));
}

BOOST_AUTO_TEST_CASE(FusionResyncFromControl) {
  Sim sim;
  sim.Run(2.0);
  for (int i = 0; i < 3; i++) { sim.Request(sim.t_sample); }
  BOOST_TEST(!Quat48IsSentinel(sim.Request(sim.t_sample)));
  sim.storage->control.resync_count.fetch_add(1);
  sim.Step();
  BOOST_TEST(sim.status.resyncs == 1);
  BOOST_TEST(sim.status.reinits == 1);
  BOOST_TEST(Quat48IsSentinel(sim.Request(sim.t_sample)));
  sim.Run(1.5);
  int sentinels = 0;
  for (int i = 0; i < 4; i++) {
    if (Quat48IsSentinel(sim.Request(sim.t_sample))) { sentinels++; }
  }
  BOOST_TEST(sentinels == 2);
  // Overrun behaves the same.
  sim.storage->control.overrun_count.fetch_add(1);
  sim.Step();
  BOOST_TEST(sim.status.fifo_overruns == 1);
  BOOST_TEST(sim.status.reinits == 2);
}

BOOST_AUTO_TEST_CASE(FusionRunningSeedsPeriod) {
  Sim sim;
  sim.storage->control.freq_fine.store(10);  // +1.3%
  sim.storage->control.running_count.fetch_add(1);
  sim.fusion.Drain(1);
  const float expected = 1.0e6f / (960.0f * 1.013f) / 4.0f;
  BOOST_TEST(std::abs(sim.fusion.period_ticks() - expected) < 1e-3f);
}

BOOST_AUTO_TEST_CASE(FusionMailboxShedAndOverflow) {
  FusionMailbox mb;
  FusionWord g;
  g.tag = kFusionTagGyro;
  FusionWord a;
  a.tag = kFusionTagAccel;
  for (int i = 0; i < 191; i++) { BOOST_TEST(mb.Push(g)); }
  BOOST_TEST(mb.Push(a));            // 191 -> 192
  BOOST_TEST(!mb.Push(a));           // shed above the threshold
  BOOST_TEST(mb.shed() == 1);
  for (int i = 192; i < 256; i++) { BOOST_TEST(mb.Push(g)); }
  BOOST_TEST(!mb.Push(g));           // full
  BOOST_TEST(mb.overflow() == 1);
  BOOST_TEST(mb.size() == 256);
  BOOST_TEST(mb.max_depth() == 256);
  FusionWord out;
  int n = 0;
  while (mb.Pop(&out)) { n++; }
  BOOST_TEST(n == 256);
  BOOST_TEST(mb.size() == 0);
}

BOOST_AUTO_TEST_CASE(FusionStationaryBiasLearnsVerticalAxis) {
  // Level board: z is the vertical axis, invisible to the accelerometer
  // innovation.  Only the stationary learner can remove a z bias.
  Sim sim;
  sim.gyro_bias[2] = 0.3f * M_PI / 180.0f;  // 0.3 dps about the vertical
  sim.gyro_bias[0] = 0.5f * M_PI / 180.0f;  // and a tilt-axis bias
  sim.Run(0.3);
  // Before any learning: drifting.
  const float yaw_early = fm::Yaw(sim.fusion.q());
  sim.Run(0.5);
  BOOST_TEST(std::abs(fm::Yaw(sim.fusion.q()) - yaw_early) > 0.05f * M_PI / 180.0f);
  // A cold start learns slowly on purpose (no startup capture since
  // 2026-10-07: the robot may well be moving in its first seconds): at
  // 4 s the vertical bias is only part way in.
  sim.Run(3.2);
  BOOST_TEST(std::abs(sim.fusion.bias()[2] - sim.gyro_bias[2]) > 0.5f * sim.gyro_bias[2]);
  BOOST_TEST(sim.fusion.stationary());
  // The tilt-axis bias is learned by the accel innovation as well
  // (kp/ki = 15 s) and by the stationary learner (10 s); its 1.7 deg tilt
  // transient is mostly gone by 12 s.  Heading is unobservable and holds
  // whatever it acquired, so judge tilt only.
  sim.Run(8.0);
  {
    float up_est[3]; float up_true[3];
    fm::BodyUp(sim.fusion.q(), up_est); fm::BodyUp(sim.q_true, up_true);
    BOOST_TEST(VecAngleDeg(up_est, up_true) < 0.75);
  }
  sim.Run(32.0);
  {
    float up_est[3]; float up_true[3];
    fm::BodyUp(sim.fusion.q(), up_est); fm::BodyUp(sim.q_true, up_true);
    BOOST_TEST(VecAngleDeg(up_est, up_true) < 0.1);
  }
  BOOST_TEST(std::abs(sim.fusion.bias()[0] - sim.gyro_bias[0]) < 0.2f * sim.gyro_bias[0]);
  BOOST_TEST(sim.fusion.stationary());
  BOOST_TEST(std::abs(sim.fusion.bias()[2] - sim.gyro_bias[2]) < 0.2f * sim.gyro_bias[2]);
  const float yaw_a = fm::Yaw(sim.fusion.q());
  sim.Run(10.0);
  const float yaw_b = fm::Yaw(sim.fusion.q());
  // Residual drift under 0.02 deg/s.
  BOOST_TEST(std::abs(yaw_b - yaw_a) < 0.2f * M_PI / 180.0f);
  BOOST_TEST(sim.status.stationary_words > 10000);
}

BOOST_AUTO_TEST_CASE(FusionStationaryGateRejectsSlowTilt) {
  // A slow tilt (0.3 dps about x) is under the gyro threshold but moves
  // the gravity direction, so the learner must not absorb it as bias.
  Sim sim;
  sim.Run(3.0);  // at rest: qualifies as stationary after 1 s
  const uint32_t words_at_rest = sim.status.stationary_words;
  BOOST_TEST(words_at_rest > 1000);
  sim.omega[0] = 0.3f * M_PI / 180.0f;
  sim.Run(20.0);
  BOOST_TEST(!sim.fusion.stationary());
  BOOST_TEST(std::abs(sim.fusion.bias()[0]) < 0.05f * M_PI / 180.0f);
  // The window trips within 0.2 deg of tilt (~0.7 s); no learning after.
  BOOST_TEST(sim.status.stationary_words - words_at_rest < 1000u);
  // Fast motion is rejected outright.
  sim.omega[0] = 0.0f;
  sim.omega[2] = 2.0f;
  sim.Run(5.0);
  BOOST_TEST(!sim.fusion.stationary());
}

BOOST_AUTO_TEST_CASE(FusionAccelCalibrationApplied) {
  // A 12 mg zero-g offset on x (the datasheet's typical) tilts the
  // estimate by 0.69 deg; the calibration parameters remove it.
  for (int corrected = 0; corrected < 2; corrected++) {
    Sim sim;
    sim.accel_bias[0] = 0.012f;
    if (corrected) { sim.fusion.params()->accel_bias[0] = 0.012f; }
    sim.Run(30.0);
    const double err = AngleDeg(sim.fusion.q(), sim.q_true);
    if (corrected) {
      BOOST_TEST(err < 0.05);
    } else {
      BOOST_TEST(err > 0.6);
      BOOST_TEST(err < 0.8);
    }
    BOOST_TEST(std::abs(sim.status.accel_raw_g[0] - 0.012f) < 0.002f);
    BOOST_TEST(std::abs(sim.status.accel_raw_g[2] - 1.0f) < 0.002f);
  }
}

BOOST_AUTO_TEST_CASE(FusionFreefallDropsCorrection) {
  // The gravity correction is recomputed per accel word but applied on
  // every gyro word; in free fall there is no gravity reference, so the
  // last correction must not keep rotating the estimate.
  Sim sim;
  sim.Run(0.2);  // initialized
  // The body tilts 30 deg between two accel words: a large innovation.
  sim.SetTilt(1.0f, 0.0f, 0.0f, 30.0f * M_PI / 180.0f);
  for (int i = 0; i < 8; i++) { sim.Step(); }
  sim.freefall = true;
  for (int i = 0; i < 8; i++) { sim.Step(); }
  float q0[4];
  std::memcpy(q0, sim.fusion.q(), sizeof(q0));
  sim.Run(0.5);  // zero gyro
  BOOST_TEST(AngleDeg(sim.fusion.q(), q0) < 0.01);
  BOOST_TEST(!sim.fusion.stationary());
}

BOOST_AUTO_TEST_CASE(FusionBiasLearning) {
  Sim sim;
  sim.gyro_bias[0] = 0.5f * M_PI / 180.0f;  // 0.5 dps
  sim.SetTilt(1.0f, 0.0f, 0.0f, 10.0f * M_PI / 180.0f);
  sim.Run(60.0);
  BOOST_TEST(std::abs(sim.fusion.bias()[0] - sim.gyro_bias[0]) <
             0.2f * sim.gyro_bias[0]);
  BOOST_TEST(AngleDeg(sim.fusion.q(), sim.q_true) < 0.4);
}

BOOST_AUTO_TEST_CASE(FusionNonFiniteCalibrationDoesNotPoisonState) {
  const float invalid_values[] = {
    std::numeric_limits<float>::quiet_NaN(),
    std::numeric_limits<float>::infinity(),
    -std::numeric_limits<float>::infinity(),
  };
  for (const float invalid : invalid_values) {
    Sim sim;
    // Apply a real x calibration alongside invalid y/z config values.
    // The invalid values must not poison the running quaternion or
    // prevent valid replies, and the finite calibration must still work.
    sim.accel_bias[0] = 0.012f;
    sim.Run(0.1);
    ImuCalConfig cal;
    cal.accel_bias = {0.012f, invalid, invalid};
    cal.accel_scale = {1.0f, invalid, invalid};
    for (int i = 0; i < 3; i++) {
      sim.fusion.params()->accel_bias[i] = cal.bias(i);
      sim.fusion.params()->accel_scale[i] = cal.scale(i);
    }
    sim.Run(5.0);
    for (int i = 0; i < 3; i++) { sim.Request(sim.t_sample); }
    BOOST_TEST(sim.ReplyErrorDeg(sim.t_sample) < 0.05);
  }
}

BOOST_AUTO_TEST_CASE(FusionStaleAfterTickWrap) {
  // A stopped acquisition: replies must stay sentinels even when the
  // 16-bit tick stamps have wrapped back to near the newest sample's
  // (every 262.144 ms), where the tick-based extrapolation limit alone
  // would accept the stale history again.
  Sim sim;
  sim.Run(3.0);
  sim.omega[1] = 1.0f;
  sim.Run(0.05);
  for (int i = 0; i < 4; i++) { sim.Request(sim.t_sample); }

  int polled_ms = 0;
  auto pause_to = [&](double ms) {
    for (; polled_ms < static_cast<int>(ms); polled_ms++) {
      sim.fusion.PollMillisecond();
    }
  };
  pause_to(5.0);
  BOOST_TEST(!Quat48IsSentinel(sim.Request(sim.t_sample + 5.0 * 250.0)));
  for (const double pause_ms : {262.144, 524.288}) {
    pause_to(pause_ms);
    BOOST_TEST(Quat48IsSentinel(sim.Request(sim.t_sample + pause_ms * 250.0)));
    BOOST_TEST(std::isnan(sim.fusion.RateAt(
        static_cast<uint16_t>(static_cast<uint64_t>(
            std::llround(sim.t_sample + pause_ms * 250.0)) & 0xffff), 1)));
  }

  // Acquisition resumes: valid again.
  sim.Run(0.05);
  for (int i = 0; i < 4; i++) { sim.Request(sim.t_sample); }
  BOOST_TEST(sim.ReplyErrorDeg(sim.t_sample) < 0.05);
}

BOOST_AUTO_TEST_CASE(FusionSteadyYawIsNotLearnedAsBias) {
  // A 6-axis IMU cannot tell a steady yaw from a bias: a robot turning at
  // a constant 2 dps for a minute must keep reading 2 dps (the magnitude
  // gate), not learn it away (review 2026-10-07: a steadiness gate did).
  Sim sim;
  sim.gyro_bias[2] = 0.005f;
  sim.Run(60.0);                                  // the slow learner has it
  // (the heading drifted ~3 deg meanwhile -- bias x tau, the price of
  // learning from zero; the robot boots from the persisted bias instead)
  float truth0[4];
  sim.TruthAt(sim.t_sample, truth0);
  const float yaw0 = fm::Yaw(sim.fusion.q()) - fm::Yaw(truth0);
  sim.omega[2] = 2.0f * static_cast<float>(M_PI) / 180.0f;
  sim.Run(60.0);
  BOOST_TEST(std::abs(sim.fusion.bias()[2] - 0.005f) < 0.001f);
  BOOST_TEST(std::abs(sim.fusion.omega()[2] - sim.omega[2]) < 0.001f);
  BOOST_TEST(!sim.fusion.stationary());
  float truth[4];
  sim.TruthAt(sim.t_sample, truth);
  const float yaw1 = fm::Yaw(sim.fusion.q()) - fm::Yaw(truth);
  BOOST_TEST(std::abs(std::remainder(yaw1 - yaw0, 2.0f * static_cast<float>(M_PI))) < 1.0f * M_PI / 180.0f);
}

BOOST_AUTO_TEST_CASE(FusionRelearnClearsAnAbsorbedRotation) {
  // A hanging robot's rope twist ramps the yaw rate slowly enough for the
  // learner to track it as bias; when a hand stops it the corrected rate
  // jumps past the gate and the learner is locked out for good (8 of 48
  // robot boards, 2026-10-07).  Only a host-requested relearn, issued
  // when the robot is known still, clears it.
  Sim sim;
  sim.gyro_bias[2] = 0.005f;
  sim.Run(5.0);
  const int n = 60 * 960;
  for (int k = 0; k < n; k++) {
    sim.omega[2] = 2.0f * static_cast<float>(M_PI) / 180.0f * static_cast<float>(k) / n;
    sim.Step();
  }
  sim.Run(10.0);
  BOOST_TEST(std::abs(sim.fusion.bias()[2] - 0.005f) > 0.02f);   // the twist was absorbed
  sim.omega[2] = 0.0f;
  sim.Run(60.0);
  BOOST_TEST(std::abs(sim.fusion.bias()[2] - 0.005f) > 0.02f);   // and stays: locked out
  BOOST_TEST(!sim.fusion.stationary());
  sim.fusion.Relearn(3000);
  sim.Run(5.0);
  BOOST_TEST(std::abs(sim.fusion.bias()[2] - 0.005f) < 0.001f);
  BOOST_TEST(std::abs(sim.fusion.omega()[2]) < 0.001f);
  BOOST_TEST(sim.fusion.stationary());
}

BOOST_AUTO_TEST_CASE(FusionRelearnWorksPastTheLearnableRange) {
  // Opposite signs: a true -5 mrad/s bias and a learned +50 (the clamp)
  // leave a -55 mrad/s residual, beyond bias_max; the relearn gate is
  // steadiness only, so it still recovers (review 2026-10-07).
  Sim sim;
  sim.gyro_bias[2] = -0.005f;
  sim.Run(5.0);
  const int n = 60 * 960;
  for (int k = 0; k < n; k++) {
    sim.omega[2] = 5.0f * static_cast<float>(M_PI) / 180.0f * static_cast<float>(k) / n;
    sim.Step();
  }
  sim.omega[2] = 0.0f;
  sim.Run(60.0);
  BOOST_TEST(sim.fusion.bias()[2] > 0.02f);                      // locked out, wrong sign
  sim.fusion.Relearn(3000);
  sim.Run(5.0);
  BOOST_TEST(std::abs(sim.fusion.bias()[2] + 0.005f) < 0.001f);
  BOOST_TEST(std::abs(sim.fusion.omega()[2]) < 0.001f);
  BOOST_TEST(sim.fusion.stationary());
}

BOOST_AUTO_TEST_CASE(FusionRelearnIgnoresAMiscalibratedAccelerometer) {
  // A board whose accelerometer reads 2.5% off g never passes the
  // stationary gate (robot board can3/34, 2026-10-07): with the host
  // vouching for stillness the relearn still runs.
  Sim sim;
  sim.gyro_bias[2] = 0.008f;
  sim.accel_bias[2] = -0.025f;                   // along gravity: |a| = 0.975 g
  sim.Run(3.0);
  for (int i = 0; i < 3; i++) { sim.fusion.params()->accel_bias[i] = 0.0f; }
  sim.Run(30.0);
  BOOST_TEST(!sim.fusion.stationary());          // gated out
  sim.fusion.Relearn(3000);
  sim.Run(5.0);
  BOOST_TEST(std::abs(sim.fusion.bias()[2] - 0.008f) < 0.001f);
  BOOST_TEST(std::abs(sim.fusion.omega()[2]) < 0.001f);
}

BOOST_AUTO_TEST_CASE(FusionRelearnDoesNotOutliveAnOutage) {
  // The host's assurance of stillness is for the request's wall time: a
  // relearn in progress when acquisition stops is cancelled (stale words,
  // and the restart's re-initialization), so a turn that begins after the
  // restart is not learned away (review 2026-10-07).
  Sim sim;
  sim.gyro_bias[2] = 0.005f;
  sim.Run(60.0);
  sim.fusion.Relearn(3000);
  sim.Run(0.5);                                   // qualifying, nothing learnt yet
  sim.Outage(5.0);
  sim.omega[2] = 2.0f * static_cast<float>(M_PI) / 180.0f;
  sim.Run(20.0);                                  // re-init, converge, turn
  BOOST_TEST(std::abs(sim.fusion.bias()[2] - 0.005f) < 0.001f);
  BOOST_TEST(std::abs(sim.fusion.omega()[2] - sim.omega[2]) < 0.001f);
}

BOOST_AUTO_TEST_CASE(FusionBootWhileTwistingIsNotAbsorbed) {
  // The joint power switch is on the robot: the boards boot while the
  // hanging body is still twisting from the touch.  Qualified at rest for
  // a moment, then a yaw rate that ramps to 1.5 dps over 1.5 s, holds,
  // and ramps back.  The old startup capture (0.1 s) tracked that ramp as
  // bias and left the stopped robot reading -1.5 dps for good; the slow
  // learner barely moves and the gate shuts it off.  The board starts
  // from its persisted bias, as on the robot.
  Sim sim;
  sim.gyro_bias[2] = 0.005f;
  sim.fusion.params()->gyro_bias0[2] = 0.005f;
  sim.Run(1.5);
  const float peak = 1.5f * static_cast<float>(M_PI) / 180.0f;
  const int ramp = static_cast<int>(1.5 * 960);
  for (int k = 0; k < ramp; k++) {
    sim.omega[2] = peak * static_cast<float>(k) / ramp;
    sim.Step();
  }
  sim.omega[2] = peak;
  sim.Run(1.0);
  for (int k = ramp; k > 0; k--) {
    sim.omega[2] = peak * static_cast<float>(k) / ramp;
    sim.Step();
  }
  sim.omega[2] = 0.0f;
  BOOST_TEST(std::abs(sim.fusion.bias()[2] - 0.005f) < 0.002f);   // < 0.12 dps taken in
  sim.Run(30.0);
  BOOST_TEST(std::abs(sim.fusion.bias()[2] - 0.005f) < 0.0005f);
  BOOST_TEST(std::abs(sim.fusion.omega()[2]) < 0.0005f);
  BOOST_TEST(sim.fusion.stationary());
}

BOOST_AUTO_TEST_CASE(FusionPersistedBiasSeedsTheLearner) {
  // The host saves each board's learned bias (imu_cal.gyro_bias); a cold
  // start begins from it, so the corrected rate and the tilt are right
  // from the first word instead of after a 1.7 deg, 15 s transient.
  Sim sim;
  sim.gyro_bias[0] = 0.5f * M_PI / 180.0f;
  sim.gyro_bias[1] = -0.4f * M_PI / 180.0f;
  sim.gyro_bias[2] = 0.3f * M_PI / 180.0f;
  for (int i = 0; i < 3; i++) { sim.fusion.params()->gyro_bias0[i] = sim.gyro_bias[i]; }
  sim.SetTilt(1.0f, 0.0f, 0.0f, 10.0f * M_PI / 180.0f);
  sim.Run(0.3);
  for (int i = 0; i < 3; i++) {
    BOOST_TEST(std::abs(sim.fusion.bias()[i] - sim.gyro_bias[i]) < 1.0e-6f);
    BOOST_TEST(std::abs(sim.fusion.omega()[i]) < 0.0005f);
  }
  sim.Run(2.0);
  BOOST_TEST(AngleDeg(sim.fusion.q(), sim.q_true) < 0.1);
  // Seeded once: a restart after an outage keeps the bias learned since
  // (here: the same value) even when the parameter has changed.
  sim.fusion.params()->gyro_bias0[2] = 0.0f;
  sim.Outage(2.0);
  sim.Run(3.0);
  BOOST_TEST(std::abs(sim.fusion.bias()[2] - sim.gyro_bias[2]) < 0.0005f);
  // Reset() (a new attach) does seed again.
  sim.fusion.Reset();
  sim.Run(0.3);
  BOOST_TEST(std::abs(sim.fusion.bias()[2]) < 1.0e-6f);
  BOOST_TEST(std::abs(sim.fusion.bias()[0] - sim.gyro_bias[0]) < 1.0e-6f);
}

BOOST_AUTO_TEST_CASE(FusionRelearnDurationIsLearningTime) {
  // `relearn 0.5` means half a second of learning after the one-second
  // qualification (the first version accepted it and learnt nothing).
  Sim sim;
  sim.gyro_bias[2] = 0.005f;
  sim.Run(60.0);
  sim.gyro_bias[2] = 0.035f;                     // a 30 mrad/s step: past the gate, locked out
  sim.Run(10.0);
  BOOST_TEST(std::abs(sim.fusion.bias()[2] - 0.005f) < 0.001f);
  BOOST_TEST(!sim.fusion.stationary());
  sim.fusion.Relearn(500);
  sim.Run(2.0);
  BOOST_TEST(std::abs(sim.fusion.bias()[2] - 0.035f) < 0.001f);
  BOOST_TEST(std::abs(sim.fusion.omega()[2]) < 0.001f);
}

BOOST_AUTO_TEST_CASE(FusionRelearnRightAfterMotion) {
  // A relearn requested the moment the robot stops (a hand sets it down,
  // then the console command) must still get its learning time: the
  // steadiness average's memory of the rotation decays over ~1.2 s,
  // which used to count as unsteady and outlast a short deadline.
  Sim sim;
  sim.gyro_bias[2] = 0.005f;
  sim.Run(60.0);
  sim.gyro_bias[2] = 0.035f;                     // past the gate, locked out
  sim.Run(10.0);
  sim.omega[2] = 2.0f;                           // a 2 rad/s turn
  sim.Run(2.0);
  sim.omega[2] = 0.0f;                           // stops, and at once:
  sim.fusion.Relearn(500);
  sim.Run(2.0);
  BOOST_TEST(std::abs(sim.fusion.bias()[2] - 0.035f) < 0.001f);
  BOOST_TEST(std::abs(sim.fusion.omega()[2]) < 0.001f);
}

BOOST_AUTO_TEST_CASE(FusionLatencyCompensation) {
  // The production default evaluates the reply latency_comp_ticks after
  // the request stamp (gyro extrapolation): for a constant rotation that
  // is the orientation at request + comp, exactly.
  const ImuFusion::Params defaults;
  BOOST_TEST(defaults.latency_comp_ticks == 500);   // 2.0 ms, the 2026-10-07 setting
  Sim sim;
  sim.fusion.params()->latency_comp_ticks = defaults.latency_comp_ticks;
  sim.omega[1] = 2.0f;
  sim.Run(3.0);
  for (int i = 0; i < 4; i++) { sim.Request(sim.t_sample); }
  const auto words = sim.Request(sim.t_sample);
  BOOST_TEST_REQUIRE(!Quat48IsSentinel(words));
  float q[4];
  DecodeQuat48(words, q);
  float at_request[4], at_comp[4];
  sim.TruthAt(sim.t_sample, at_request);
  sim.TruthAt(sim.t_sample + defaults.latency_comp_ticks, at_comp);
  BOOST_TEST(AngleDeg(q, at_comp) < 0.02);
  // 2 rad/s x 2 ms = 0.23 deg ahead of the uncompensated truth
  BOOST_TEST(AngleDeg(q, at_request) > 0.2);
}

#ifdef MOTEUS_TS_PROBE
BOOST_AUTO_TEST_CASE(FusionChipTimestampProbe) {
  // The transfer-delay probe recovers the FIFO write -> arrival delay of
  // the gyro word after a timestamp word from that word and a live
  // counter read (two clocks, one tie point), to the counter's 21.75 us
  // resolution.
  for (const bool ts_first : {false, true}) {
  for (const double dmin : {100.0, 250.0}) {
    Sim sim;
    sim.ts_first = ts_first;
    sim.dmin = dmin;
    sim.Run(3.0);   // 2880 samples: the last one carries a timestamp word
    sim.Step();
    sim.Step();     // the measured word: the gyro two slots on
    const auto& s = sim.status;
    BOOST_TEST(s.ts_pairs == 0);   // no live read yet
    sim.Probe(sim.t_sample + dmin + 60.0);
    BOOST_TEST(s.ts_pairs == 1);
    BOOST_TEST(s.ts_rejects == 0);
    BOOST_TEST(std::abs(s.ts_delay_mean_us - dmin * kFusionTickUs) < 40.0);
    BOOST_TEST(std::abs(s.ts_delay_mean_us - s.ts_delay_min_us) < 1.0);
    // Constant transfer, no jitter: the arrival floor IS the arrival, so
    // the fusion clock's offset from the chip equals the transfer.
    BOOST_TEST(std::abs(s.ts_model_mean_us - dmin * kFusionTickUs) < 40.0);

    // A probe seen before its words are processed waits for them.
    sim.drain_each = false;
    sim.Run(32.0 / kFusionGyroOdrHz);   // through the next timestamp word and its measured slot, undrained
    sim.Probe(sim.t_sample + dmin + 60.0);   // Drain: probe first, then the words
    BOOST_TEST(s.ts_pairs == 2);
    BOOST_TEST(s.ts_rejects == 0);
    BOOST_TEST(std::abs(s.ts_delay_mean_us - dmin * kFusionTickUs) < 40.0);
    BOOST_TEST(std::abs(s.ts_delay_mean_us - s.ts_delay_min_us) < 40.0);

    // A live read corrupted by a byte rollover (256 LSB high) is rejected,
    // not reported as a 5.6 ms transfer.
    sim.drain_each = true;
    sim.Run(32.0 / kFusionGyroOdrHz);   // the next timestamp word and its measured slot
    sim.Probe(sim.t_sample + dmin + 60.0, 256);
    BOOST_TEST(s.ts_pairs == 2);
    BOOST_TEST(s.ts_rejects == 1);
    BOOST_TEST(std::abs(s.ts_delay_mean_us - s.ts_delay_min_us) < 40.0);
  }
  }
}

BOOST_AUTO_TEST_CASE(FusionChipTimestampProbeSurvivesAStall) {
  // The main loop away for two probe cycles (a flash write, a long
  // console command): the words of both cycles are queued behind the
  // latest live read.  The first cycle's pair is stale and rejected,
  // the read must survive it for its own pair, and the cycles after
  // that pair as usual (the first version spent the read on the stale
  // pair and then rejected every cycle, one read ahead, for good).
  Sim sim;
  sim.dmin = 100.0;
  sim.Run(3.0);
  sim.Step();
  sim.Step();
  sim.Probe(sim.t_sample + sim.dmin + 60.0);
  const auto& s = sim.status;
  BOOST_TEST(s.ts_pairs == 1);
  BOOST_TEST(s.ts_rejects == 0);

  sim.drain_each = false;
  // Two more timestamp words, undrained; the last step is the second
  // one's measured slot.
  for (int i = 0; i < 64; i++) { sim.Step(); }
  sim.Probe(sim.t_sample + sim.dmin + 60.0);   // the latest read, then the backlog
  BOOST_TEST(s.ts_pairs == 2);
  BOOST_TEST(s.ts_rejects == 1);

  // The cycles after it, in the real order (the live read lands before
  // its pair's word is drained), pair as usual.
  for (int cycle = 0; cycle < 3; cycle++) {
    for (int i = 0; i < 32; i++) { sim.Step(); }
    sim.Probe(sim.t_sample + sim.dmin + 60.0);
  }
  BOOST_TEST(s.ts_pairs == 5);
  BOOST_TEST(s.ts_rejects == 1);
}
#endif  // MOTEUS_TS_PROBE
