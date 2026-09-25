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
    if (k % 32 == 0) {
      FusionWord ts;
      ts.t = g.t;
      ts.seq = seq;
      ts.tag = kFusionTagTimestamp;
      storage->mailbox.Push(ts);
    }
    if (drain_each) { fusion.Drain(16); }
  }

  void Run(double seconds) {
    const int n = static_cast<int>(seconds * kFusionGyroOdrHz);
    for (int i = 0; i < n; i++) { Step(); }
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

  // At rest the stationary learner takes the bias out of the rate.
  sim.Run(5.0);
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
  // Startup capture: within 4 s at rest both biases are in, and the
  // tilt transient the x bias caused is decaying (kp = 0.3: ~3 s).
  sim.Run(3.2);
  BOOST_TEST(std::abs(sim.fusion.bias()[2] - sim.gyro_bias[2]) < 0.2f * sim.gyro_bias[2]);
  BOOST_TEST(std::abs(sim.fusion.bias()[0] - sim.gyro_bias[0]) < 0.2f * sim.gyro_bias[0]);
  // Heading is unobservable and holds whatever it acquired before the
  // capture (~0.3 deg here), so judge tilt only.
  {
    float up_est[3]; float up_true[3];
    fm::BodyUp(sim.fusion.q(), up_est); fm::BodyUp(sim.q_true, up_true);
    BOOST_TEST(VecAngleDeg(up_est, up_true) < 0.5);
  }
  sim.Run(8.0);
  {
    float up_est[3]; float up_true[3];
    fm::BodyUp(sim.fusion.q(), up_est); fm::BodyUp(sim.q_true, up_true);
    BOOST_TEST(VecAngleDeg(up_est, up_true) < 0.1);
  }
  sim.Run(32.0);
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
  sim.Run(0.2);  // initialized, still on the fast startup gain
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
