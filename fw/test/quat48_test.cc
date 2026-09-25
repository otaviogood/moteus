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

#include "fw/quat48.h"

#include <cmath>
#include <ostream>
#include <random>

#include <boost/test/auto_unit_test.hpp>

using namespace moteus;

namespace moteus {
std::ostream& operator<<(std::ostream& os, const Quat48Words& w) {
  return os << std::hex << "0x" << w.w0 << " 0x" << w.w1 << " 0x" << w.w2
            << std::dec;
}
}

namespace {

// Rotation angle (deg) between two unit quaternions, sign-insensitive.
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

// The rigorous bound from docs/imu_orientation_quantization.py.
constexpr double kBoundDeg = 0.0085675;

struct Vector {
  float q[4];
  Quat48Words words;
};

// docs/imu_orientation_redesign.md §4.4, bit 47 = 0.
const Vector kVectors[] = {
  {{1.0f, 0.0f, 0.0f, 0.0f}, {0x4000, 0x2000, 0x1000}},
  {{0.70710678f, 0.70710678f, 0.0f, 0.0f}, {0x7fff, 0x2000, 0x1000}},
  {{0.5f, 0.5f, 0.5f, 0.5f}, {0x6d40, 0x36a0, 0x1b50}},
  {{0.0f, 0.6f, 0.0f, 0.8f}, {0xc000, 0x3b26, 0x7000}},
  {{-0.8f, 0.36f, -0.48f, 0.0f}, {0x9f6a, 0x35b8, 0x1000}},
  {{0.2063f, -0.5157f, 0.7220f, -0.4126f}, {0xd2ab, 0x08a9, 0x46aa}},
};

}  // namespace

BOOST_AUTO_TEST_CASE(Quat48TestVectors) {
  for (const auto& v : kVectors) {
    BOOST_TEST_CONTEXT("q=" << v.q[0] << "," << v.q[1] << ","
                       << v.q[2] << "," << v.q[3]) {
      const auto words = EncodeQuat48(v.q[0], v.q[1], v.q[2], v.q[3], false);
      BOOST_TEST(words.w0 == v.words.w0);
      BOOST_TEST(words.w1 == v.words.w1);
      BOOST_TEST(words.w2 == v.words.w2);
      BOOST_TEST(!Quat48IsSentinel(words));
      BOOST_TEST(!Quat48Toggle(words));

      float q[4] = {};
      BOOST_TEST(DecodeQuat48(words, q));
      BOOST_TEST(AngleDeg(q, v.q) < kBoundDeg);
    }
  }
}

BOOST_AUTO_TEST_CASE(Quat48ToggleBit) {
  const auto& v = kVectors[5];
  const auto w0 = EncodeQuat48(v.q[0], v.q[1], v.q[2], v.q[3], false);
  const auto w1 = EncodeQuat48(v.q[0], v.q[1], v.q[2], v.q[3], true);
  BOOST_TEST(w1.w0 == w0.w0);
  BOOST_TEST(w1.w1 == w0.w1);
  BOOST_TEST(w1.w2 == (w0.w2 | 0x8000));
  BOOST_TEST(Quat48Toggle(w1));
  BOOST_TEST(!Quat48Toggle(w0));

  float qa[4] = {};
  float qb[4] = {};
  BOOST_TEST(DecodeQuat48(w0, qa));
  BOOST_TEST(DecodeQuat48(w1, qb));
  for (int i = 0; i < 4; i++) { BOOST_TEST(qa[i] == qb[i]); }
}

BOOST_AUTO_TEST_CASE(Quat48Sentinel) {
  const Quat48Words zero;
  BOOST_TEST(Quat48IsSentinel(zero));
  // Bit 47 alone is still the sentinel.
  const Quat48Words toggled{0, 0, 0x8000};
  BOOST_TEST(Quat48IsSentinel(toggled));
  float q[4] = {1, 2, 3, 4};
  BOOST_TEST(!DecodeQuat48(toggled, q));
  BOOST_TEST(q[0] == 1.0f);

  // Non-finite and zero inputs encode as the sentinel.
  BOOST_TEST(Quat48IsSentinel(EncodeQuat48(0, 0, 0, 0, false)));
  BOOST_TEST(Quat48IsSentinel(EncodeQuat48(NAN, 0, 0, 0, false)));
}

BOOST_AUTO_TEST_CASE(Quat48RoundTripRandom) {
  std::mt19937 rng(12345);
  std::normal_distribution<double> nd(0.0, 1.0);
  double max_err = 0.0;
  for (int n = 0; n < 200000; n++) {
    double q[4];
    double norm = 0.0;
    for (auto& v : q) { v = nd(rng); norm += v * v; }
    norm = std::sqrt(norm);
    float qf[4];
    for (int i = 0; i < 4; i++) { qf[i] = static_cast<float>(q[i] / norm); }

    const auto words = EncodeQuat48(qf[0], qf[1], qf[2], qf[3], n & 1);
    BOOST_TEST(!Quat48IsSentinel(words));
    BOOST_TEST(Quat48Toggle(words) == static_cast<bool>(n & 1));
    float out[4] = {};
    BOOST_TEST(DecodeQuat48(words, out));
    const double err = AngleDeg(out, qf);
    if (err > max_err) { max_err = err; }
  }
  // float32 arithmetic adds a little to the ideal-arithmetic bound;
  // allow 20% on top of it.
  BOOST_TEST(max_err < kBoundDeg * 1.2);
}

BOOST_AUTO_TEST_CASE(Quat48SignInvariance) {
  // q and -q must encode to identical words.
  const auto& v = kVectors[5];
  const auto a = EncodeQuat48(v.q[0], v.q[1], v.q[2], v.q[3], false);
  const auto b = EncodeQuat48(-v.q[0], -v.q[1], -v.q[2], -v.q[3], false);
  BOOST_TEST(a == b);
}
