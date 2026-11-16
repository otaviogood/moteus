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

#include "fw/gyro_audio.h"

#include <cmath>
#include <cstdlib>
#include <vector>

#include <boost/test/auto_unit_test.hpp>

using namespace moteus;

namespace {
void Push(GyroAudio* audio, int x, int y, int z) {
  const int16_t w[3] = {static_cast<int16_t>(x), static_cast<int16_t>(y),
                        static_cast<int16_t>(z)};
  audio->Push(w);
}

struct Sample {
  int8_t v[3];
};

struct Reply {
  size_t size = 0;
  uint8_t format = 0;
  uint16_t first = 0;
  std::vector<Sample> samples;
};

Reply Take(GyroAudio* audio, size_t room = 64) {
  uint8_t out[64] = {};
  Reply r;
  r.size = audio->WriteReply(out, room);
  if (r.size < kGyroAudioHeaderSize) { return r; }
  r.format = out[0];
  r.first = static_cast<uint16_t>(out[2] | (out[3] << 8));
  for (int i = 0; i < out[1]; i++) {
    Sample s;
    for (int a = 0; a < 3; a++) {
      s.v[a] = static_cast<int8_t>(out[4 + 3 * i + a]);
    }
    r.samples.push_back(s);
  }
  BOOST_TEST(r.size == kGyroAudioHeaderSize + 3u * out[1]);
  return r;
}
}

BOOST_AUTO_TEST_CASE(GyroAudioCodeIsNearestStep) {
  // Every value a code stands for, ascending.
  std::vector<int32_t> steps;
  for (int c = 0; c <= 127; c++) {
    steps.push_back(GyroAudioDecode(static_cast<int8_t>(c)));
    if (c > 0) { BOOST_TEST(steps[c] > steps[c - 1]); }
    BOOST_TEST(GyroAudioDecode(static_cast<int8_t>(-c)) == -steps[c]);
    BOOST_TEST(GyroAudioEncode(steps[c]) == c);
    BOOST_TEST(GyroAudioEncode(-steps[c]) == -c);
  }
  BOOST_TEST(steps[31] == 31);
  BOOST_TEST(steps[32] == 32);
  BOOST_TEST(steps[33] == 34);
  BOOST_TEST(steps[127] == kGyroAudioMaxStep);
  BOOST_TEST(GyroAudioDecode(-128) == -kGyroAudioMaxStep);

  // Every input: exact up to 31, the nearest step up to the limit (a
  // tie goes to the larger one), the limit beyond.
  for (int32_t d = -140000; d <= 140000; d++) {
    const int32_t got = GyroAudioDecode(GyroAudioEncode(d));
    const int32_t m = std::abs(d);
    int32_t best = steps[0];
    for (int32_t s : steps) {
      if (std::abs(s - m) <= std::abs(best - m)) { best = s; }
    }
    BOOST_REQUIRE(got == (d < 0 ? -best : best));
    if (m <= 31) { BOOST_REQUIRE(got == d); }
    if (m >= 32 && m <= kGyroAudioMaxStep) {
      // Within 1/32 of the value.
      BOOST_REQUIRE(std::abs(got - d) * 32 <= m);
    }
  }
}

// The same vectors as humanoid3 orin/tests/test_gyro_audio.py.
BOOST_AUTO_TEST_CASE(GyroAudioCodeVectors) {
  BOOST_TEST(GyroAudioEncode(0) == 0);
  BOOST_TEST(GyroAudioEncode(1) == 1);
  BOOST_TEST(GyroAudioEncode(-1) == -1);
  BOOST_TEST(GyroAudioEncode(31) == 31);
  BOOST_TEST(GyroAudioEncode(32) == 32);
  BOOST_TEST(GyroAudioEncode(33) == 33);
  BOOST_TEST(GyroAudioEncode(63) == 48);
  BOOST_TEST(GyroAudioEncode(100) == 57);
  BOOST_TEST(GyroAudioEncode(-250) == -79);
  BOOST_TEST(GyroAudioEncode(1000) == 111);
  BOOST_TEST(GyroAudioEncode(1983) == 127);
  BOOST_TEST(GyroAudioEncode(-5000) == -127);
  BOOST_TEST(GyroAudioEncode(131070) == 127);
  BOOST_TEST(GyroAudioDecode(33) == 34);
  BOOST_TEST(GyroAudioDecode(48) == 64);
  BOOST_TEST(GyroAudioDecode(57) == 100);
  BOOST_TEST(GyroAudioDecode(-79) == -248);
  BOOST_TEST(GyroAudioDecode(111) == 992);
}

BOOST_AUTO_TEST_CASE(GyroAudioSmallChangesAreExact) {
  GyroAudio audio;
  // The first sample has nothing before it; after that each code is the
  // change itself while it stays within +-31.
  const int x[] = {1000, 1003, 1001, 1001, 970, 1001};
  const int y[] = {-20000, -20000, -20031, -20030, -20029, -20000};
  const int z[] = {5, -5, 6, -7, 8, -9};
  for (int k = 0; k < 6; k++) { Push(&audio, x[k], y[k], z[k]); }
  auto r = Take(&audio);
  BOOST_TEST(r.format == kGyroAudioFormat);
  BOOST_TEST(r.first == 0);
  BOOST_TEST(r.samples.size() == 6u);
  for (int a = 0; a < 3; a++) { BOOST_TEST(r.samples[0].v[a] == 0); }
  for (int k = 1; k < 6; k++) {
    BOOST_TEST(r.samples[k].v[0] == x[k] - x[k - 1]);
    BOOST_TEST(r.samples[k].v[1] == y[k] - y[k - 1]);
    BOOST_TEST(r.samples[k].v[2] == z[k] - z[k - 1]);
  }

  // Nothing new: an empty reply that still names the next sample.
  r = Take(&audio);
  BOOST_TEST(r.size == kGyroAudioHeaderSize);
  BOOST_TEST(r.first == 6);
  BOOST_TEST(r.samples.empty());
}

BOOST_AUTO_TEST_CASE(GyroAudioRunningSumFollowsTheGyro) {
  GyroAudio audio;
  // A loud two-tone signal plus a ramp: the running sum of the decoded
  // codes stays within half a step of the input, and does not drift.
  const int kCount = 5000;
  double worst = 0.0;
  int32_t sum[3] = {};
  int32_t start[3] = {};
  for (int k = 0; k < kCount; k++) {
    const int w[3] = {
      static_cast<int>(2000.0 * std::sin(k * 0.9) + 2.0 * k),
      static_cast<int>(800.0 * std::sin(k * 0.05) - 300.0 * std::sin(k * 2.1)),
      static_cast<int>(20.0 * std::sin(k * 1.7)),
    };
    Push(&audio, w[0], w[1], w[2]);
    const auto r = Take(&audio);
    BOOST_REQUIRE(r.samples.size() == 1u);
    for (int a = 0; a < 3; a++) {
      if (k == 0) { start[a] = w[a]; }
      const int32_t step = GyroAudioDecode(r.samples[0].v[a]);
      sum[a] += step;
      const double error = std::abs((start[a] + sum[a]) - w[a]);
      // Half a step of the code that was sent.
      const double half_step = std::max(0.5, std::abs(step) / 32.0);
      BOOST_REQUIRE(error <= half_step);
      worst = std::max(worst, error);
    }
  }
  BOOST_TEST(worst > 0.0);   // the signal was loud enough to need rounding
}

BOOST_AUTO_TEST_CASE(GyroAudioFullScaleCatchesUp) {
  GyroAudio audio;
  // A full-scale jump is more than one code holds: the codes stay at
  // the limit until the running sum has caught up, then it is exact.
  Push(&audio, -32768, 0, 32767);
  for (int k = 0; k < 40; k++) { Push(&audio, 32767, 0, -32768); }
  int32_t sum[3] = {-32768, 0, 32767};
  int limited = 0;
  for (int taken = 0; taken < 41;) {
    const auto r = Take(&audio);
    BOOST_REQUIRE(!r.samples.empty());
    for (const auto& s : r.samples) {
      if (taken > 0 && s.v[0] == 127) { limited++; }
      for (int a = 0; a < 3; a++) { sum[a] += GyroAudioDecode(s.v[a]); }
      taken++;
    }
  }
  BOOST_TEST(limited == 33);   // 65535 / 1984
  BOOST_TEST(sum[0] == 32767);
  BOOST_TEST(sum[1] == 0);
  BOOST_TEST(sum[2] == -32768);
}

BOOST_AUTO_TEST_CASE(GyroAudioLostSamples) {
  GyroAudio audio;
  for (int k = 0; k < 5; k++) { Push(&audio, 10 * k, 0, 0); }
  Take(&audio);

  // Three samples lost: they count, read 0, and the sample after the
  // gap reads 0 too (the running sum starts over there).
  audio.Skip(3);
  Push(&audio, 5000, 0, 0);
  Push(&audio, 5010, -3, 0);
  auto r = Take(&audio);
  BOOST_TEST(r.first == 5);
  BOOST_TEST(r.samples.size() == 5u);
  for (int i = 0; i < 4; i++) {
    for (int a = 0; a < 3; a++) { BOOST_TEST(r.samples[i].v[a] == 0); }
  }
  BOOST_TEST(r.samples[4].v[0] == 10);
  BOOST_TEST(r.samples[4].v[1] == -3);

  // A restart of the sample stream: no count is known, the next sample
  // reads 0.
  audio.ResetHistory();
  Push(&audio, 1, 2, 3);
  Push(&audio, 2, 2, 2);
  r = Take(&audio);
  BOOST_TEST(r.first == 10);
  BOOST_TEST(r.samples.size() == 2u);
  BOOST_TEST(r.samples[0].v[0] == 0);
  BOOST_TEST(r.samples[1].v[0] == 1);
  BOOST_TEST(r.samples[1].v[2] == -1);

  // A gap longer than the ring.
  audio.Skip(1000);
  r = Take(&audio);
  BOOST_TEST(r.first == static_cast<uint16_t>(12 + 1000 - GyroAudio::kSize));
  BOOST_TEST(r.samples.size() == 20u);
  for (const auto& s : r.samples) { BOOST_TEST(s.v[0] == 0); }
}

BOOST_AUTO_TEST_CASE(GyroAudioReplyRoomAndBacklog) {
  GyroAudio audio;
  // x steps by a different small amount every sample, so the order is
  // checkable: sample k reads (k % 29) - 14 on x and -that on z.
  auto step = [](int k) { return (k % 29) - 14; };
  int x = 0;
  int k = 0;
  auto push = [&]() {
    if (k > 0) { x += step(k); }
    Push(&audio, x, 7, -x);
    k++;
  };
  auto check = [&](const Sample& s, int index) {
    BOOST_TEST(s.v[0] == (index == 0 ? 0 : step(index)));
    BOOST_TEST(s.v[1] == 0);
    BOOST_TEST(s.v[2] == (index == 0 ? 0 : -step(index)));
  };
  while (k < 30) { push(); }

  // A 32-byte frame holds 9 samples; the rest stay for the next reply.
  auto r = Take(&audio, 32);
  BOOST_TEST(r.size == 31u);
  BOOST_TEST(r.first == 0);
  BOOST_TEST(r.samples.size() == 9u);
  for (int i = 0; i < 9; i++) { check(r.samples[i], i); }
  // A full frame holds 20.
  r = Take(&audio);
  BOOST_TEST(r.size == 64u);
  BOOST_TEST(r.first == 9);
  BOOST_TEST(r.samples.size() == 20u);
  for (int i = 0; i < 20; i++) { check(r.samples[i], 9 + i); }
  r = Take(&audio);
  BOOST_TEST(r.first == 29);
  BOOST_TEST(r.samples.size() == 1u);

  // More than the ring holds: the oldest are dropped, and the sample
  // number says so.
  while (k < 30 + 200) { push(); }
  r = Take(&audio);
  BOOST_TEST(r.first == 230 - GyroAudio::kSize);
  BOOST_TEST(r.samples.size() == 20u);
  for (int i = 0; i < 20; i++) {
    check(r.samples[i], 230 - GyroAudio::kSize + i);
  }
  r = Take(&audio);
  BOOST_TEST(r.first == 230 - GyroAudio::kSize + 20);
  BOOST_TEST(r.samples.size() == 20u);

  // Too small for a header, and too small for one sample.
  BOOST_TEST(audio.WriteReply(nullptr, 3) == 0u);
  uint8_t out[8] = {};
  BOOST_TEST(audio.WriteReply(out, 6) == kGyroAudioHeaderSize);
  BOOST_TEST(out[1] == 0);
}

BOOST_AUTO_TEST_CASE(GyroAudioSampleNumberWraps) {
  GyroAudio audio;
  auto sample = [](int k) { return (k * 37) % 25; };
  int k = 0;
  // Polled every 8 samples across the 16-bit wrap.
  uint16_t next = 0;
  for (int poll = 0; poll < 8200; poll++) {
    for (int i = 0; i < 8; i++, k++) { Push(&audio, sample(k), 0, 0); }
    const auto r = Take(&audio);
    BOOST_REQUIRE(r.first == next);
    BOOST_REQUIRE(r.samples.size() == 8u);
    next = static_cast<uint16_t>(next + 8);
  }
  BOOST_TEST(k > 65536);
  // Not polled for exactly a whole wrap, then for a little more: the
  // whole ring every time, oldest first, with the right codes.
  for (int extra : {0, 10}) {
    for (int i = 0; i < 65536 + extra; i++, k++) {
      Push(&audio, sample(k), 0, 0);
    }
    int first = k - GyroAudio::kSize;
    while (first < k) {
      const auto r = Take(&audio);
      BOOST_REQUIRE(r.first == static_cast<uint16_t>(first));
      BOOST_REQUIRE(!r.samples.empty());
      for (size_t i = 0; i < r.samples.size(); i++) {
        const int j = first + static_cast<int>(i);
        BOOST_TEST(r.samples[i].v[0] == sample(j) - sample(j - 1));
      }
      first += static_cast<int>(r.samples.size());
    }
    BOOST_TEST(first == k);
    BOOST_TEST(Take(&audio).samples.size() == 0u);
  }
}

BOOST_AUTO_TEST_CASE(GyroAudioRestartDropsUnsent) {
  GyroAudio audio;
  Push(&audio, 0, 0, 0);
  Push(&audio, 100, 0, 0);
  Push(&audio, 200, 0, 0);
  Take(&audio);
  Push(&audio, 300, 0, 0);
  Push(&audio, 400, 0, 0);

  // A new sensor: the two unsent samples of the old one are gone, the
  // numbering carries on, and the first new sample reads 0.
  audio.Restart();
  BOOST_TEST(Take(&audio).samples.size() == 0u);
  Push(&audio, -5000, 0, 0);
  Push(&audio, -4990, 0, 0);
  const auto r = Take(&audio);
  BOOST_TEST(r.first == 5);
  BOOST_REQUIRE(r.samples.size() == 2u);
  BOOST_TEST(r.samples[0].v[0] == 0);
  BOOST_TEST(r.samples[1].v[0] == 10);
}

BOOST_AUTO_TEST_CASE(GyroAudioRequestStrip) {
  uint8_t request[] = {kGyroAudioRequest, kGyroAudioFormat, 0x11, 0x00};
  size_t size = sizeof(request);
  BOOST_TEST(StripGyroAudioRequest(request, &size));
  BOOST_TEST(size == 2u);
  BOOST_TEST(request[0] == 0x11);

  // The first format (one byte per sample) is no longer answered.
  uint8_t old_format[] = {kGyroAudioRequest, 0x01};
  size = sizeof(old_format);
  BOOST_TEST(!StripGyroAudioRequest(old_format, &size));
  BOOST_TEST(size == 2u);

  uint8_t other[] = {0x60, 0x02};
  size = sizeof(other);
  BOOST_TEST(!StripGyroAudioRequest(other, &size));
  uint8_t shorty[] = {kGyroAudioRequest};
  size = sizeof(shorty);
  BOOST_TEST(!StripGyroAudioRequest(shorty, &size));

  BOOST_TEST(GyroAudioReplyId(2) == 0x782u);
  BOOST_TEST(GyroAudioReplyId(0x7e) == 0x7feu);
}
