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

#include <cmath>
#include <cstdint>

/// "Smallest three" quaternion wire format used by the IMU fusion
/// registers 0x06d-0x06f.  See docs/imu_orientation_redesign.md §4 and
/// the Python reference in docs/imu_orientation_quantization.py.
///
/// 48-bit little-endian value across the three register words:
///   bits  0-14  15-bit unsigned code of the first remaining component
///   bits 15-29  second remaining component
///   bits 30-44  third remaining component
///   bits 45-46  index (0..3, w x y z) of the omitted component, which is
///               the largest in magnitude and made positive by negating
///               the whole quaternion when necessary
///   bit  47     freshness toggle (§4.2), not part of the payload
/// Low 47 bits all zero is the "no data" sentinel (§4.3).

namespace moteus {

struct Quat48Words {
  uint16_t w0 = 0;  // register 0x06d
  uint16_t w1 = 0;  // register 0x06e
  uint16_t w2 = 0;  // register 0x06f

  bool operator==(const Quat48Words& o) const {
    return w0 == o.w0 && w1 == o.w1 && w2 == o.w2;
  }
};

constexpr uint32_t kQuat48Levels = 32767;
constexpr int kQuat48IndexShift = 45;
constexpr int kQuat48ToggleBit = 47;
constexpr uint64_t kQuat48PayloadMask = (1ull << 47) - 1;

/// Encode a (w, x, y, z) quaternion of any non-zero norm.  Returns the
/// sentinel if the input is not finite or has zero norm.
#if defined(TARGET_STM32G4)
__attribute__((noinline, optimize("Os")))
#endif
inline Quat48Words EncodeQuat48(float w, float x, float y, float z,
                                bool toggle) {
  constexpr float kSqrt2 = 1.41421356f;
  float q[4] = {w, x, y, z};
  const float n2 = w * w + x * x + y * y + z * z;
  if (!(n2 > 0.0f) || !std::isfinite(n2)) {
    return Quat48Words{};
  }
  const float inv = 1.0f / std::sqrt(n2);
  for (auto& v : q) { v *= inv; }

  // Largest magnitude, ties -> lowest index (strict comparison).
  int i = 0;
  float best = std::abs(q[0]);
  for (int k = 1; k < 4; k++) {
    if (std::abs(q[k]) > best) {
      best = std::abs(q[k]);
      i = k;
    }
  }
  if (q[i] < 0.0f) {
    for (auto& v : q) { v = -v; }
  }

  uint64_t v = 0;
  int j = 0;
  for (int k = 0; k < 4; k++) {
    if (k == i) { continue; }
    float u = q[k] * kSqrt2;
    if (u > 1.0f) { u = 1.0f; }
    if (u < -1.0f) { u = -1.0f; }
    // code = round((u + 1) / 2 * 32767); the argument is >= 0, so
    // adding 0.5 and truncating is round-half-up.
    const float c = (u + 1.0f) * 0.5f * static_cast<float>(kQuat48Levels);
    uint32_t code = static_cast<uint32_t>(c + 0.5f);
    if (code > kQuat48Levels) { code = kQuat48Levels; }
    v |= static_cast<uint64_t>(code) << (15 * j);
    j++;
  }
  v |= static_cast<uint64_t>(i) << kQuat48IndexShift;
  if (toggle) { v |= 1ull << kQuat48ToggleBit; }

  Quat48Words result;
  result.w0 = static_cast<uint16_t>(v & 0xffff);
  result.w1 = static_cast<uint16_t>((v >> 16) & 0xffff);
  result.w2 = static_cast<uint16_t>((v >> 32) & 0xffff);
  return result;
}

inline bool Quat48IsSentinel(const Quat48Words& w) {
  const uint64_t v =
      (static_cast<uint64_t>(w.w0) |
       (static_cast<uint64_t>(w.w1) << 16) |
       (static_cast<uint64_t>(w.w2) << 32)) & kQuat48PayloadMask;
  return v == 0;
}

inline bool Quat48Toggle(const Quat48Words& w) {
  return (w.w2 & 0x8000) != 0;
}

/// Decode into q[4] = (w, x, y, z).  Returns false (and leaves q
/// untouched) for the sentinel.  Used by host tests and tools; the
/// firmware only encodes.
inline bool DecodeQuat48(const Quat48Words& w, float* q) {
  constexpr float kSqrt2 = 1.41421356f;
  uint64_t v =
      static_cast<uint64_t>(w.w0) |
      (static_cast<uint64_t>(w.w1) << 16) |
      (static_cast<uint64_t>(w.w2) << 32);
  v &= kQuat48PayloadMask;
  if (v == 0) { return false; }

  const int i = static_cast<int>((v >> kQuat48IndexShift) & 0x3);
  float others[3] = {};
  float sum_sq = 0.0f;
  for (int j = 0; j < 3; j++) {
    const uint32_t code = static_cast<uint32_t>((v >> (15 * j)) & 0x7fff);
    others[j] = (static_cast<float>(code) /
                 static_cast<float>(kQuat48Levels) * 2.0f - 1.0f) / kSqrt2;
    sum_sq += others[j] * others[j];
  }
  int j = 0;
  for (int k = 0; k < 4; k++) {
    if (k == i) { continue; }
    q[k] = others[j++];
  }
  const float rem = 1.0f - sum_sq;
  q[i] = std::sqrt(rem > 0.0f ? rem : 0.0f);
  const float n = std::sqrt(q[0] * q[0] + q[1] * q[1] +
                            q[2] * q[2] + q[3] * q[3]);
  if (n > 0.0f) {
    for (int k = 0; k < 4; k++) { q[k] /= n; }
  }
  return true;
}

}
