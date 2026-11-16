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

#include <cstddef>
#include <cstdint>
#include <cstring>

/// Fork-specific: the IMU gyro as a vibration pickup, a signed waveform
/// that can be played back (docs/protocol/registers.md, "Gyro audio").
///
/// The gyro measures angular velocity, so the change from one 960 Hz
/// sample to the next is the angular acceleration: what a contact
/// microphone on the joint would pick up.  Every gyro sample becomes
/// three signed bytes, one per axis: that change, compressed like
/// telephone audio (fine steps when quiet, coarse steps when loud).
///
///   magnitude   code       step
///   0 .. 31     0 .. 31    1       (exact; 1 count = 70 mdps)
///   32 .. 62    32 .. 47   2
///   64 .. 124   48 .. 63   4
///   ...
///   1024..1984  112..127   64
///
/// The byte is the code with the sign of the change (int8, -127..127).
/// Each code is the change from the value the receiver has rebuilt so
/// far, not from the previous raw sample, so rounding does not add up:
/// the running sum of the decoded codes follows the gyro to within half
/// a step, and a change beyond +-1984 counts (139 deg/s in one sample)
/// is caught up over the following samples instead of lost.
///
/// A sample with nothing before it (the first after a restart, the
/// first after samples the board lost) reads 0 on every axis and the
/// running sum starts over from there.
///
/// The codes wait in a small ring.  A request frame whose first two
/// bytes are kGyroAudioRequest, kGyroAudioFormat is answered with the
/// samples since the previous reply:
///
///   byte 0     kGyroAudioFormat
///   byte 1     n, the number of samples that follow
///   byte 2-3   sample number of the first one (uint16, little-endian;
///              counts every gyro sample since the fusion started,
///              lost ones included, and wraps)
///   byte 4..   n samples, oldest first, 3 bytes each: x, y, z
///
/// The reply has the 11-bit ID GyroAudioReplyId(): above every command
/// and telemetry ID, so on the bus it loses arbitration to all of them.
namespace moteus {

constexpr uint8_t kGyroAudioRequest = 0x66;  // unused by mjlib
constexpr uint8_t kGyroAudioFormat = 2;
constexpr size_t kGyroAudioHeaderSize = 4;
constexpr size_t kGyroAudioSampleSize = 3;
constexpr int32_t kGyroAudioMaxStep = 1984;

constexpr uint32_t kGyroAudioReplyIdBase = 0x780;

constexpr uint32_t GyroAudioReplyId(uint8_t board_id) {
  return kGyroAudioReplyIdBase | (board_id & 0x7f);
}

/// The value a code stands for, raw counts.
constexpr int32_t GyroAudioDecode(int8_t code) {
  const int32_t c = code < 0 ? -static_cast<int32_t>(code) : code;
  const int32_t m =
      (c < 32) ? c :
      (c > 127) ? kGyroAudioMaxStep :
      ((16 + (c & 15)) << ((c >> 4) - 1));
  return code < 0 ? -m : m;
}

/// The code nearest to a change of d raw counts.
inline int8_t GyroAudioEncode(int32_t d) {
  const uint32_t m = d < 0 ? 0u - static_cast<uint32_t>(d) : d;
  int32_t code = 0;
  if (m < 32) {
    code = m;
  } else if (m >= static_cast<uint32_t>(kGyroAudioMaxStep)) {
    code = 127;
  } else {
    int shift = (31 - __builtin_clz(m)) - 4;       // 1 .. 6
    uint32_t mantissa = (m + (1u << (shift - 1))) >> shift;  // 16 .. 32
    if (mantissa == 32) {
      shift++;
      mantissa = 16;
    }
    code = (shift + 1) * 16 + (mantissa - 16);
  }
  return static_cast<int8_t>(d < 0 ? -code : code);
}

/// If the frame [data, *size) starts with a gyro audio request in a
/// format this firmware knows, remove it in place and return true.
inline bool StripGyroAudioRequest(uint8_t* data, size_t* size) {
  if (*size < 2 || data[0] != kGyroAudioRequest ||
      data[1] != kGyroAudioFormat) {
    return false;
  }
  std::memmove(data, data + 2, *size - 2);
  *size -= 2;
  return true;
}

/// The codes of the newest kSize samples.  One context only (the main
/// loop): the fusion pushes, the CAN reply takes.
class GyroAudio {
 public:
  static constexpr uint16_t kSize = 64;

  /// The next gyro sample, raw counts.
  void Push(const int16_t* w) {
    int8_t* const out = buf_[total_ % kSize];
    for (int i = 0; i < 3; i++) {
      if (!synced_) {
        rebuilt_[i] = w[i];
        out[i] = 0;
      } else {
        const int8_t code = GyroAudioEncode(w[i] - rebuilt_[i]);
        rebuilt_[i] += GyroAudioDecode(code);
        out[i] = code;
      }
    }
    synced_ = true;
    Advance();
  }

  /// lost samples never arrived: they count, and read 0.
  void Skip(uint16_t lost) {
    const uint16_t zeros = lost < kSize ? lost : kSize;
    if (lost > zeros) {
      // Everything before the gap is older than the ring.
      total_ = static_cast<uint16_t>(total_ + (lost - zeros));
      sent_ = total_;
    }
    for (uint16_t i = 0; i < zeros; i++) {
      std::memset(buf_[total_ % kSize], 0, kGyroAudioSampleSize);
      Advance();
    }
    synced_ = false;
  }

  /// The next sample does not follow the previous one.
  void ResetHistory() { synced_ = false; }

  /// A new sensor (the fusion was attached again): samples not sent
  /// yet are dropped, the sample number carries on.
  void Restart() {
    sent_ = total_;
    synced_ = false;
  }

  /// Write a reply with the samples not sent yet, as many as fit in
  /// room bytes, and return its size.  Samples older than the ring are
  /// skipped; the sample number shows the jump.
  size_t WriteReply(uint8_t* out, size_t room) {
    if (room < kGyroAudioHeaderSize) { return 0; }
    uint16_t pending = static_cast<uint16_t>(total_ - sent_);
    if (pending > kSize) {
      sent_ = static_cast<uint16_t>(total_ - kSize);
      pending = kSize;
    }
    size_t n = (room - kGyroAudioHeaderSize) / kGyroAudioSampleSize;
    if (n > pending) { n = pending; }
    out[0] = kGyroAudioFormat;
    out[1] = static_cast<uint8_t>(n);
    out[2] = static_cast<uint8_t>(sent_ & 0xff);
    out[3] = static_cast<uint8_t>(sent_ >> 8);
    out += kGyroAudioHeaderSize;
    for (size_t i = 0; i < n; i++) {
      std::memcpy(out, buf_[sent_ % kSize], kGyroAudioSampleSize);
      out += kGyroAudioSampleSize;
      sent_++;
    }
    return kGyroAudioHeaderSize + n * kGyroAudioSampleSize;
  }

  uint16_t total() const { return total_; }

 private:
  // One more sample; the unsent ones never exceed the ring, so their
  // 16-bit count cannot wrap however long nobody asks.
  void Advance() {
    total_++;
    if (static_cast<uint16_t>(total_ - sent_) > kSize) {
      sent_ = static_cast<uint16_t>(total_ - kSize);
    }
  }

  int32_t rebuilt_[3] = {};
  bool synced_ = false;
  uint16_t total_ = 0;
  uint16_t sent_ = 0;
  int8_t buf_[kSize][kGyroAudioSampleSize] = {};
};

}  // namespace moteus
