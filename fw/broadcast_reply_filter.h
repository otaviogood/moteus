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
#include <cstddef>
#include <cstdint>

namespace moteus {

/// A block of registers, start .. start + count - 1.
struct RegisterRange {
  uint16_t start = 0;
  uint16_t count = 0;
};

/// What a sensor board (g_otavio_flags bit 0: no gate driver found at
/// boot) answers in *broadcast* requests (CAN destination 0x7F), so
/// the robot's one shared telemetry request gets only the registers
/// the host uses from these boards.  Requests addressed to the board
/// itself are never filtered.
constexpr RegisterRange kSensorBoardBroadcastReads[] = {
  {0x050, 4},   // encoder 0/1 position and velocity (0x050-0x053)
  {0x065, 11},  // IMU: gyro rate slot 0x065-0x067, quaternion 0x06d-0x06f
};

namespace detail {

inline bool ReadVaruint(const uint8_t* data, size_t size, size_t* pos,
                        uint32_t* value) {
  uint32_t result = 0;
  for (int shift = 0; shift < 35; shift += 7) {
    if (*pos >= size) { return false; }
    const uint8_t byte = data[(*pos)++];
    result |= static_cast<uint32_t>(byte & 0x7f) << shift;
    if ((byte & 0x80) == 0) {
      *value = result;
      return true;
    }
  }
  return false;
}

inline bool Overlaps(const RegisterRange& range,
                     uint32_t start, uint32_t count) {
  return start < static_cast<uint32_t>(range.start) + range.count &&
      range.start < start + count;
}

inline bool KeepRead(const RegisterRange* begin, const RegisterRange* end,
                     uint32_t start, uint32_t count) {
  for (auto* range = begin; range != end; ++range) {
    if (Overlaps(*range, start, count)) { return true; }
  }
  return false;
}

}  // namespace detail

/// Keeps only the register reads of a broadcast request that touch one
/// of the ranges [begin, end), in place, and returns the new size (0:
/// nothing left to answer).  Only read subframes (0x10-0x1f) are
/// dropped, whole: a read that touches a range is kept with all its
/// registers.  Writes pass through and no-ops (the padding) are
/// dropped; at any other subframe type, or a malformed one, the rest
/// of the frame is kept unchanged.
inline size_t FilterBroadcastReads(const RegisterRange* begin,
                                   const RegisterRange* end,
                                   uint8_t* data, size_t size) {
  size_t read_pos = 0;
  size_t write_pos = 0;
  while (read_pos < size) {
    const size_t start = read_pos;
    uint32_t type = 0;
    bool keep = true;
    bool parsed = detail::ReadVaruint(data, size, &read_pos, &type);

    if (parsed && type == 0x50) {
      // No-op (also the padding of a rounded-up CAN FD frame).
      keep = false;
    } else if (parsed && type < 0x20) {
      // 0x00-0x0f write, 0x10-0x1f read: [count] register [values].
      uint32_t count = type & 0x03;
      if (count == 0) {
        parsed = detail::ReadVaruint(data, size, &read_pos, &count);
      }
      uint32_t reg = 0;
      parsed = parsed && detail::ReadVaruint(data, size, &read_pos, &reg);
      if (parsed && type < 0x10) {
        const uint32_t value_size =
            std::array<uint32_t, 4>{{1, 2, 4, 4}}[(type >> 2) & 0x03];
        read_pos += count * value_size;
        parsed = read_pos <= size;
      } else if (parsed) {
        keep = detail::KeepRead(begin, end, reg, count);
      }
    } else {
      parsed = false;
    }

    if (!parsed) {
      // Unknown or malformed: keep the rest as is.
      for (size_t i = start; i < size; i++) {
        data[write_pos++] = data[i];
      }
      return write_pos;
    }
    if (keep) {
      for (size_t i = start; i < read_pos; i++) {
        data[write_pos++] = data[i];
      }
    }
  }
  return write_pos;
}

}  // namespace moteus
