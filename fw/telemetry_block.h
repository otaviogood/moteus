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
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iterator>
#include <limits>
#include <type_traits>
#include <variant>

#include "mjlib/multiplex/format.h"

/// Fork-specific: a fixed-layout telemetry reply for the humanoid robot
/// (docs/protocol/registers.md, "Telemetry block").
///
/// A request whose first two bytes are kTelemetryBlockRequest and a
/// layout version this firmware knows gets, in front of the normal
/// reply to the rest of the frame, one type byte (which layout) and the
/// values of that layout's registers, packed little-endian with no
/// per-register headers.  Each value is exactly what a register read of
/// that type returns (same scaling, same NaN code).  Motor and sensor
/// boards have their own layouts.
namespace moteus {

constexpr uint8_t kTelemetryBlockRequest = 0x60;  // unused by mjlib
constexpr uint8_t kTelemetryBlockVersion = 1;
constexpr uint8_t kTelemetryBlockMotor = 0x61;    // version 1, motor board
constexpr uint8_t kTelemetryBlockSensor = 0x62;   // version 1, sensor board

struct BlockRead {
  uint16_t reg;
  uint8_t type;  // 0 int8, 1 int16, 2 int32, 3 float
};

constexpr BlockRead kMotorTelemetryBlock[] = {
  {0x06d, 1}, {0x06e, 1}, {0x06f, 1},  // quaternion, quat48 words
  {0x065, 1}, {0x066, 1}, {0x067, 1},  // gyro rate, 0.001 rad/s
  {0x004, 2},                          // q current, 0.001 A
  {0x007, 1},                          // power, 0.05 W
  {0x00a, 0},                          // motor temperature, 1 C
  {0x00d, 0}, {0x00e, 0}, {0x00f, 0},  // bus voltage 0.5 V, board temp 1 C, fault
  {0x000, 0},                          // mode
};

constexpr BlockRead kSensorTelemetryBlock[] = {
  {0x050, 1}, {0x051, 1},              // encoder 0: 0.0001 rev, 0.00025 rev/s
  {0x06d, 1}, {0x06e, 1}, {0x06f, 1},  // quaternion, quat48 words
  {0x065, 1}, {0x066, 1}, {0x067, 1},  // gyro rate, 0.001 rad/s
};

namespace detail {

inline size_t BlockTypeSize(uint8_t type) {
  return type == 0 ? 1 : type == 1 ? 2 : 4;
}

template <typename T>
void PutLe(T value, uint8_t* out) {
  std::memcpy(out, &value, sizeof(value));  // the STM32 is little-endian
}

inline bool ReadBlockVaruint(const uint8_t* data, size_t size, size_t* pos,
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

}  // namespace detail

/// If the frame [data, *size) starts with a block request of a version
/// this firmware knows, remove it in place and return true.
inline bool StripTelemetryBlockRequest(uint8_t* data, size_t* size) {
  if (*size < 2 || data[0] != kTelemetryBlockRequest ||
      data[1] != kTelemetryBlockVersion) {
    return false;
  }
  std::memmove(data, data + 2, *size - 2);
  *size -= 2;
  return true;
}

template <size_t N>
constexpr size_t TelemetryBlockSize(const BlockRead (&layout)[N]) {
  size_t result = 1;
  for (const auto& item : layout) {
    result += item.type == 0 ? 1 : item.type == 1 ? 2 : 4;
  }
  return result;
}

/// Write this board's block (type byte, then the values) to out and
/// return its size.  read(reg, type) returns a
/// mjlib::multiplex::Format::ReadResult; a register that reports an
/// error goes out as the type's NaN code.
template <typename ReadFn>
size_t WriteTelemetryBlock(bool sensor, ReadFn read, uint8_t* out) {
  using Format = mjlib::multiplex::Format;
  const BlockRead* begin = sensor ?
      std::begin(kSensorTelemetryBlock) : std::begin(kMotorTelemetryBlock);
  const BlockRead* end = sensor ?
      std::end(kSensorTelemetryBlock) : std::end(kMotorTelemetryBlock);
  size_t pos = 0;
  out[pos++] = sensor ? kTelemetryBlockSensor : kTelemetryBlockMotor;
  for (auto* item = begin; item != end; ++item) {
    const Format::ReadResult result = read(item->reg, item->type);
    const auto* value = std::get_if<Format::Value>(&result);
    // Convert whatever the register returned to the layout's type, so
    // the layout never shifts.
    auto as = [&](auto nan_code) {
      using T = decltype(nan_code);
      if (!value) { return nan_code; }
      return std::visit([&](auto v) -> T {
        if constexpr (std::is_floating_point_v<decltype(v)> &&
                      !std::is_floating_point_v<T>) {
          if (!std::isfinite(v)) { return nan_code; }
        }
        return static_cast<T>(v);
      }, *value);
    };
    switch (item->type) {
      case 0: { detail::PutLe(as(std::numeric_limits<int8_t>::min()), out + pos); break; }
      case 1: { detail::PutLe(as(std::numeric_limits<int16_t>::min()), out + pos); break; }
      case 2: { detail::PutLe(as(std::numeric_limits<int32_t>::min()), out + pos); break; }
      default: { detail::PutLe(as(std::numeric_limits<float>::quiet_NaN()), out + pos); break; }
    }
    pos += detail::BlockTypeSize(item->type);
  }
  return pos;
}

/// The length of the longest run of whole reply subframes at the start
/// of [data, size) that fits in room bytes.  Register replies and
/// errors are measured item by item; at any other subframe type (a
/// tunneled stream) the rest is kept only if all of it fits.
inline size_t FitReplyItems(const uint8_t* data, size_t size, size_t room) {
  if (size <= room) { return size; }
  size_t pos = 0;
  size_t fit = 0;
  while (pos < size) {
    uint32_t type = 0;
    if (!detail::ReadBlockVaruint(data, size, &pos, &type)) { break; }
    bool ok = true;
    if (type == 0x50) {
      // Nop: one byte.
    } else if (type >= 0x20 && type < 0x30) {
      uint32_t count = type & 0x03;
      if (count == 0) {
        ok = detail::ReadBlockVaruint(data, size, &pos, &count);
      }
      uint32_t reg = 0;
      ok = ok && detail::ReadBlockVaruint(data, size, &pos, &reg);
      pos += count * detail::BlockTypeSize((type >> 2) & 0x03);
    } else if (type == 0x30 || type == 0x31) {
      uint32_t reg = 0, err = 0;
      ok = detail::ReadBlockVaruint(data, size, &pos, &reg) &&
          detail::ReadBlockVaruint(data, size, &pos, &err);
    } else {
      break;
    }
    if (!ok || pos > size || pos > room) { break; }
    fit = pos;
  }
  return fit;
}

}  // namespace moteus
