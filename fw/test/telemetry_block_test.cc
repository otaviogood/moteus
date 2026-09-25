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

#include "fw/telemetry_block.h"

#include <vector>

#include <boost/test/auto_unit_test.hpp>

using namespace moteus;
using Format = mjlib::multiplex::Format;

namespace {
using Bytes = std::vector<uint8_t>;

// A register server stand-in: every register reads as its own number
// in the requested type, except a few special cases.
Format::ReadResult FakeRead(uint16_t reg, size_t type) {
  if (reg == 0x066) {  // a NaN gyro axis, as the controller scales it
    return Format::Value(static_cast<int16_t>(-32768));
  }
  if (reg == 0x00f) { return static_cast<uint32_t>(1); }  // an error
  switch (type) {
    case 0: return Format::Value(static_cast<int8_t>(reg));
    case 1: return Format::Value(static_cast<int16_t>(reg * 10));
    case 2: return Format::Value(static_cast<int32_t>(reg * 1000));
  }
  return Format::Value(static_cast<float>(reg));
}
}

BOOST_AUTO_TEST_CASE(TelemetryBlockSizes) {
  static_assert(TelemetryBlockSize(kMotorTelemetryBlock) == 24);
  static_assert(TelemetryBlockSize(kSensorTelemetryBlock) == 17);
}

BOOST_AUTO_TEST_CASE(TelemetryBlockStripRequest) {
  Bytes frame = {0x60, 0x01, 0x11, 0x00, 0x50};
  size_t size = frame.size();
  BOOST_TEST(StripTelemetryBlockRequest(frame.data(), &size));
  BOOST_TEST(size == 3u);
  BOOST_TEST(frame[0] == 0x11);
  BOOST_TEST(frame[1] == 0x00);

  // An unknown layout version, or no marker: left alone.
  Bytes future = {0x60, 0x02};
  size = future.size();
  BOOST_TEST(!StripTelemetryBlockRequest(future.data(), &size));
  BOOST_TEST(size == 2u);
  Bytes plain = {0x11, 0x00};
  size = plain.size();
  BOOST_TEST(!StripTelemetryBlockRequest(plain.data(), &size));
}

BOOST_AUTO_TEST_CASE(TelemetryBlockMotorLayout) {
  uint8_t out[64] = {};
  const size_t n = WriteTelemetryBlock(false, FakeRead, out);
  BOOST_REQUIRE(n == 24u);
  const Bytes expected = {
    0x61,
    0x42, 0x04, 0x4c, 0x04, 0x56, 0x04,  // 0x06d-0x06f: 1090, 1100, 1110
    0xf2, 0x03, 0x00, 0x80, 0x06, 0x04,  // 0x065 1010, 0x066 NaN, 0x067 1030
    0xa0, 0x0f, 0x00, 0x00,              // 0x004: 4000
    0x46, 0x00,                          // 0x007 (int16): 70
    0x0a, 0x0d, 0x0e,                    // motor temp, voltage, board temp
    0x80,                                // 0x00f: error -> NaN code
    0x00,                                // mode
  };
  BOOST_TEST(Bytes(out, out + n) == expected);
}

BOOST_AUTO_TEST_CASE(TelemetryBlockSensorLayout) {
  uint8_t out[64] = {};
  const size_t n = WriteTelemetryBlock(true, FakeRead, out);
  BOOST_REQUIRE(n == 17u);
  const Bytes expected = {
    0x62,
    0x20, 0x03, 0x2a, 0x03,              // 0x050 800, 0x051 810
    0x42, 0x04, 0x4c, 0x04, 0x56, 0x04,
    0xf2, 0x03, 0x00, 0x80, 0x06, 0x04,
  };
  BOOST_TEST(Bytes(out, out + n) == expected);
}

BOOST_AUTO_TEST_CASE(TelemetryBlockFloatNanToInt) {
  // A register that answers a float NaN in an integer slot goes out as
  // the NaN code rather than an arbitrary conversion.
  auto nan_read = [](uint16_t, size_t) {
    return Format::ReadResult(
        Format::Value(std::numeric_limits<float>::quiet_NaN()));
  };
  uint8_t out[64] = {};
  const size_t n = WriteTelemetryBlock(true, nan_read, out);
  BOOST_REQUIRE(n == 17u);
  BOOST_TEST(out[1] == 0x00);
  BOOST_TEST(out[2] == 0x80);
}

BOOST_AUTO_TEST_CASE(TelemetryBlockFitReplyItems) {
  const Bytes reply = {
    0x2e, 0x01, 0, 0, 0, 0, 0, 0, 0, 0,  // float x2 @0x001 (10 bytes)
    0x21, 0x70, 0x05,                    // int8 @0x070 (3 bytes)
    0x31, 0x99, 0x01, 0x02,              // read error, reg 0x99 (varuint), err 2
    0x50,                                // nop
  };
  BOOST_TEST(FitReplyItems(reply.data(), reply.size(), 64) == reply.size());
  BOOST_TEST(FitReplyItems(reply.data(), reply.size(), 12) == 10u);
  BOOST_TEST(FitReplyItems(reply.data(), reply.size(), 13) == 13u);
  BOOST_TEST(FitReplyItems(reply.data(), reply.size(), 16) == 13u);
  BOOST_TEST(FitReplyItems(reply.data(), reply.size(), 9) == 0u);

  // A tunneled stream is kept only whole.
  const Bytes stream = {0x21, 0x00, 0x0a, 0x41, 0x01, 0x03, 'a', 'b', 'c'};
  BOOST_TEST(FitReplyItems(stream.data(), stream.size(), 8) == 3u);
}
