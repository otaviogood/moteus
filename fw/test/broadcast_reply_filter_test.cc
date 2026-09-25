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

#include "fw/broadcast_reply_filter.h"

#include <iterator>
#include <vector>

#include <boost/test/auto_unit_test.hpp>

using namespace moteus;

namespace {
using Bytes = std::vector<uint8_t>;

// humanoid3 orin/can_backend.request_joint_telemetry.
const Bytes kRobotRequest = {
  0x15, 0x50,  // int16 x1 @0x050  aux1 encoder position
  0x17, 0x6d,  // int16 x3 @0x06d  quat48
  0x1d, 0x01,  // float x1 @0x001  position
  0x1d, 0x02,  // float x1 @0x002  velocity
  0x16, 0x0d,  // int16 x2 @0x00d  voltage, board temperature
  0x11, 0x0a,  // int8  x1 @0x00a  motor temperature
  0x15, 0x07,  // int16 x1 @0x007  power
  0x15, 0x04,  // int16 x1 @0x004  q current
  0x11, 0x00,  // int8  x1 @0x000  mode
  0x11, 0x0f,  // int8  x1 @0x00f  fault
};

using Ranges = std::vector<RegisterRange>;

Bytes Filter(const Ranges& ranges, Bytes data) {
  data.resize(FilterBroadcastReads(ranges.data(), ranges.data() + ranges.size(),
                                   data.data(), data.size()));
  return data;
}
}

BOOST_AUTO_TEST_CASE(BroadcastReplySensorBoard) {
  const Ranges sensor(std::begin(kSensorBoardBroadcastReads),
                      std::end(kSensorBoardBroadcastReads));
  const Bytes expected = {0x15, 0x50, 0x17, 0x6d};
  BOOST_TEST(Filter(sensor, kRobotRequest) == expected);
}

BOOST_AUTO_TEST_CASE(BroadcastReplyMultiRegisterReads) {
  // A read that touches a range is kept whole.
  const Bytes request = {0x17, 0x0d};  // int16 x3 @0x00d-0x00f
  BOOST_TEST(Filter(Ranges{{0x00f, 1}}, request) == request);
  BOOST_TEST(Filter(Ranges{{0x00a, 3}}, request).empty());
}

BOOST_AUTO_TEST_CASE(BroadcastReplyVaruintCountAndWrites) {
  // Writes pass through with their payload; a read with a varuint
  // count (low bits 0) and a two-byte register is filtered correctly.
  const Bytes request = {
    0x01, 0x00, 0x0a,              // write int8 x1 @0x000 = 10
    0x0d, 0x20, 0, 0, 0x80, 0x3f,  // write float x1 @0x020 = 1.0
    0x14, 0x05, 0x80, 0x02,        // read int16 x5 @0x100
    0x15, 0x50,                    // read int16 x1 @0x050
  };
  const Bytes expected = {
    0x01, 0x00, 0x0a,
    0x0d, 0x20, 0, 0, 0x80, 0x3f,
    0x15, 0x50,
  };
  BOOST_TEST(Filter(Ranges{{0x050, 1}}, request) == expected);
}

BOOST_AUTO_TEST_CASE(BroadcastReplyPaddingAndEmpty) {
  const Ranges config{{0x06d, 3}};
  // CAN FD padding (0x50 no-ops) goes too.
  BOOST_TEST(Filter(config, {0x17, 0x6d, 0x50, 0x50}) == (Bytes{0x17, 0x6d}));
  // Nothing this board answers: an empty frame.
  BOOST_TEST(Filter(config, {0x11, 0x00, 0x50}).empty());
}

BOOST_AUTO_TEST_CASE(BroadcastReplyUnknownOrMalformedKeepsRest) {
  const Ranges config{{0x06d, 3}};
  // A tunnel subframe (0x40) ends filtering; the rest is untouched.
  BOOST_TEST(Filter(config, {0x11, 0x00, 0x40, 0x01, 0x02, 0x11, 0x0f}) ==
             (Bytes{0x40, 0x01, 0x02, 0x11, 0x0f}));
  // A truncated write is kept as is.
  BOOST_TEST(Filter(config, {0x11, 0x00, 0x0d, 0x20, 0x00}) ==
             (Bytes{0x0d, 0x20, 0x00}));
}
