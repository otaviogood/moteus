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

#include "fw/command_update.h"

#include <vector>

#include <boost/test/auto_unit_test.hpp>

using namespace moteus;

BOOST_AUTO_TEST_CASE(CommandUpdateStrip) {
  std::vector<uint8_t> frame = {0x64, 0x0d, 0x22, 1, 2, 3, 4};
  size_t size = frame.size();
  BOOST_TEST(StripCommandUpdate(frame.data(), &size));
  BOOST_TEST(size == 6u);
  BOOST_TEST(frame[0] == 0x0d);
  BOOST_TEST(frame[5] == 4);

  // An ordinary command (mode write first), and an empty frame: left alone.
  std::vector<uint8_t> command = {0x01, 0x00, 0x0a};
  size = command.size();
  BOOST_TEST(!StripCommandUpdate(command.data(), &size));
  BOOST_TEST(size == 3u);
  size = 0;
  BOOST_TEST(!StripCommandUpdate(command.data(), &size));
}
