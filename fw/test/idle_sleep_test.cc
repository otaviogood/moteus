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

#include "fw/idle_sleep.h"

#include <boost/test/auto_unit_test.hpp>

using namespace moteus;

BOOST_AUTO_TEST_CASE(IdleSleepRequestParse) {
  uint8_t seconds = 99;
  uint8_t flags = 99;
  const uint8_t request[] = {kIdleSleepRequest, kIdleSleepFormat, 5};
  BOOST_TEST(ParseIdleSleepRequest(request, sizeof(request), &seconds, &flags));
  BOOST_TEST(seconds == 5);
  BOOST_TEST(flags == 0);

  const uint8_t forever[] = {kIdleSleepRequest, kIdleSleepFormat, 0};
  BOOST_TEST(ParseIdleSleepRequest(forever, sizeof(forever), &seconds, &flags));
  BOOST_TEST(seconds == 0);

  const uint8_t watchdog_only[] = {kIdleSleepRequest, kIdleSleepFormat, 7,
                                   kIdleSleepBenchNoBusWake};
  BOOST_TEST(ParseIdleSleepRequest(watchdog_only, sizeof(watchdog_only),
                                   &seconds, &flags));
  BOOST_TEST(seconds == 7);
  BOOST_TEST(flags == kIdleSleepBenchNoBusWake);

  // Too short, an unknown format, another subframe, or a sleep nothing
  // could end: not a request, and the outputs are left alone.
  seconds = 99;
  flags = 99;
  BOOST_TEST(!ParseIdleSleepRequest(request, 2, &seconds, &flags));
  const uint8_t unknown[] = {kIdleSleepRequest, 0x02, 5};
  BOOST_TEST(!ParseIdleSleepRequest(unknown, sizeof(unknown), &seconds, &flags));
  const uint8_t audio[] = {0x66, 0x02, 0x00};
  BOOST_TEST(!ParseIdleSleepRequest(audio, sizeof(audio), &seconds, &flags));
  const uint8_t stuck[] = {kIdleSleepRequest, kIdleSleepFormat, 0,
                           kIdleSleepBenchNoBusWake};
  BOOST_TEST(!ParseIdleSleepRequest(stuck, sizeof(stuck), &seconds, &flags));
  BOOST_TEST(seconds == 99);
  BOOST_TEST(flags == 99);
}
