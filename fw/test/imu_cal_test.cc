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

#include "fw/imu_cal.h"

#include <limits>
#include <string>
#include <vector>

#include <boost/test/auto_unit_test.hpp>

namespace {
struct NameCollector {
  template <typename NameValuePair>
  void Visit(const NameValuePair& nvp) { names.push_back(nvp.name()); }
  std::vector<std::string> names;
};
}  // namespace

BOOST_AUTO_TEST_CASE(ImuCalDefaultsAreIdentity) {
  moteus::ImuCalConfig cal;
  for (int i = 0; i < 3; i++) {
    BOOST_TEST(cal.accel_bias[i] == 0.0f);
    BOOST_TEST(cal.accel_scale[i] == 1.0f);
  }
}

BOOST_AUTO_TEST_CASE(ImuCalSerializesAllArrays) {
  // The config keys are imu_cal.accel_bias.0..2, imu_cal.accel_scale.0..2
  // and imu_cal.gyro_bias.0..2; the robot updater's expected-added list
  // and the host's bias save depend on these names.
  moteus::ImuCalConfig cal;
  NameCollector c;
  cal.Serialize(&c);
  BOOST_TEST(c.names.size() == 3);
  BOOST_TEST(c.names[0] == "accel_bias");
  BOOST_TEST(c.names[1] == "accel_scale");
  BOOST_TEST(c.names[2] == "gyro_bias");
}

BOOST_AUTO_TEST_CASE(ImuCalGyroBiasGuard) {
  moteus::ImuCalConfig cal;
  cal.gyro_bias[0] = std::numeric_limits<float>::quiet_NaN();
  cal.gyro_bias[1] = 0.1f;        // past bias_max: not a gyro bias
  cal.gyro_bias[2] = -0.0043f;
  BOOST_TEST(cal.gyro_bias_rad_s(0) == 0.0f);
  BOOST_TEST(cal.gyro_bias_rad_s(1) == 0.0f);
  BOOST_TEST(cal.gyro_bias_rad_s(2) == -0.0043f);
}
