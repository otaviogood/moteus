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
#include <cmath>

#include "mjlib/base/visitor.h"

namespace moteus {

// Per-board IMU calibration, persisted as the top-level `imu_cal`
// configuration group (docs/imu_orientation_redesign.md, calibration
// plan step 2/3).  Whichever aux port runs the IMU fusion applies it.
//
// Accelerometer: the fusion uses a = (raw - accel_bias) / accel_scale,
// in g.  Bias comes from the six-position test on the bench
// (utils/imu_fusion_bench/six_point.py) or the encoder-referenced fit
// on the robot.  A scale outside (0.5, 2) is treated as 1, and a
// non-finite bias is treated as 0.
struct ImuCalConfig {
  std::array<float, 3> accel_bias = {0.0f, 0.0f, 0.0f};   // g
  std::array<float, 3> accel_scale = {1.0f, 1.0f, 1.0f};  // unitless

  float bias(int axis) const {
    const float value = accel_bias[axis];
    return std::isfinite(value) ? value : 0.0f;
  }

  float scale(int axis) const {
    const float value = accel_scale[axis];
    return (value > 0.5f && value < 2.0f) ? value : 1.0f;
  }

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(accel_bias));
    a->Visit(MJ_NVP(accel_scale));
  }
};

}  // namespace moteus
