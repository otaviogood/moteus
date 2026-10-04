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

#include "fw/motor_thermal_model.h"

#include <cmath>
#include <limits>

#include <boost/test/auto_unit_test.hpp>

using namespace moteus;

namespace {
constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
constexpr float kDt = 0.001f;

// One node (no slow node): winding 12.6 J/C to the FET via 1.5 C/W, no
// resistance scale or tempco.
MotorThermalParams OneNode() {
  MotorThermalParams p;
  p.winding_J_per_C = 12.6f;
  p.winding_to_slow_C_per_W = 1.5f;
  p.slow_J_per_C = 0.0f;
  p.resistance_scale = 1.0f;
  p.resistance_tempco = 0.0f;
  return p;
}

float Run(MotorThermalModel* model, float seconds, float i2, float fet) {
  float result = kNaN;
  const int steps = static_cast<int>(std::lround(seconds / kDt));
  for (int i = 0; i < steps; i++) {
    result = model->Update(i2, fet);
  }
  return result;
}
}

BOOST_AUTO_TEST_CASE(MotorThermalStartsAtFet) {
  // At power-up the first readings can be far off (the ADC is not running
  // yet); the estimate is the FET reading plus a rise, so it follows the
  // FET from the first good reading on.
  MotorThermalModel model;
  model.Configure(0.3f, kDt, OneNode());
  BOOST_TEST(model.rise_C() == 0.0f);
  BOOST_TEST(model.Update(0.0f, -40.0f) == -40.0f);
  BOOST_TEST(model.Update(0.0f, 31.0f) == 31.0f);
  BOOST_TEST(Run(&model, 5.0f, 0.0f, 31.0f) == 31.0f);
}

BOOST_AUTO_TEST_CASE(MotorThermalStepResponse) {
  // 10 A, 0.3 ohm: 45 W.  Winding 12.6 J/C to the FET via 1.5 C/W:
  // steady rise 67.5 C, time constant 18.9 s.
  MotorThermalModel model;
  model.Configure(0.3f, kDt, OneNode());
  model.Update(0.0f, 25.0f);
  const float p = 1.5f * 0.3f * 100.0f;
  const float tau = 12.6f * 1.5f;
  // One step: exactly P dt / C.
  model.Update(100.0f, 25.0f);
  BOOST_TEST(model.rise_C() == p * kDt / 12.6f, boost::test_tools::tolerance(1e-4f));
  const float at_5s = Run(&model, 5.0f - kDt, 100.0f, 25.0f);
  BOOST_TEST(at_5s - 25.0f == p * 1.5f * (1.0f - std::exp(-5.0f / tau)),
             boost::test_tools::tolerance(0.01f));
  const float steady = Run(&model, 300.0f, 100.0f, 25.0f);
  BOOST_TEST(steady - 25.0f == p * 1.5f, boost::test_tools::tolerance(0.01f));

  // It cools back to the FET with the same time constant.
  const float cooled = Run(&model, tau, 0.0f, 25.0f);
  BOOST_TEST(cooled - 25.0f == p * 1.5f * std::exp(-1.0f),
             boost::test_tools::tolerance(0.01f));
}

BOOST_AUTO_TEST_CASE(MotorThermalFollowsFet) {
  // With no current the estimate is the FET reading; after heating, the
  // rise sits on top of the new reading.
  MotorThermalModel model;
  model.Configure(0.3f, kDt, OneNode());
  BOOST_TEST(model.Update(0.0f, 40.0f) == 40.0f);
  Run(&model, 5.0f, 100.0f, 40.0f);
  const float rise = model.rise_C();
  BOOST_TEST(rise > 10.0f);
  BOOST_TEST(model.Update(0.0f, 45.0f) == 45.0f + model.rise_C());
  BOOST_TEST(model.rise_C() < rise);
}

BOOST_AUTO_TEST_CASE(MotorThermalTwoNode) {
  // The default (two-node) parameters, without the resistance scale and
  // tempco: steady rise P (R1 + R2); early on the winding alone.
  MotorThermalParams params;
  params.resistance_scale = 1.0f;
  params.resistance_tempco = 0.0f;
  MotorThermalModel model;
  model.Configure(0.3f, kDt, params);
  model.Update(0.0f, 25.0f);
  const float p = 1.5f * 0.3f * 100.0f;
  const float at_1s = Run(&model, 1.0f, 100.0f, 25.0f) - 25.0f;
  BOOST_TEST(at_1s > 0.9f * p * 1.0f / params.winding_J_per_C);
  BOOST_TEST(at_1s < p * 1.0f / params.winding_J_per_C);
  BOOST_TEST(model.slow_rise_C() > 0.0f);
  BOOST_TEST(model.slow_rise_C() < 0.1f * at_1s);
  const float steady = Run(&model, 600.0f, 100.0f, 25.0f) - 25.0f;
  BOOST_TEST(steady == p * (params.winding_to_slow_C_per_W +
                            params.slow_to_fet_C_per_W),
             boost::test_tools::tolerance(0.01f));
  BOOST_TEST(model.slow_rise_C() == p * params.slow_to_fet_C_per_W,
             boost::test_tools::tolerance(0.01f));

  // Without a slow node (C2 0) it is cleared.
  params.slow_J_per_C = 0.0f;
  model.Configure(0.3f, kDt, params);
  BOOST_TEST(model.slow_rise_C() == 0.0f);
}

BOOST_AUTO_TEST_CASE(MotorThermalResistanceTempco) {
  // The copper loss grows with the winding temperature.
  auto params = OneNode();
  params.resistance_scale = 1.1f;
  params.resistance_tempco = 0.00393f;
  MotorThermalModel model;
  model.Configure(0.3f, kDt, params);
  model.Update(0.0f, 25.0f);
  const float steady = Run(&model, 400.0f, 100.0f, 25.0f);
  // Fixed point: rise = 1.5 * 0.33 * (1 + a rise) * 100 * 1.5.
  const float k = 1.5f * 0.33f * 100.0f * 1.5f;
  const float rise = k / (1.0f - 0.00393f * k);
  BOOST_TEST(steady - 25.0f == rise, boost::test_tools::tolerance(0.01f));
}

BOOST_AUTO_TEST_CASE(MotorThermalRightKneePulse) {
  // The robot's right knee, 10 A for 5 s (2026-10-02): the copper rose
  // 17 C by its resistance while the thermistor stayed within 0.4 C.
  // The (conservative) defaults reproduce the copper rise.
  MotorThermalModel model;
  model.Configure(0.275778f, kDt);
  model.Update(0.0f, 27.1f);
  const float end = Run(&model, 5.0f, 100.0f, 27.3f);
  BOOST_TEST(end - 27.3f > 14.0f);
  BOOST_TEST(end - 27.3f < 20.0f);
}

BOOST_AUTO_TEST_CASE(MotorThermalInvalidInputs) {
  // An uncalibrated motor (resistance 0) adds no heat.
  MotorThermalModel model;
  model.Configure(0.0f, kDt, OneNode());
  model.Update(0.0f, 25.0f);
  BOOST_TEST(Run(&model, 10.0f, 100.0f, 25.0f) == 25.0f);

  // A non-finite FET reading gives a NaN estimate (protection then falls
  // back to the thermistor) without corrupting the rise.
  model.Configure(0.3f, kDt, OneNode());
  Run(&model, 10.0f, 100.0f, 25.0f);
  BOOST_TEST(std::isnan(model.Update(100.0f, kNaN)));
  BOOST_TEST(std::isfinite(model.rise_C()));
  BOOST_TEST(model.rise_C() > 20.0f);
  BOOST_TEST(model.Update(0.0f, 30.0f) > 50.0f);
  model.Reset();
  BOOST_TEST(model.Update(0.0f, 30.0f) == 30.0f);
}

BOOST_AUTO_TEST_CASE(MotorThermalOffsetHotterOfTwo) {
  // The hotter of the estimate and the thermistor.
  BOOST_TEST(MotorThermalOffset(45.0f, 30.0f) == 15.0f);
  BOOST_TEST(MotorThermalOffset(25.0f, 30.0f) == 0.0f);
  // servo.enable_motor_temperature 0: the thermistor reads 0 and the
  // estimate alone protects.
  BOOST_TEST(MotorThermalOffset(40.0f, 0.0f) == 40.0f);
  // Non-finite: protect on the thermistor alone.
  BOOST_TEST(MotorThermalOffset(kNaN, 30.0f) == 0.0f);
  BOOST_TEST(MotorThermalOffset(40.0f, kNaN) == 0.0f);
}
