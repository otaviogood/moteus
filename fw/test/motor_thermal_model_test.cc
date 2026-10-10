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

float Run2(WindingRise* w, float seconds, float power_W) {
  float result = 0.0f;
  const int steps = static_cast<int>(std::lround(seconds / kDt));
  for (int i = 0; i < steps; i++) {
    result = w->Update(power_W);
  }
  return result;
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

BOOST_AUTO_TEST_CASE(MotorThermalWindingRise) {
  // 10 A at 0.3 ohm (45 W): steady rise 45 * 1.26 = 56.7 C, tau 15 s.
  WindingRise w;
  w.Configure(kDt);
  const float p = 45.0f;
  const float at_15s = Run2(&w, 15.0f, p);
  BOOST_TEST(at_15s == p * 1.26f * (1.0f - std::exp(-1.0f)),
             boost::test_tools::tolerance(0.02f));
  BOOST_TEST(Run2(&w, 150.0f, p) == p * 1.26f, boost::test_tools::tolerance(0.01f));
  // Cooling follows the current, not the housing: the rise decays to 0.
  BOOST_TEST(Run2(&w, 75.0f, 0.0f) < 0.5f);
}

BOOST_AUTO_TEST_CASE(MotorThermalMonitorTrips) {
  MotorThermalParams p;
  ThermistorMonitor m;
  m.Configure(kDt, p);
  // Both flat: no fallback.
  for (int i = 0; i < 30000; i++) {
    BOOST_TEST(!m.Update(30.0f, 30.0f, 32.0f, 1.0f, true));
  }
  // A 2 s burst where the estimate leads a lagging sensor: no trip
  // (persistence), as at every heating onset on a healthy motor.
  for (int i = 0; i < 2000; i++) {
    BOOST_TEST(!m.Update(30.0f + 0.001f * i, 30.0f + 0.001f * i, 30.0f + 0.004f * i, 1.0f, true));
  }
  for (int i = 0; i < 30000; i++) { m.Update(32.0f, 32.0f, 38.0f, 1.0f, true); }
  // The estimate climbs 1 C/s and the thermistor follows it: still none.
  float t = 32.0f;
  for (int i = 0; i < 30000; i++) {
    t += 0.001f;
    BOOST_TEST(!m.Update(t, t, t + 6.0f, 1.0f, true));
  }
  // The estimate keeps climbing, the thermistor stops: the thermistor's
  // 20 s-window rise decays from 20 C below 0.3 x the estimate's (20 C)
  // after ~24 s.
  float e = t + 6.0f;
  int tripped_at = -1;
  for (int i = 0; i < 40000; i++) {
    e += 0.001f;
    if (m.Update(t, t, e, 1.0f, true)) { tripped_at = i; break; }
  }
  BOOST_TEST(tripped_at > 15000);
  BOOST_TEST(tripped_at < 30000);
  // The estimate now leads by ~30 C: the latch keeps the fallback on past
  // the hold (the lead itself keeps re-tripping); once the lead is back
  // under 15 C the hold runs out and it releases.
  for (int i = 0; i < 130000; i++) { BOOST_TEST(m.Update(t, t, e, 1.0f, false)); }
  int held = 0;
  while (m.Update(t, t, t + 10.0f, 1.0f, false) && held < 200000) { held++; }
  BOOST_TEST(held > 115000);
  BOOST_TEST(held < 125000);
}

BOOST_AUTO_TEST_CASE(MotorThermalMonitorHoldAfterTrip) {
  // A trip with a small lead releases 120 s after the last trip.
  MotorThermalParams p;
  ThermistorMonitor m;
  m.Configure(kDt, p);
  for (int i = 0; i < 30000; i++) { m.Update(50.0f, 50.0f, 52.0f, 1.0f, true); }
  // The thermistor falls 8 C in 2 s while driven: trips.
  bool tripped = false;
  for (int i = 0; i < 2000 && !tripped; i++) {
    tripped = m.Update(50.0f - 0.004f * i, 50.0f - 0.004f * i, 52.0f, 1.0f, true);
  }
  BOOST_TEST(tripped);
  int held = 0;
  while (m.Update(42.0f, 42.0f, 52.0f, 1.0f, false) && held < 200000) { held++; }
  BOOST_TEST(held > 115000);
  BOOST_TEST(held < 125000);
}

BOOST_AUTO_TEST_CASE(MotorThermalMonitorStuckSensorWithCoupling) {
  // A stuck sensor on a motor with coupling 0.7: the correction makes the
  // corrected value rise with the estimate, but the sensor itself shows
  // none of its 70 % share, so it trips.
  MotorThermalParams p;
  ThermistorMonitor m;
  m.Configure(kDt, p);
  for (int i = 0; i < 30000; i++) { m.Update(40.0f, 40.0f, 42.0f, 0.7f, true); }
  bool tripped = false;
  for (int i = 0; i < 40000 && !tripped; i++) {
    const float rise = 0.001f * i;                    // estimate 1 C/s
    tripped = m.Update(40.0f, 40.0f + 0.3f * rise, 42.0f + rise, 0.7f, true);
  }
  BOOST_TEST(tripped);
  // The same motor with a working sensor showing its 70 % share: no trip.
  ThermistorMonitor m2;
  m2.Configure(kDt, p);
  for (int i = 0; i < 30000; i++) { m2.Update(40.0f, 40.0f, 42.0f, 0.7f, true); }
  for (int i = 0; i < 40000; i++) {
    const float rise = 0.001f * i;
    BOOST_TEST(!m2.Update(40.0f + 0.7f * rise, 40.0f + rise, 42.0f + rise, 0.7f, true));
  }
  // A sensor mostly on the housing (coupling 0.4) shows no fast rise: the
  // follow check does not apply, only the lead check (here 25 C).
  ThermistorMonitor m3;
  m3.Configure(kDt, p);
  for (int i = 0; i < 30000; i++) { m3.Update(40.0f, 40.0f, 42.0f, 0.4f, true); }
  bool tripped3 = false;
  for (int i = 0; i < 20000 && !tripped3; i++) {
    const float rise = 0.001f * i;
    tripped3 = m3.Update(40.0f, 40.0f + 0.6f * rise, 42.0f + rise, 0.4f, true);
  }
  BOOST_TEST(!tripped3);
  BOOST_TEST(m3.Update(40.0f, 45.0f, 71.0f, 0.4f, true));
}

BOOST_AUTO_TEST_CASE(MotorThermalMonitorLeadWarmupAndConfigure) {
  MotorThermalParams p;
  ThermistorMonitor m2;
  m2.Configure(kDt, p);
  for (int i = 0; i < 30000; i++) { m2.Update(30.0f, 30.0f, 40.0f, 1.0f, false); }
  // A 30 C lead of the estimate over the corrected thermistor: trips at once.
  BOOST_TEST(m2.Update(30.0f, 30.0f, 61.0f, 1.0f, false));
  // A reconfiguration keeps the fallback (any config write reconfigures).
  m2.Configure(kDt, p);
  BOOST_TEST(m2.Update(30.0f, 30.0f, 61.0f, 1.0f, false));
  // A dead (disabled) thermistor reads 0: the lead trips it too, but only
  // after the 2 s warm-up (the servo bypasses the monitor for a thermistor
  // disabled in config).
  ThermistorMonitor m3;
  m3.Configure(kDt, p);
  for (int i = 0; i < 1999; i++) { BOOST_TEST(!m3.Update(0.0f, 0.0f, 40.0f, 1.0f, false)); }
  BOOST_TEST(!m3.Update(30.0f, 30.0f, 32.0f, 1.0f, false));     // the last warm-up step
  BOOST_TEST(m3.Update(0.0f, 0.0f, 40.0f, 1.0f, false));
  // Non-finite inputs never trip (the thermistor alone protects).
  ThermistorMonitor m4;
  m4.Configure(kDt, p);
  BOOST_TEST(!m4.Update(kNaN, kNaN, 90.0f, 1.0f, true));
}

BOOST_AUTO_TEST_CASE(MotorThermalOffsetRules) {
  // Normally: the corrected thermistor (never below the reading).
  BOOST_TEST(MotorThermalOffset(30.0f, 36.0f, 60.0f, false) == 6.0f);
  BOOST_TEST(MotorThermalOffset(30.0f, 30.0f, 60.0f, false) == 0.0f);
  // Fallback: the hotter of the corrected thermistor and the estimate.
  BOOST_TEST(MotorThermalOffset(30.0f, 36.0f, 60.0f, true) == 30.0f);
  BOOST_TEST(MotorThermalOffset(30.0f, 36.0f, 20.0f, true) == 6.0f);
  // servo.enable_motor_temperature 0: the thermistor reads 0, the estimate
  // alone protects in fallback.
  BOOST_TEST(MotorThermalOffset(0.0f, 0.0f, 40.0f, true) == 40.0f);
  // Non-finite: 0.
  BOOST_TEST(MotorThermalOffset(kNaN, kNaN, 40.0f, true) == 0.0f);
  BOOST_TEST(MotorThermalOffset(30.0f, 30.0f, kNaN, true) == 0.0f);
}
