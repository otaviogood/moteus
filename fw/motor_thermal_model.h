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
#include <cstdint>

#include "mjlib/base/visitor.h"

/// Fork-specific: a winding temperature estimate for motors whose
/// thermistor cannot be trusted to follow the copper (placed on the
/// housing, detached, intermittent, or dead).  Protection uses the hotter
/// of the estimate and the motor thermistor: a faulty thermistor reads
/// low, so the estimate covers it, and a working one is never ignored.
///
/// A two-node model of the winding's rise above the board's FET
/// thermistor (clean and reliable on every board).  The winding (C1)
/// gains the copper loss P = 1.5 R(T) (Id^2 + Iq^2) and passes heat
/// through R1 to a slow node (C2: the motor body, which the board may not
/// see), which loses it through R2 to the FET reading:
///
///   C1 d(r1)/dt = P - (r1 - r2) / R1
///   C2 d(r2)/dt = (r1 - r2) / R1 - r2 / R2,    estimate = Tfet + r1
///
/// With C2 = 0 the slow node drops out (one node: C1 through R1).  The
/// FET reading supplies ambient and the heating the board does see; the
/// model supplies the rest.  Since the states are rises, not
/// temperatures, the estimate needs no seeding: it equals the FET reading
/// from the first one (at power-up, on a mode change).  It follows the
/// FET's own changes at once instead of with the winding's lag: higher
/// while the motor warms, the safe direction.
///
/// It runs from the 1 ms main-loop poll, never from the PWM interrupt:
/// the winding's fastest thermal time constant is ~10 s.  The interrupt's
/// derate and fault checks see it only through their thresholds, which
/// the poll lowers by the estimate's lead over the thermistor.

namespace moteus {

struct MotorThermalConfig {
  // A plain integer, not an enum: an enum field's name table and schema
  // cost ~0.6 kB of flash.
  //   0 (kOff): the motor thermistor alone (stock behavior).
  //   1 (kEstimate): protect on the hotter of the motor thermistor and
  //     the FET-based estimate.  For a dead thermistor (or one that reads
  //     high), set servo.enable_motor_temperature 0: it then reads 0 and
  //     the estimate alone protects.
  static constexpr int8_t kOff = 0;
  static constexpr int8_t kEstimate = 1;

  int8_t mode = kOff;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(mode));
  }
};

/// The model's parameters: compiled in, not config, because each config
/// field costs ~400 bytes of serialization code.  A conservative fit to
/// the humanoid3 robot (2026-10-03; 8 joints at 4 A for 10 min, 24 joints
/// at 10 A for 5 s), under-prediction weighted 10x: at most 1.3 C below a
/// working thermistor in any run, typically 2-5 C above it, up to 15 C
/// above at the end of long holds on boards well coupled to their motor.
/// Larger resistances and smaller capacities give a hotter estimate.
struct MotorThermalParams {
  float winding_J_per_C = 13.4f;
  float winding_to_slow_C_per_W = 1.08f;
  float slow_J_per_C = 77.0f;           // 0: no slow node
  float slow_to_fet_C_per_W = 0.57f;
  // The calibrated motor.resistance_ohm read 2-15 % below the resistance
  // measured from current sweeps on the robot.
  float resistance_scale = 1.1f;
  float resistance_tempco = 0.00393f;   // copper, per C above 25 C
};

class MotorThermalModel {
 public:
  /// @param resistance_ohm the calibrated phase resistance (0 if the
  /// motor is uncalibrated, which turns the heat input off)
  /// @param dt_s the update period
  void Configure(float resistance_ohm, float dt_s,
                 const MotorThermalParams& p = {}) {
    // P = 1.5 R I^2: d-q currents are amplitude invariant.
    heat_W_per_A2_ = resistance_ohm > 0.0f ?
        1.5f * resistance_ohm * p.resistance_scale : 0.0f;
    tempco_ = p.resistance_tempco;
    // Explicit Euler: every rate is far below 1 / dt at 1 ms.
    k1_heat_ = dt_s / p.winding_J_per_C;
    k1_link_ = k1_heat_ / p.winding_to_slow_C_per_W;
    const bool slow = p.slow_J_per_C > 0.0f;
    k2_link_ = slow ?
        dt_s / (p.slow_J_per_C * p.winding_to_slow_C_per_W) : 0.0f;
    k2_loss_ = slow ?
        dt_s / (p.slow_J_per_C * p.slow_to_fet_C_per_W) : 0.0f;
    if (!slow) { slow_rise_C_ = 0.0f; }
  }

  void Reset() { rise_C_ = slow_rise_C_ = 0.0f; }

  /// Advance one step.  @param i_squared_A2 Id^2 + Iq^2 (0 while the
  /// bridge is off), @param fet_C the FET temperature.  Returns the
  /// winding estimate (NaN while the FET reading is not finite; the rises
  /// keep integrating).
  float Update(float i_squared_A2, float fet_C) {
    const float winding_C =
        (std::isfinite(fet_C) ? fet_C : 25.0f) + rise_C_;
    const float power_W = heat_W_per_A2_ *
        (1.0f + tempco_ * (winding_C - 25.0f)) * i_squared_A2;
    const float link = rise_C_ - slow_rise_C_;
    rise_C_ += k1_heat_ * power_W - k1_link_ * link;
    slow_rise_C_ += k2_link_ * link - k2_loss_ * slow_rise_C_;
    return fet_C + rise_C_;
  }

  float rise_C() const { return rise_C_; }
  float slow_rise_C() const { return slow_rise_C_; }

 private:
  float heat_W_per_A2_ = 0.0f;
  float tempco_ = 0.0f;
  float k1_heat_ = 0.0f;
  float k1_link_ = 0.0f;
  float k2_link_ = 0.0f;
  float k2_loss_ = 0.0f;
  float rise_C_ = 0.0f;
  float slow_rise_C_ = 0.0f;
};

/// How far the protection thresholds move: the estimate's lead over the
/// motor thermistor reading, never negative (the hotter of the two).
/// Non-finite inputs give 0 (protect on the thermistor alone).
inline float MotorThermalOffset(float estimate_C, float thermistor_C) {
  const float lead = estimate_C - thermistor_C;
  return lead > 0.0f ? lead : 0.0f;   // false for NaN
}

}
