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

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "mjlib/base/visitor.h"

/// Fork-specific motor thermal protection for thermistors that cannot be
/// trusted to follow the copper (partly on the housing, slow, intermittent,
/// dead; humanoid3 2026-10).  Three pieces, all run from the 1 ms main-loop
/// poll, never from the PWM interrupt; the interrupt's derate and fault
/// checks see only two thresholds that the poll lowers by the offset.
///
/// 1. A per-motor correction of the thermistor.  A sensor that sits partly
///    on the housing sees only a fraction `thermistor_coupling` of the
///    winding's rise over the housing, so that rise, modelled from the
///    copper loss (one node: Cw, Rwh), is added back:
///      corrected = thermistor + (1 - coupling) * rise_w
///    Additive, so it is right while cooling too (a gain would keep reading
///    a warm housing as a hot winding).  Measured per motor against the
///    winding's copper resistance; 1.0 (the default) changes nothing.
/// 2. An estimate of the winding from the board's FET thermistor
///    (MotorThermalModel: a two-node rise over the FET), compiled-in
///    conservative parameters.  Used only as a fallback.
/// 3. A plausibility monitor: when the estimate says the winding is heating
///    and the thermistor itself does not show its share of it (its coupling
///    times the rise), or the thermistor drops while the motor is driven,
///    or the estimate exceeds the corrected thermistor by a wide margin,
///    protection switches to the hotter of the corrected thermistor and the
///    estimate: for a hold time, and for as long as the estimate's lead stays
///    large.  Otherwise the model's own errors never derate a healthy sensor.
///    A thermistor disabled in config (servo.enable_motor_temperature 0)
///    selects the estimate outright.

namespace moteus {

struct MotorThermalConfig {
  // A plain integer, not an enum: an enum field's name table and schema
  // cost ~0.6 kB of flash.
  //   0 (kOff): the motor thermistor alone (stock behavior).
  //   1 (kOn): the corrected thermistor, with the FET-based estimate as
  //     a fallback when the thermistor is implausible.  For a dead
  //     thermistor set servo.enable_motor_temperature 0: it then reads 0,
  //     the monitor trips on the first heating, and the estimate protects.
  static constexpr int8_t kOff = 0;
  static constexpr int8_t kOn = 1;

  int8_t mode = kOff;
  // The fraction of the winding's rise over the housing this motor's
  // thermistor sees: 1 on the winding, lower when partly on the housing.
  // Clamped to [0, 1].
  float thermistor_coupling = 1.0f;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(mode));
    a->Visit(MJ_NVP(thermistor_coupling));
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

  // The winding's rise over the housing, for the thermistor correction
  // (bench copper-from-resistance and the 4 A robot runs, 2026-10-03).
  float winding_over_housing_J_per_C = 11.9f;
  float winding_over_housing_C_per_W = 1.26f;

  // The plausibility monitor.
  float monitor_window_s = 20.0f;      // the rises are over this time
  float monitor_min_rise_C = 4.0f;     // estimate rise that counts as heating
  float monitor_follow = 0.3f;         // thermistor must rise this fraction of its share
  float monitor_follow_min_coupling = 0.6f;  // below this the sensor is mostly on the
                                       // housing and shows no fast rise: no follow check
  float monitor_persist_s = 3.0f;      // ... for this long (thermistors lag 1-6 s)
  float monitor_drop_C = 4.0f;         // thermistor fall while driven that trips
  float monitor_lead_C = 25.0f;        // estimate - corrected thermistor that trips
  float monitor_lead_release_C = 15.0f;  // ... and below which a trip may release
  float monitor_hold_s = 120.0f;       // fallback stays on this long after a trip
  float monitor_warmup_s = 2.0f;       // no judgement this long after (re)start:
                                       // the thermistor reads 0 before its filter settles
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

/// The winding's rise over the housing: one node fed by the copper loss.
class WindingRise {
 public:
  void Configure(float dt_s, const MotorThermalParams& p = {}) {
    k_heat_ = dt_s / p.winding_over_housing_J_per_C;
    k_loss_ = k_heat_ / p.winding_over_housing_C_per_W;
  }
  void Reset() { rise_C_ = 0.0f; }
  float Update(float power_W) {
    rise_C_ += k_heat_ * power_W - k_loss_ * rise_C_;
    return rise_C_;
  }
  float rise_C() const { return rise_C_; }

 private:
  float k_heat_ = 0.0f;
  float k_loss_ = 0.0f;
  float rise_C_ = 0.0f;
};

/// Decides whether the thermistor can be trusted right now.
class ThermistorMonitor {
 public:
  void Configure(float dt_s, const MotorThermalParams& p = {}) {
    a_ = 1.0f - dt_s / p.monitor_window_s;
    min_rise_C_ = p.monitor_min_rise_C;
    follow_ = p.monitor_follow;
    follow_min_coupling_ = p.monitor_follow_min_coupling;
    drop_C_ = p.monitor_drop_C;
    lead_C_ = p.monitor_lead_C;
    lead_release_C_ = p.monitor_lead_release_C;
    persist_steps_ = static_cast<int32_t>(p.monitor_persist_s / dt_s);
    hold_steps_ = static_cast<int32_t>(p.monitor_hold_s / dt_s);
    warmup_steps_ = static_cast<int32_t>(p.monitor_warmup_s / dt_s);
    // A reconfiguration (any config write) keeps the hold, latch and
    // history; only the first configuration arms the warm-up.
    if (!primed_ && timer_ == 0) { warmup_ = warmup_steps_; }
  }
  void Reset() {
    timer_ = 0; lagging_ = 0; latched_ = false;
    thermistor_lp_ = estimate_lp_ = 0.0f; primed_ = false;
    warmup_ = warmup_steps_;
  }

  /// @param thermistor_C the filtered thermistor itself, @param corrected_C
  /// it plus the coupling correction, @param estimate_C the FET-based
  /// estimate, @param coupling the thermistor's share of the winding's
  /// rise, @param driven whether current flows.  Returns whether the
  /// fallback is active after this step.
  bool Update(float thermistor_C, float corrected_C, float estimate_C,
              float coupling, bool driven) {
    if (!std::isfinite(thermistor_C) || !std::isfinite(corrected_C) ||
        !std::isfinite(estimate_C)) {
      return active();
    }
    if (warmup_ > 0) {
      warmup_--;
      return false;
    }
    if (!primed_) {
      thermistor_lp_ = thermistor_C; estimate_lp_ = estimate_C; primed_ = true;
    }
    const float rise_t = thermistor_C - thermistor_lp_;
    const float rise_e = estimate_C - estimate_lp_;
    thermistor_lp_ += (1.0f - a_) * rise_t;
    estimate_lp_ += (1.0f - a_) * rise_e;
    // Not following: the estimate's 20 s rise says heating and the sensor
    // itself shows less than a fraction of its share of it (a sensor
    // partly on the housing sees `coupling` of the winding's rise), for
    // longer than a sensor's lag.  The correction is left out here so it
    // cannot make a stuck sensor look responsive.  A sensor mostly on the
    // housing shows no fast rise at all (robot: coupling 0.4-0.5 sensors
    // show ~0 in 20 s): only the lead and drop checks apply to it.
    if (coupling >= follow_min_coupling_ && rise_e > min_rise_C_ &&
        rise_t < follow_ * coupling * rise_e) {
      if (lagging_ < persist_steps_) { lagging_++; }
    } else {
      lagging_ = 0;
    }
    const bool not_following = lagging_ >= persist_steps_;
    const bool fell = driven && rise_t < -drop_C_ && rise_e >= 0.0f;
    const float lead = estimate_C - corrected_C;
    if (not_following || fell || lead > lead_C_) {
      timer_ = hold_steps_;
      latched_ = true;
    } else if (timer_ > 0) {
      timer_--;
    }
    // A trip releases only once the hold has passed AND the sensor agrees
    // with the estimate again; a sensor stuck below it stays covered.
    if (latched_ && timer_ == 0 && lead < lead_release_C_) {
      latched_ = false;
    }
    return active();
  }
  bool active() const { return timer_ > 0 || latched_; }

 private:
  float a_ = 1.0f;
  float min_rise_C_ = 4.0f, follow_ = 0.3f, drop_C_ = 4.0f, lead_C_ = 25.0f;
  float follow_min_coupling_ = 0.6f;
  float lead_release_C_ = 15.0f;
  int32_t persist_steps_ = 0;
  int32_t hold_steps_ = 0;
  int32_t warmup_steps_ = 0;
  int32_t timer_ = 0;
  int32_t lagging_ = 0;
  int32_t warmup_ = 0;
  bool latched_ = false;
  float thermistor_lp_ = 0.0f, estimate_lp_ = 0.0f;
  bool primed_ = false;
};

/// How far the protection thresholds move: the protection temperature's
/// lead over the thermistor reading, never negative.  Protection is the
/// corrected thermistor, or the hotter of it and the estimate while the
/// fallback is active.  Non-finite inputs give 0 (the thermistor alone).
inline float MotorThermalOffset(float thermistor_C, float corrected_C,
                                float estimate_C, bool fallback) {
  float protect = corrected_C;
  if (fallback && estimate_C > protect) { protect = estimate_C; }
  const float lead = protect - thermistor_C;
  return lead > 0.0f ? lead : 0.0f;   // false for NaN
}

}
