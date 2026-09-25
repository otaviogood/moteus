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

#include <cstdint>

#include "mbed.h"

#include "fw/measured_hw_rev.h"

/// Logic-analyzer timing markers, bench builds only:
///
///   tools/bazel build --config=target //:target --copt=-DMOTEUS_SCOPE_MARKERS
///
/// Each marker is one GPIOC BSRR write (a few cycles).  moteus r4.x
/// (hardware family 0) only, on its two debug pads (leave aux2 pins 2
/// and 3 unused).  DBG1 (PC14) always shows the control interrupt, ADC
/// sampled -> done.  DBG2 (PC15) shows one of, by console command:
///
///   `d mark phase`   kPhase: high between `d mark 1` and `d mark 0`
///                    (utils/imu_fusion_bench/scope_capture.py)
///   `d mark can`     kCanFrame: high for one CAN frame, from StartFrame
///                    until its command reaches the servo (or the frame
///                    ends if it carries none), notched LOW while the
///                    frame's fusion work runs (kFusionFrame: BeginFrame,
///                    the quaternion reply)
///                    (utils/imu_fusion_bench/scope_can_capture.py)
///   `d mark fusion`  all main-loop fusion work: kFusionFrame and the
///                    mailbox drain (kFusionDrain)
///
/// PC14/PC15 are low-speed pins (RM0440: 2 MHz, 30 pF), so their edges
/// lag the CAN pins by some 100 ns.  Without the define everything here
/// compiles to nothing.

namespace moteus {
namespace scope {

enum Marker : uint8_t {
  kIsr,
  kPhase,
  kCanFrame,
  kFusionFrame,
  kFusionDrain,
  kNumMarkers,
};

enum class Dbg2 : uint8_t {
  kPhase,
  kCan,
  kFusion,
};

#ifdef MOTEUS_SCOPE_MARKERS

constexpr uint32_t kDbg1 = 1u << 14;
constexpr uint32_t kDbg2 = 1u << 15;

/// The BSRR words for each marker's Set and Clear (0 = not shown).
inline uint32_t g_set[kNumMarkers] = {};
inline uint32_t g_clear[kNumMarkers] = {};

inline void Show(Marker m, uint32_t pin) {
  g_set[m] = pin;
  g_clear[m] = pin << 16;
}

inline void ShowInverted(Marker m, uint32_t pin) {
  g_set[m] = pin << 16;
  g_clear[m] = pin;
}

/// What DBG2 shows (the phase marker after Init()).
inline void SelectDbg2(Dbg2 what) {
  if (g_set[kIsr] == 0) { return; }
  for (int m = kPhase; m < kNumMarkers; m++) {
    g_set[m] = g_clear[m] = 0;
  }
  GPIOC->BSRR = kDbg2 << 16;
  switch (what) {
    case Dbg2::kPhase: {
      Show(kPhase, kDbg2);
      break;
    }
    case Dbg2::kCan: {
      Show(kCanFrame, kDbg2);
      ShowInverted(kFusionFrame, kDbg2);
      break;
    }
    case Dbg2::kFusion: {
      Show(kFusionFrame, kDbg2);
      Show(kFusionDrain, kDbg2);
      break;
    }
  }
}

/// Call once after the configuration is loaded.
inline void Init() {
  if (g_measured_hw_family != 0) { return; }
  __HAL_RCC_GPIOC_CLK_ENABLE();
  GPIO_InitTypeDef init = {};
  init.Pin = kDbg1 | kDbg2;
  init.Mode = GPIO_MODE_OUTPUT_PP;
  init.Pull = GPIO_NOPULL;
  init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOC, &init);
  GPIOC->BSRR = init.Pin << 16;
  Show(kIsr, kDbg1);
  SelectDbg2(Dbg2::kPhase);
}

inline void Set(Marker m) { GPIOC->BSRR = g_set[m]; }
inline void Clear(Marker m) { GPIOC->BSRR = g_clear[m]; }

#else

inline void Init() {}
inline void Set(Marker) {}
inline void Clear(Marker) {}
inline void SelectDbg2(Dbg2) {}

#endif

}  // namespace scope
}  // namespace moteus
