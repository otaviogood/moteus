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

#include "fw/imu_fusion.h"

namespace moteus {

/// Owner tag for ClaimTimer(): pointer identity, so one object.
inline constexpr char kFusionTimerOwner[] = "fusion";
inline constexpr char kAux1TimerOwner[] = "aux1";
inline constexpr char kAux2TimerOwner[] = "aux2";

/// The single static FusionStorage block (docs §5.8), claimed by
/// whichever aux port runs the fusion.  owner: 1 = aux1, 2 = aux2.
/// Returns nullptr if another owner holds it.
FusionStorage* ClaimFusionStorage(uint8_t owner);
void ReleaseFusionStorage(uint8_t owner);
uint8_t FusionStorageOwner();

/// Hardware timer ownership (docs §5.4).  TIM3 is shared between aux
/// pin modes (PWM output, hardware quadrature), BiSS-C and the fusion
/// time base / FDCAN timestamp counter.  Claim returns false if the
/// timer is held by a different owner string (pointer identity).
bool ClaimTimer(TIM_TypeDef* timer, const char* owner);
void ReleaseTimer(TIM_TypeDef* timer, const char* owner);
const char* TimerOwner(TIM_TypeDef* timer);

/// Start TIM3 as the free-running 16-bit, 4 us tick counter used for
/// fusion stamps and FDCAN RX timestamps.
void StartFusionTimer();

}
