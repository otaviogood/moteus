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

#include "fw/imu_fusion_storage.h"

namespace moteus {

namespace {

FusionStorage g_fusion_storage;
uint8_t g_fusion_owner = 0;

struct TimerClaim {
  TIM_TypeDef* timer = nullptr;
  const char* owner = nullptr;
};
TimerClaim g_timer_claims[4] = {};

}  // namespace

FusionStorage* ClaimFusionStorage(uint8_t owner) {
  if (g_fusion_owner != 0 && g_fusion_owner != owner) { return nullptr; }
  g_fusion_owner = owner;
  return &g_fusion_storage;
}

void ReleaseFusionStorage(uint8_t owner) {
  if (g_fusion_owner == owner) { g_fusion_owner = 0; }
}

uint8_t FusionStorageOwner() { return g_fusion_owner; }

bool ClaimTimer(TIM_TypeDef* timer, const char* owner) {
  TimerClaim* free_slot = nullptr;
  for (auto& claim : g_timer_claims) {
    if (claim.timer == timer) {
      return claim.owner == owner;
    }
    if (claim.timer == nullptr && free_slot == nullptr) {
      free_slot = &claim;
    }
  }
  if (free_slot == nullptr) { return false; }
  free_slot->timer = timer;
  free_slot->owner = owner;
  return true;
}

void ReleaseTimer(TIM_TypeDef* timer, const char* owner) {
  for (auto& claim : g_timer_claims) {
    if (claim.timer == timer && claim.owner == owner) {
      claim = {};
    }
  }
}

const char* TimerOwner(TIM_TypeDef* timer) {
  for (const auto& claim : g_timer_claims) {
    if (claim.timer == timer) { return claim.owner; }
  }
  return nullptr;
}

void StartFusionTimer() {
  // TIM3 kernel clock is 170 MHz (PCLK1 * 2); the clock is enabled at
  // boot in moteus.cc.  4 us tick, free running, no interrupts.
  const uint32_t timer_clock = HAL_RCC_GetPCLK1Freq() * 2;
  TIM3->CR1 = 0;
  TIM3->DIER = 0;
  // A previous aux configuration may have left encoder or PWM mode
  // behind; SMCR encoder mode would clock CNT from the pins.
  TIM3->SMCR = 0;
  TIM3->CCER = 0;
  TIM3->CCMR1 = 0;
  TIM3->CCMR2 = 0;
  TIM3->PSC = (timer_clock / 250000u) - 1;
  TIM3->ARR = 0xffff;
  TIM3->EGR = TIM_EGR_UG;
  TIM3->CNT = 0;
  TIM3->CR1 = TIM_CR1_CEN;
}

}
