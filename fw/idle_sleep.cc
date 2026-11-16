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

#include "mbed.h"

#include "fw/measured_hw_rev.h"
#include "fw/moteus_hw.h"

namespace moteus {

namespace {
void Drive(PinName pin, int level) {
  if (pin == NC) { return; }
  // At its level from the first instant: an output that starts low
  // would put a dominant glitch on the CAN bus.
  gpio_t gpio;
  gpio_init_out_ex(&gpio, pin, level);
}

void DriveHigh(PinName pin) { Drive(pin, 1); }

constexpr uint32_t kCyclesPerMs = 170000;

// True when no enabled I2C peripheral is in a transfer.  The control
// interrupt steps each transfer one byte at a time (fw/stm32_i2c.h); a
// transfer stopped in the middle holds the clock (or the device holds
// the data line) low through the whole sleep, and current flows in the
// pull-up the whole time.
bool I2cIdle() {
  struct Instance {
    I2C_TypeDef* i2c;
    volatile uint32_t* enable_reg;
    uint32_t enable_bit;
  };
  const Instance instances[] = {
    { I2C1, &RCC->APB1ENR1, RCC_APB1ENR1_I2C1EN },
    { I2C2, &RCC->APB1ENR1, RCC_APB1ENR1_I2C2EN },
    { I2C3, &RCC->APB1ENR1, RCC_APB1ENR1_I2C3EN },
    { I2C4, &RCC->APB1ENR2, RCC_APB1ENR2_I2C4EN },
  };
  for (const auto& instance : instances) {
    if ((*instance.enable_reg & instance.enable_bit) == 0) { continue; }
    if ((instance.i2c->CR1 & I2C_CR1_PE) == 0) { continue; }
    if ((instance.i2c->ISR & I2C_ISR_BUSY) != 0 ||
        (instance.i2c->CR2 & I2C_CR2_START) != 0) {
      return false;
    }
  }
  return true;
}
}

void EnterIdleSleep(uint8_t max_seconds, uint8_t bench_flags) {
  // Stop between two I2C transfers: interrupts masked only once every
  // bus is idle, so the control interrupt can finish a transfer it has
  // started.  A transfer of the IMU's takes about half a millisecond;
  // after 2000 tries, 5 ms or more (a wedged bus), the board sleeps
  // anyway.  Counted in tries, not cycles: a profiling build's control
  // interrupt resets the cycle counter.
  for (int tries = 0; ; tries++) {
    __disable_irq();
    if (I2cIdle() || tries >= 2000) { break; }
    __enable_irq();
    for (volatile int i = 0; i < 100; i++) {}
  }

  // The gate driver off and the control interrupt stopped.
  MoteusEnsureOff();

  // Nothing but the CAN edge may end the sleep.  With interrupts
  // masked, a pending enabled interrupt still ends WFI; its handler
  // never runs, the reset below does.
  for (auto& reg : NVIC->ICER) { reg = 0xffffffffu; }
  SysTick->CTRL = 0;
  SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk;

  if (g_otavio_flags & 1) {
    // A sensor board: its LEDs are lit when high.  Only yellow
    // (debug_led2) lit: green is "running", red "error".  These boards
    // have red and green the other way round from the pin names: red
    // is power_led, green debug_led1 (see "d led" in board_debug.cc).
    Drive(g_hw_pins.power_led, 0);
    Drive(g_hw_pins.debug_led1, 0);
    Drive(g_hw_pins.debug_led2, 1);
  } else {
    // A motor board: its LEDs are lit when low.  All off.
    Drive(g_hw_pins.power_led, 1);
    Drive(g_hw_pins.debug_led1, 1);
    Drive(g_hw_pins.debug_led2, 1);
  }

  // The CAN transmit pin idle (recessive) whatever the controller was
  // doing when its clock stops, then the controller itself off.
  DriveHigh(g_hw_pins.can_td);
  RCC->APB1RSTR1 |= RCC_APB1RSTR1_FDCANRST;

  // If the wait above gave up, a reset at least makes the processor
  // let go of the I2C clock line.
  RCC->APB1RSTR1 |= RCC_APB1RSTR1_I2C1RST | RCC_APB1RSTR1_I2C2RST |
                    RCC_APB1RSTR1_I2C3RST;
  RCC->APB1RSTR2 |= RCC_APB1RSTR2_I2C4RST;

  // ADCs, DACs, op-amps and comparators draw current in Stop mode
  // while enabled; a reset turns them off.  (SYSCFG's reset covers the
  // op-amps and comparators.)
  RCC->AHB2RSTR |=
      RCC_AHB2RSTR_ADC12RST | RCC_AHB2RSTR_ADC345RST |
      RCC_AHB2RSTR_DAC1RST | RCC_AHB2RSTR_DAC2RST |
      RCC_AHB2RSTR_DAC3RST | RCC_AHB2RSTR_DAC4RST;
  RCC->APB2RSTR |= RCC_APB2RSTR_SYSCFGRST;
  RCC->APB2RSTR &= ~RCC_APB2RSTR_SYSCFGRST;
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

  // Wake on a falling edge of the CAN receive pin: the start of any
  // frame on the bus.
  const PinName rd = g_hw_pins.can_rd;
  const uint32_t port = STM_PORT(rd);
  const uint32_t pin = STM_PIN(rd);
  auto* const gpio = reinterpret_cast<GPIO_TypeDef*>(
      GPIOA_BASE + 0x400u * port);
  gpio->MODER &= ~(3u << (2 * pin));  // input
  const uint32_t field = 4 * (pin & 3);
  SYSCFG->EXTICR[pin >> 2] =
      (SYSCFG->EXTICR[pin >> 2] & ~(0xfu << field)) | (port << field);
  EXTI->IMR1 = 0;
  EXTI->IMR2 = 0;
  EXTI->RTSR1 = 0;
  EXTI->FTSR1 = 1u << pin;

  // Arm it once the bus has been idle for 2 ms (the end of the request
  // frame and anything it caused), or after 100 ms whatever the bus
  // does: a busy bus then wakes the board at once, as it should.
  {
    const uint32_t start = DWT->CYCCNT;
    uint32_t idle_since = start;
    for (;;) {
      const uint32_t now = DWT->CYCCNT;
      if ((gpio->IDR & (1u << pin)) == 0) { idle_since = now; }
      if (now - idle_since > 2 * kCyclesPerMs) { break; }
      if (now - start > 100 * kCyclesPerMs) { break; }
    }
  }
  EXTI->PR1 = 0xffffffffu;
  EXTI->PR2 = 0xffffffffu;
  if ((bench_flags & kIdleSleepBenchNoBusWake) == 0) {
    EXTI->IMR1 = 1u << pin;
  }
  const IRQn_Type irq =
      (pin >= 10) ? EXTI15_10_IRQn :
      (pin >= 5) ? EXTI9_5_IRQn :
      static_cast<IRQn_Type>(EXTI0_IRQn + pin);
  for (auto& reg : NVIC->ICPR) { reg = 0xffffffffu; }
  NVIC_EnableIRQ(irq);

  if (max_seconds != 0) {
    // The independent watchdog runs from its own 32 kHz oscillator
    // through Stop mode and resets the processor when it expires.
    // /256 is 8 ms per count, 32.8 s at most.
    const uint32_t seconds =
        max_seconds > kIdleSleepMaxSeconds ? kIdleSleepMaxSeconds : max_seconds;
    IWDG->KR = 0xcccc;  // start
    IWDG->KR = 0x5555;  // unlock
    IWDG->PR = 6;
    IWDG->RLR = seconds * 125 - 1;
    for (uint32_t i = 0; i < 2000000 && IWDG->SR != 0; i++) {}
    IWDG->KR = 0xaaaa;  // reload
  }

  // Stop 1: every clock off, low-power regulator, RAM kept.
  RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;
  PWR->CR1 = (PWR->CR1 & ~PWR_CR1_LPMS) | PWR_CR1_LPMS_STOP1;
  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
  __DSB();
  __WFI();
  __ISB();

  NVIC_SystemReset();
  for (;;) {}
}

}  // namespace moteus
