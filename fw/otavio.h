// This file is for "bit banging" printfs as serial data to the "otavio" pin.
// Hook up the Saleae logic analyzer to the otavio pin and set it to
// 115200 baud serial, 1 start , 1 stop, no parity. Then you can get
// very realtime printfs to time and debug the firmware.

#pragma once

#include "mbed.h"
#include "fw/moteus_hw.h"

namespace moteus {

inline void CpuDelay(uint32_t cycles) {
  const uint32_t start_cycle = DWT->CYCCNT;
  const uint32_t target_cycle = start_cycle + cycles;

  // // Handle potential counter overflow, although unlikely for short delays
  // if (target_cycle < start_cycle) {
  //   // Wait for the counter to wrap around
  //   while (DWT->CYCCNT > start_cycle) {
  //     __NOP(); // Small delay to prevent hard spinning
  //   }
  // }
  // Wait until the target cycle count is reached
  while (DWT->CYCCNT < target_cycle) {
     __NOP(); // Small delay to prevent hard spinning
  }
}

inline void SendDebug(const char* str) {
  // At 115200 baud, calibrate CPU_CYCLES to match your CPU frequency
  // For example, at 170MHz, you'd need ~145 cycles for 8.68us
  constexpr uint32_t CPU_CYCLES = 145*5;//10; // Why do i have to multiply by 10?

  // Temporarily disable interrupts for bit-banging
  uint32_t primask = __get_PRIMASK();
  __disable_irq();

  DigitalOut db2(g_hw_pins.debug2, 1);
  // DigitalOut db2(g_hw_pins.otavio_pin, 1);

  while (*str) {
    // Start bit (0)
    db2.write(0);
    CpuDelay(CPU_CYCLES);

    // Data bits (LSB first)
    for (int i = 0; i < 8; i++) {
      db2.write((*str >> i) & 0x01);
      CpuDelay(CPU_CYCLES);
    }

    // Stop bit (1)
    db2.write(1);
    CpuDelay(CPU_CYCLES);

    str++;
  }

  // Return to idle state
  db2.write(1);

  // Re-enable interrupts
  if (!primask) {
    __enable_irq();
  }
}

template<typename... Args>
inline void SendDebugf(const char* format, Args... args) {
    // Buffer size should be adjusted based on your needs
    char buffer[128];
    snprintf(buffer, sizeof(buffer), format, args...);
    SendDebug(buffer);
}

}  // namespace moteus
