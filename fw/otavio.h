// This file is for "bit banging" printfs as serial data to the "otavio" pin.
// Hook up the Saleae logic analyzer to the otavio pin and set it to
// 115200 baud serial, 1 start , 1 stop, no parity. Then you can get
// very realtime printfs to time and debug the firmware.

#pragma once

#include "mbed.h"
#include "fw/moteus_hw.h"

namespace moteus {

inline void CpuDelay(uint32_t cycles) {
  for (volatile uint32_t i = 0; i < cycles; i++) {
    __NOP();
  }
}

inline void SendDebug(const char* str) {
  // At 115200 baud, calibrate CPU_CYCLES to match your CPU frequency
  // For example, at 170MHz, you'd need ~145 cycles for 8.68us
  constexpr uint32_t CPU_CYCLES = 145;

  DigitalOut db2(g_hw_pins.otavio_pin, 1);

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
}

template<typename... Args>
inline void SendDebugf(const char* format, Args... args) {
    // Buffer size should be adjusted based on your needs
    char buffer[128];
    snprintf(buffer, sizeof(buffer), format, args...);
    SendDebug(buffer);
}

}  // namespace moteus
