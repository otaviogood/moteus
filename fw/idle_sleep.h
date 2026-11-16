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

#include <cstddef>
#include <cstdint>

/// Fork-specific, BENCH PROTOTYPE: put the board to sleep on command
/// (docs/protocol/registers.md, "Idle sleep").
///
/// A frame whose first three bytes are kIdleSleepRequest,
/// kIdleSleepFormat, max_seconds asks the board to sleep.  It is meant
/// to be broadcast (11-bit ID 0x07F) by the host when a run is over.
/// There is no reply.
///
/// The board obeys only when the motor is stopped (mode 0); in any
/// other mode, a fault included, the frame is ignored.  Asleep, the
/// processor's clocks are off (STM32 Stop mode), the gate driver is
/// disabled and the LEDs are off, except that a sensor board lights its
/// yellow LED to show that it sleeps.  The board waits for the I2C bus
/// to be idle (at most 5 ms) before it stops.
///
/// The next edge on the CAN bus, from any frame to any board, wakes it,
/// and it restarts exactly as from power-on: about 40 ms until it
/// answers, a few hundred ms until the orientation is valid.  Whatever
/// a power cycle loses, this loses.  The frame that woke it is not
/// received (a sender with nobody else on the bus keeps retrying it
/// until the board is up and acknowledges it).
///
/// max_seconds 1..32 is for bench tests: the board also wakes by
/// itself after that long (watchdog), so a board whose CAN wake does
/// not work comes back.  0 sleeps until the bus wakes it.
///
/// An optional fourth byte holds bench flags.  kIdleSleepBenchNoBusWake:
/// the bus does not wake the board, only the watchdog does (to tell a
/// sleep that does not hold from a bus that keeps waking it).  It is
/// refused with max_seconds 0.
namespace moteus {

constexpr uint8_t kIdleSleepRequest = 0x68;  // unused by mjlib
constexpr uint8_t kIdleSleepFormat = 1;
constexpr uint8_t kIdleSleepMaxSeconds = 32;
constexpr uint8_t kIdleSleepBenchNoBusWake = 0x01;

/// True if the frame starts with a sleep request in a format this
/// firmware knows; *max_seconds is its third byte, *bench_flags its
/// fourth (0 if the frame ends before it).
inline bool ParseIdleSleepRequest(const uint8_t* data, size_t size,
                                  uint8_t* max_seconds,
                                  uint8_t* bench_flags) {
  if (size < 3 || data[0] != kIdleSleepRequest ||
      data[1] != kIdleSleepFormat) {
    return false;
  }
  const uint8_t flags = size >= 4 ? data[3] : 0;
  // Never a sleep that nothing can end.
  if ((flags & kIdleSleepBenchNoBusWake) != 0 && data[2] == 0) {
    return false;
  }
  *max_seconds = data[2];
  *bench_flags = flags;
  return true;
}

/// Sleep now; returns only through a processor reset.  The caller has
/// checked that the motor is stopped.  Target only (fw/idle_sleep.cc).
[[noreturn]] void EnterIdleSleep(uint8_t max_seconds, uint8_t bench_flags);

}  // namespace moteus
