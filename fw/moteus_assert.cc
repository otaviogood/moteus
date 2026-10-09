// Copyright 2023 mjbots Robotic Systems, LLC.  info@mjbots.com
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

#include <bits/functexcept.h>

#include "mbed_assert.h"
#include "mbed.h"

#include "hal/gpio_api.h"

#include "fw/moteus_hw.h"

namespace mjlib {
namespace base {

void assertion_failed(const char* expression, const char* filename, int line) {
  mbed_assert_internal(expression, filename, line);
}

}
}

// Flash is tight.  Two pieces of the C/C++ runtime that can only ever
// halt the controller are replaced here with direct halts, so that the
// code they would format or unwind with is not linked at all.
//
// 1. newlib's assert(): the only users are the asserts inside newlib's
//    own dtoa/strtod (reached from snprintf("%f") and strtof).  Its
//    stock __assert_func reports with fiprintf(stderr, ...), which
//    links a second, integer-only printf engine (_vfiprintf_r, ~4.8 kB)
//    that nothing else in the firmware uses.  Reporting through mbed's
//    assert instead shares the vsnprintf engine the firmware already
//    carries, and ends in mbed_die like everything else.
extern "C" {
void __assert_func(const char* file, int line, const char* func,
                   const char* expr) {
  (void)func;
  mbed_assert_internal(expr, file, line);
}
}

// 2. libstdc++'s throw helpers, e.g. the bounds check in
//    std::string_view::substr.  The firmware is built with
//    -fno-exceptions, so nothing can catch: the stock helpers build a
//    std::logic_error (a heap-allocated std::string), throw, find no
//    handler, and terminate -- which keeps the whole exception runtime
//    linked (__cxa_throw, the personality routine, the ARM unwinder,
//    their tables, ~4 kB).  Halting directly loses nothing.  The full
//    hosted set from <bits/functexcept.h> is defined so a new use of
//    any of them never drags the library's versions back in; the
//    unreferenced ones are garbage collected.
namespace std {
namespace {
[[noreturn]] void Halt(const char* what) {
  mbed_assert_internal(what, "libstdc++", 0);
}
}

void __throw_bad_exception() { Halt("bad_exception"); }
void __throw_bad_alloc() { Halt("bad_alloc"); }
void __throw_bad_array_new_length() { Halt("bad_array_new_length"); }
void __throw_bad_cast() { Halt("bad_cast"); }
void __throw_bad_typeid() { Halt("bad_typeid"); }
void __throw_logic_error(const char* what) { Halt(what); }
void __throw_domain_error(const char* what) { Halt(what); }
void __throw_invalid_argument(const char* what) { Halt(what); }
void __throw_length_error(const char* what) { Halt(what); }
void __throw_out_of_range(const char* what) { Halt(what); }
void __throw_out_of_range_fmt(const char* fmt, ...) { Halt(fmt); }
void __throw_runtime_error(const char* what) { Halt(what); }
void __throw_range_error(const char* what) { Halt(what); }
void __throw_overflow_error(const char* what) { Halt(what); }
void __throw_underflow_error(const char* what) { Halt(what); }
void __throw_ios_failure(const char* what) { Halt(what); }
void __throw_ios_failure(const char* what, int) { Halt(what); }
void __throw_system_error(int) { Halt("system_error"); }
void __throw_future_error(int) { Halt("future_error"); }
void __throw_bad_function_call() { Halt("bad_function_call"); }
}

extern "C" {
void mbed_die(void) {
  // We want to ensure the motor controller is disabled and flash an
  // LED which exists.
  moteus::MoteusEnsureOff();

  // If we got here before g_hw_pins was populated (e.g. an assertion
  // fired during family detection), debug_led1 is NC.  Calling
  // gpio_init_out with NC leaves the gpio_t's mask/reg_set/reg_clr
  // uninitialized, and the subsequent gpio_write would dereference a
  // garbage stack pointer.  Just spin in that case.
  const auto led_pin = moteus::g_hw_pins.debug_led1;
  if (led_pin == NC) {
    for (;;) {}
  }

  gpio_t led;
  gpio_init_out(&led, led_pin);

  // Now flash an actual LED.
  for (;;) {
    gpio_write(&led, 0);
    wait_ms(200);
    gpio_write(&led, 1);
    wait_ms(200);
  }
}
}
