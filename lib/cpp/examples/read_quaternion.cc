// Copyright 2023 mjbots Robotic Systems, LLC.  info@mjbots.com

// Reads the orientation of a LSM6DSV16X IMU on the Aux2 port
// (aux2.i2c.devices.0.type 3).  Registers 0x06d-0x06f carry a 48-bit
// "smallest three" quaternion; see docs/protocol/registers.md.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <thread>

#include "moteus.h"

using namespace mjbots::moteus;

namespace {
// Three register words -> (w, x, y, z).  Returns false for the "no data"
// sentinel.
bool DecodeQuat48(uint16_t w0, uint16_t w1, uint16_t w2, double q[4]) {
  const uint64_t v = (static_cast<uint64_t>(w0) |
                      (static_cast<uint64_t>(w1) << 16) |
                      (static_cast<uint64_t>(w2) << 32)) &
                     ((1ull << 47) - 1);
  if (v == 0) { return false; }
  const int omitted = static_cast<int>((v >> 45) & 0x3);
  double sum = 0.0;
  for (int i = 0, j = 0; i < 4; i++) {
    if (i == omitted) { continue; }
    const double code = static_cast<double>((v >> (15 * j)) & 0x7fff);
    q[i] = (code / 32767.0 * 2.0 - 1.0) / std::sqrt(2.0);
    sum += q[i] * q[i];
    j++;
  }
  q[omitted] = std::sqrt(std::max(0.0, 1.0 - sum));
  return true;
}
}  // namespace

int main(int argc, char** argv) {
  Controller::DefaultArgProcess(argc, argv);

  Controller c;

  std::cout << "Reading the Aux2 IMU orientation. Press Ctrl+C to exit."
            << std::endl;

  while (true) {
    const auto maybe_result = c.ReadQuaternion();
    if (!maybe_result) {
      std::cerr << "Failed to get quaternion data" << std::endl;
      continue;
    }

    uint16_t words[3] = {};
    for (const auto& value : maybe_result->values.extra) {
      const int index = value.register_number - Register::kAux2QuaternionX;
      if (index >= 0 && index < 3) {
        words[index] = static_cast<uint16_t>(static_cast<int16_t>(value.value));
      }
    }

    double q[4] = {};
    if (DecodeQuat48(words[0], words[1], words[2], q)) {
      std::cout << "wxyz = [" << q[0] << ", " << q[1] << ", " << q[2]
                << ", " << q[3] << "]  toggle " << (words[2] >> 15)
                << std::endl;
    } else {
      std::cout << "no data (IMU not configured or filter warming up)"
                << std::endl;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  return 0;
}
