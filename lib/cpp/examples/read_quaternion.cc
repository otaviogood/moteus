// Copyright 2023 mjbots Robotic Systems, LLC.  info@mjbots.com

#include <iostream>
#include <cmath>
#include <thread>
#include <chrono>

#include "mjbots/moteus/moteus.h"

using namespace mjbots::moteus;

// Helper function to convert uint16_t to float16 format
float float16_to_float32(uint16_t value) {
  // Extract sign, exponent, and mantissa
  const int sign = ((value & 0x8000) >> 15);
  const int exponent = ((value & 0x7C00) >> 10);
  const int mantissa = (value & 0x03FF);

  // Handle special cases
  if (exponent == 0) {
    // Subnormal or zero
    if (mantissa == 0) {
      return sign ? -0.0f : 0.0f;
    } else {
      // Subnormal - normalized representation
      return (sign ? -1.0f : 1.0f) * (mantissa / 1024.0f) * std::pow(2.0f, -14);
    }
  } else if (exponent == 31) {
    // Infinity or NaN
    if (mantissa == 0) {
      return sign ? -INFINITY : INFINITY;
    } else {
      return NAN;
    }
  } else {
    // Normalized value
    return (sign ? -1.0f : 1.0f) * (1.0f + mantissa / 1024.0f) * std::pow(2.0f, exponent - 15);
  }
}

int main(int argc, char** argv) {
  // Process command line arguments
  Controller::DefaultArgProcess(argc, argv);

  // Create a controller, default to ID 1
  Controller c;

  // Set up a query to read quaternion values
  Query::Format qf;

  // Clear the default query elements
  qf.mode = Resolution::kIgnore;
  qf.position = Resolution::kIgnore;
  qf.velocity = Resolution::kIgnore;
  qf.torque = Resolution::kIgnore;
  qf.voltage = Resolution::kIgnore;
  qf.temperature = Resolution::kIgnore;
  qf.fault = Resolution::kIgnore;

  // Set up the quaternion values to query
  qf.extra[0].register_number = Register::kAux2QuaternionX;
  qf.extra[0].resolution = Resolution::kInt16;
  qf.extra[1].register_number = Register::kAux2QuaternionY;
  qf.extra[1].resolution = Resolution::kInt16;
  qf.extra[2].register_number = Register::kAux2QuaternionZ;
  qf.extra[2].resolution = Resolution::kInt16;

  std::cout << "Reading quaternion values from LSM6DSV16X IMU on Aux2. Press Ctrl+C to exit." << std::endl;
  std::cout << std::endl;

  try {
    while (true) {
      // Query the controller
      auto maybe_result = c.ReadQuaternion();

      if (!maybe_result) {
        std::cerr << "Failed to get quaternion data" << std::endl;
        continue;
      }

      // Extract the quaternion values
      const auto& values = maybe_result->values;

      // Find the quaternion values in the result
      int16_t quat_x = 0, quat_y = 0, quat_z = 0;

      for (int i = 0; i < Query::kMaxExtra; i++) {
        const auto& value = values.extra[i];
        if (value.register_number == Register::kAux2QuaternionX) {
          quat_x = static_cast<int16_t>(value.value);
        } else if (value.register_number == Register::kAux2QuaternionY) {
          quat_y = static_cast<int16_t>(value.value);
        } else if (value.register_number == Register::kAux2QuaternionZ) {
          quat_z = static_cast<int16_t>(value.value);
        }
      }

      // Convert to float16
      float x = float16_to_float32(quat_x);
      float y = float16_to_float32(quat_y);
      float z = float16_to_float32(quat_z);

      // Calculate w component: In a proper quaternion, x² + y² + z² + w² = 1
      // So w = sqrt(1 - (x² + y² + z²))
      float w = std::sqrt(1.0f - (x*x + y*y + z*z));

      // Display the quaternion values
      std::cout << "Quaternion: [" 
                << w << ", " 
                << x << ", " 
                << y << ", " 
                << z << "]" << std::endl;

      // Sleep for a bit
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}