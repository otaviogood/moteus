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
#include <string_view>

#include "mbed.h"

#include "fw/ccm.h"
#include "fw/imu_fusion.h"
#include "fw/stm32_i2c.h"

/// ISR side of the LSM6DSV16X fusion mode (docs §5.1, §5.2): a
/// non-blocking init sequence, then a status/word read loop over the
/// chip FIFO.  Issues at most one I2C transaction per control cycle and
/// never waits.  Completed words go into the FusionMailbox with a TIM3
/// tick stamp; everything else is a sticky counter in FusionControl.

namespace moteus {

class Lsm6dsv16xFusionDriver {
 public:
  enum class State : uint8_t {
    kIdle = 0,
    kSelectMainBank = 1,
    kReset = 2,
    kWaitReset = 3,
    kConfig = 4,
    kReadWhoAmI = 5,
    kReadFreqFine = 6,
    kRunning = 7,
    kRetryWait = 8,
  };

  enum class Resync : uint8_t {
    kNone = 0,
    kI2cError = 1,
    kResetTimeout = 2,
    kWhoAmI = 3,
  };

  Lsm6dsv16xFusionDriver(Stm32I2c* i2c, uint8_t address,
                         FusionStorage* storage,
                         aux::ImuFusionStatus* status)
      : i2c_(i2c), address_(address), storage_(storage), status_(status) {
    Publish();
  }

  /// ISR, with the I2C peripheral idle.  Starts at most one
  /// transaction; returns true if one was started.
#if defined(TARGET_STM32G4)
  __attribute__((noinline))
#endif
  bool ISR_Start() MOTEUS_CCM_ATTRIBUTE {
    switch (state_) {
      case State::kIdle: {
        state_ = State::kSelectMainBank;
        return Write(kRegFuncCfgAccess, 0x00);
      }
      case State::kSelectMainBank: {
        // Transaction in flight (or lost); wait for completion.
        return false;
      }
      case State::kReset: {
        return Write(kRegCtrl3, 0x01);  // SW_RESET
      }
      case State::kWaitReset: {
        return Read(kRegCtrl3, 1);
      }
      case State::kConfig: {
        return Write(kConfig[config_step_].reg, kConfig[config_step_].value);
      }
      case State::kReadWhoAmI: {
        return Read(kRegWhoAmI, 1);
      }
      case State::kReadFreqFine: {
        return Read(kRegInternalFreqFine, 1);
      }
      case State::kRunning: {
        if (halt_) { return false; }
        if (remaining_words_ > 0) {
          return Read(kRegFifoDataOutTag, 7);
        }
        return Read(kRegFifoStatus1, 2);
      }
      case State::kRetryWait: {
        if (retry_wait_ > 0) {
          retry_wait_--;
          return false;
        }
        state_ = State::kIdle;
        return false;
      }
    }
    return false;
  }

  /// ISR, when the transaction started by ISR_Start() finished.
#if defined(TARGET_STM32G4)
  __attribute__((noinline))
#endif
  void ISR_Complete(bool ok) MOTEUS_CCM_ATTRIBUTE {
    if (!ok) {
      if (status_) { status_->i2c_errors++; }
      consecutive_errors_++;
      if (state_ == State::kRunning && remaining_words_ == 0 &&
          consecutive_errors_ < kErrorsBeforeResync) {
        // A failed FIFO status read has not consumed a sample, so it
        // can be retried.  A failed word read may have consumed a word
        // without advancing gyro_seq_; resync below rather than
        // silently integrating across an unknown gap.  In particular,
        // the arrival tracker cannot recover the lost rotation.
        Publish();
        return;
      }
      // A restart re-inits the chip and the fusion reconverges (>= 0.2 s)
      // anyway; the wait keeps a chip that stopped answering from being
      // retried every other control cycle forever.
      Restart(Resync::kI2cError, kI2cErrorRetryCycles);
      return;
    }
    consecutive_errors_ = 0;

    switch (state_) {
      case State::kIdle:
      case State::kRetryWait: {
        break;
      }
      case State::kSelectMainBank: {
        state_ = State::kReset;
        break;
      }
      case State::kReset: {
        state_ = State::kWaitReset;
        wait_count_ = 0;
        break;
      }
      case State::kWaitReset: {
        if ((buf_[0] & 0x01) == 0) {
          state_ = State::kConfig;
          config_step_ = 0;
        } else if (++wait_count_ > kResetTimeoutCycles) {
          Restart(Resync::kResetTimeout, kRetryCycles);
        }
        break;
      }
      case State::kConfig: {
        config_step_++;
        if (config_step_ >= kConfigSteps) {
          state_ = State::kReadWhoAmI;
        }
        break;
      }
      case State::kReadWhoAmI: {
        if (buf_[0] != kWhoAmIValue) {
          Restart(Resync::kWhoAmI, kRetryCycles);
        } else {
          state_ = State::kReadFreqFine;
        }
        break;
      }
      case State::kReadFreqFine: {
        storage_->control.freq_fine.store(static_cast<int8_t>(buf_[0]));
        storage_->control.running_count.fetch_add(1);
        remaining_words_ = 0;
        state_ = State::kRunning;
        break;
      }
      case State::kRunning: {
        if (remaining_words_ > 0) {
          ParseWord();
          remaining_words_--;
        } else {
          // FIFO_STATUS1/2: DIFF_FIFO[8:0], FIFO_OVR_LATCHED in bit 3
          // of STATUS2 (cleared by this read).
          remaining_words_ = static_cast<uint16_t>(
              buf_[0] | ((buf_[1] & 0x01) << 8));
          if (buf_[1] & 0x08) {
            storage_->control.overrun_count.fetch_add(1);
          }
        }
        break;
      }
    }
    Publish();
  }

  State state() const { return state_; }
  Resync last_resync() const { return last_resync_; }

  // Debug hooks (console `aux2 fusion ...`).
  void set_halt(bool halt) { halt_ = halt; }
  bool halt() const { return halt_; }
  void set_drop(uint16_t words) { drop_ = words; }

 private:
  struct ConfigStep {
    uint8_t reg;
    uint8_t value;
  };

  static constexpr uint8_t kRegFuncCfgAccess = 0x01;
  static constexpr uint8_t kRegIfCfg = 0x03;
  static constexpr uint8_t kRegFifoCtrl3 = 0x09;
  static constexpr uint8_t kRegFifoCtrl4 = 0x0A;
  static constexpr uint8_t kRegWhoAmI = 0x0F;
  static constexpr uint8_t kRegCtrl1 = 0x10;
  static constexpr uint8_t kRegCtrl2 = 0x11;
  static constexpr uint8_t kRegCtrl3 = 0x12;
  static constexpr uint8_t kRegCtrl6 = 0x15;
  static constexpr uint8_t kRegCtrl7 = 0x16;
  static constexpr uint8_t kRegCtrl8 = 0x17;
  static constexpr uint8_t kRegCtrl9 = 0x18;
  static constexpr uint8_t kRegFifoStatus1 = 0x1B;
  static constexpr uint8_t kRegInternalFreqFine = 0x4F;
  static constexpr uint8_t kRegFunctionsEnable = 0x50;
  static constexpr uint8_t kRegFifoDataOutTag = 0x78;
  static constexpr uint8_t kRegEmbFuncEnA = 0x04;      // embedded bank
  static constexpr uint8_t kRegEmbFuncFifoEnA = 0x44;  // embedded bank
  static constexpr uint8_t kWhoAmIValue = 0x70;

  // docs/imu_orientation_redesign.md §5.1, fusion mode.
  static constexpr ConfigStep kConfig[] = {
    // ASF_CTRL: keep the I2C anti-spike filters on.  By default the chip
    // turns them off for good once it sees the I3C broadcast address
    // (7'h7E/W) on the bus (datasheet 5.2.3), which a glitch can fake.
    // IF_CFG survives SW_RESET and MCU resets (the chip keeps power), so
    // this also protects every later boot until a power cycle.
    {kRegIfCfg, 0x20},
    {kRegCtrl3, 0x44},            // BDU, IF_INC
    {kRegCtrl1, 0x06},            // accel 120 Hz, high performance
    {kRegCtrl2, 0x09},            // gyro 960 Hz, high performance
    {kRegCtrl6, 0x24},            // +-2000 dps, LPF1 149 Hz
    {kRegCtrl7, 0x01},            // LPF1_G_EN
    {kRegCtrl8, 0x01},            // +-4 g, LPF2 at ODR/4
    {kRegCtrl9, 0x08},            // LPF2_XL_EN
    {kRegFunctionsEnable, 0x40},  // TIMESTAMP_EN
    {kRegFifoCtrl3, 0x96},        // BDR_GY 960 Hz, BDR_XL 120 Hz
    {kRegFifoCtrl4, 0x00},        // bypass: flush
    {kRegFuncCfgAccess, 0x80},    // embedded bank
    // The chip keeps power across MCU resets and flashing, and SW_RESET
    // is not documented to clear the embedded bank: disable the SFLP and
    // its FIFO batching explicitly.
    {kRegEmbFuncEnA, 0x00},       // SFLP off
    {kRegEmbFuncFifoEnA, 0x00},   // no SFLP batching
    {kRegFuncCfgAccess, 0x00},    // main bank
    {kRegFifoCtrl4, 0xC6},        // timestamp every 32 slots, continuous
  };
  static constexpr uint8_t kConfigSteps =
      sizeof(kConfig) / sizeof(kConfig[0]);

  static constexpr uint16_t kResetTimeoutCycles = 300;  // 10 ms
  static constexpr uint16_t kRetryCycles = 3000;        // 100 ms
  static constexpr uint16_t kI2cErrorRetryCycles = 120;  // 4 ms
  static constexpr uint8_t kErrorsBeforeResync = 3;

  bool Write(uint8_t reg, uint8_t value) {
    tx_ = value;
    i2c_->StartWriteMemory(
        address_, reg,
        std::string_view(reinterpret_cast<const char*>(&tx_), 1));
    return true;
  }

  bool Read(uint8_t reg, size_t size) {
    i2c_->StartReadMemory(
        address_, reg,
        mjlib::base::string_span(reinterpret_cast<char*>(buf_), size));
    return true;
  }

  void Restart(Resync reason, uint16_t wait_cycles) {
    last_resync_ = reason;
    storage_->control.resync_count.fetch_add(1);
    remaining_words_ = 0;
    if (wait_cycles > 0) {
      retry_wait_ = wait_cycles;
      state_ = State::kRetryWait;
    } else {
      state_ = State::kIdle;
    }
    Publish();
  }

  void ParseWord() {
    FusionWord word;
    word.t = static_cast<uint16_t>(TIM3->CNT);
    word.tag = static_cast<uint8_t>(buf_[0] >> 3);
    word.flags = static_cast<uint8_t>((buf_[0] >> 1) & 0x03);  // TAG_CNT
    word.v[0] = static_cast<int16_t>(buf_[1] | (buf_[2] << 8));
    word.v[1] = static_cast<int16_t>(buf_[3] | (buf_[4] << 8));
    word.v[2] = static_cast<int16_t>(buf_[5] | (buf_[6] << 8));
    if (word.tag == kFusionTagGyro) {
      // Counted before the drop decision so every loss is visible.
      gyro_seq_++;
      word.seq = gyro_seq_;
      if (drop_ > 0) {
        drop_--;
        return;
      }
    } else {
      word.seq = gyro_seq_;
    }
    storage_->mailbox.Push(word);
  }

  void Publish() {
    storage_->control.init_state.store(static_cast<uint8_t>(state_));
  }

  Stm32I2c* const i2c_;
  const uint8_t address_;
  FusionStorage* const storage_;
  aux::ImuFusionStatus* const status_;

  State state_ = State::kIdle;
  Resync last_resync_ = Resync::kNone;
  uint8_t config_step_ = 0;
  uint16_t wait_count_ = 0;
  uint16_t retry_wait_ = 0;
  uint16_t remaining_words_ = 0;
  uint16_t gyro_seq_ = 0;
  uint16_t drop_ = 0;
  uint8_t consecutive_errors_ = 0;
  bool halt_ = false;
  uint8_t tx_ = 0;
  uint8_t buf_[8] = {};
};

}
