/**
 * Copyright 2024 Scott Hinton
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
 * associated documentation files (the “Software”), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge, publish, distribute,
 * sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
 * NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT
 * OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#pragma once

#include <cstdint>
#include <vector>

#include "MessageIds.hpp"

namespace novatel::oem7 {
enum class BoundaryLimitStatus : uint8_t {
  kWithinAcceptableBounds,
  kLowerWarnViolation,
  kLowerRedlineViolation,
  kUpperWarnViolation,
  kUpperRedlineViolation
};

enum class HwMonitorReadingType : uint8_t {
  kTemperature = 0x01,
  kAntennaCurrent,
  kDigitalCore3v3 = 0x06,
  kAntennaVoltage,
  kDigital1v2CoreVoltage,
  kRegulatedSupplyVoltage = 0x0F,
  k1v8 = 0x11,
  k5v = 0x15,
  kSecondaryTemperature,
  kPeripheralCoreVoltage,
  kSecondaryAntennaCurrent,
  kSecondaryAntennaVoltage
};

struct HwMonitorStatus {
  BoundaryLimitStatus boundary_limit_status{};
  HwMonitorReadingType hw_monitor_reading_type{};
  uint16_t spare{};
};

struct HwMonitorEntry {
  float reading{};
  HwMonitorStatus status{};
};

struct HWMONITOR {
  static constexpr MessageId kMessageId = MessageId::HWMONITOR;
  using RecordType = HwMonitorEntry;
  uint32_t num_records{};
  std::vector<HwMonitorEntry> records{};
  uint32_t crc{};
};
}