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

#include "MessageIds.hpp"

namespace novatel::oem7 {
enum class ClockStatus : uint32_t { kValid, kConverging, kIterating, kInvalid };

enum class UTCStatus : uint32_t { kInvalid, kValid, kWarning };

struct TIME {
  static constexpr MessageId kMessageId = MessageId::TIME;
  ClockStatus clock_status{};
  double receiver_offset_from_gps_time_s{};
  double offset_std_dev_s{};
  double receiver_offset_from_utc_time_s{};
  uint32_t utc_year{};
  uint8_t utc_month{};
  uint8_t utc_day{};
  uint8_t utc_hour{};
  uint8_t utc_minute{};
  uint32_t utc_msec{};  //! 0-60999
  UTCStatus utc_status{};
  uint32_t crc{};
};
};