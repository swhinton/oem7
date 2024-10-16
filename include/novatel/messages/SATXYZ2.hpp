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

namespace novatel::oem7 {

enum SatSystem : uint32_t { kGps, kGlonass, kSbas, kGalileo = 5, kBeiDou, kQzss, kNavIc = 9 };

struct SatelliteRecord {
  SatSystem system{};
  uint32_t id{};
  double r_x_ecef_m{};
  double r_y_ecef_m{};
  double r_z_ecef_m{};
  double clock_correction_m{};
  double ionosphere_delay_m{};
  double troposphere_delay_m{};
  double reserved0{};
  double reserved1{};
};

struct SATXYZ2 {
  static constexpr MessageId kMessageId = MessageId::SATXYZ2;
  using RecordType = SatelliteRecord;
  uint32_t num_records;
  std::vector<SatelliteRecord> records;
  uint32_t crc;
};
};  // namespace novatel::oem7