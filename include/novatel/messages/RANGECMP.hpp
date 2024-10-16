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

struct RangeObservation {
  uint32_t channel_tracking_status{};
  uint64_t part0{};  // Doppler Frequency [0:27] Hz, Pseudorange measurement [28:65] 1/128 m
  uint32_t doppler_freq_hz() const { return part0 & 0xFFFFFFF; }
  uint64_t pseudo_range_meas_m() const { return 0xFFFFFFFFF & (part0 >> 28); }
  uint32_t adr{};   //! 1/256 cycles
  uint8_t part1{};  // StdDev-PSR [0:3] m, StdDev-ADR [4:7] (n+1)/512 cycles
  uint8_t stddev_psr_m() const { return part1 & 0xF; }
  uint8_t stddev_adr_cycles() const { return 0xF & (part1 >> 4); }
  uint8_t prn_slot{};
  uint32_t
      part2{};  // Lock Time [0:20] 1/32 seconds, C/No [21:25]  (20+n) dB-Hz, GLONASS Freq number [26:
  uint32_t lock_time_sec() const { return part2 & 0x1FFFFF; }
  uint8_t c_no() const { return 0x1F & (part2 >> 21); }
  uint8_t glonass_freq_num() const { return 0x7F & (part2 >> 26); }
  uint16_t reserved{};
};

struct RANGECMP {
  static constexpr MessageId kMessageId = MessageId::RANGECMP;
  using RecordType = RangeObservation;
  uint32_t num_records{};
  std::vector<RangeObservation> records;
  uint32_t crc{};
};
};  // namespace novatel::oem7