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

#include <zpp_bits.h>

#include <span>
#include <system_error>

#include "novatel/compat.hpp"
#include "novatel/messages/HWMONITOR.hpp"
#include "novatel/messages/RANGECMP.hpp"
#include "novatel/messages/SATXYZ2.hpp"

namespace novatel::oem7::detail {

// Maybe concepts make sense here but....

template <typename T, typename U>
OEM7_EXPECTED<void, std::errc> get_variable_length_message(std::span<const std::byte> raw, T& msg) {
  zpp::bits::in in(raw, zpp::bits::endian::little{});
  if (auto ser_result = in(msg.num_records); failure(ser_result)) {
    return OEM7_UNEXPECTED(ser_result);
  }
  msg.records.resize(msg.num_records);

  for (std::size_t i = 0; i < msg.num_records; ++i) {
    U record{};
    if (auto ser_result = in(record); failure(ser_result)) {
      return OEM7_UNEXPECTED(ser_result);
    }
    msg.records[i] = std::move(record);
  }
  // Not a failure if we can't pull the CRC out, if caller is checking CRCs, it won't match.
  (void)in(msg.crc);
  return {};
}
}  // namespace novatel::oem7::detail