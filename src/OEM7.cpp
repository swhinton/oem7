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

#include "novatel/OEM7.hpp"

#include "novatel/crc.hpp"

namespace novatel::oem7 {

OEM7_EXPECTED<std::tuple<MessageHeader, std::size_t, std::size_t>, std::pair<ssize_t, std::errc>>
next_message(std::span<const std::byte> buf) {
  const auto it = std::ranges::find_first_of(buf, kSync);
  if (it == buf.end()) {
    return OEM7_UNEXPECTED(
        std::make_pair<std::size_t, std::errc>{0, std::errc::no_message_available});
  }

  const auto offset = std::distance(buf.begin(), it);
  const auto msg_span = buf.subspan(offset);
  zpp::bits::in in(msg_span, zpp::bits::endian::little{});

  MessageHeader header{};
  if (auto ser_result = in(header); failure(ser_result)) {
    return OEM7_UNEXPECTED(std::make_pair<ssize_t, std::errc>{-1, ser_result});
  }

  if ((in.position() + header.message_length + sizeof(CRC_t)) > buf.size()) {
    return OEM7_UNEXPECTED(
        std::make_pair<ssize_t, std::errc>(static_cast<ssize_t>(offset), std::errc::no_buffer_space));
  }

  return std::tuple(header, in.position(), in.position() + header.message_length + sizeof(CRC_t));
}



}
