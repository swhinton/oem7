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

#include "detail/parsers.hpp"
#include "messages/MessageIds.hpp"

namespace novatel::oem7 {
using SyncType = std::array<std::byte, 3>;
using CRC_t = uint32_t;

constexpr SyncType kSync = {std::byte{0xAA}, std::byte{0x44}, std::byte{0x12}};

enum class TimeStatus : uint8_t {
  kUnknown = 20,
  kApproximate = 60,
  kCoarseAdjusting = 80,
  kCoarse = 100,
  kCoarseSteering = 120,
  kFreeWheeling = 130,
  kFineAdjusting = 140,
  kFine = 160,
  kFineBackupSteering = 170,
  kFineSteering = 180,
  kSatTime = 200  //! Time from satellite, only used in ephemeris, almanac, et al.
};

// The "top bytes" get dropped because the field can only actually be a uint8_t.
// Why wouldn't you just use the uint8 values? ...
enum class PortIdentifier : uint16_t {
  kNoPorts = 0,
  kCOM1 = 0x20,
  kCOM2 = 0x40,
  kCOM3 = 0x60,
  kSPECIAL = 0xa0,
  kTHISPORT = 0xc0,
  kFILE = 0xe0,
  kUSB1 = 0x5a0,
  kUSB2 = 0x6a0,
  kUSB3 = 0x7a0,
  kAUX = 0x8a0,
  kCOM4 = 0xba0,
  kETH1 = 0xca0,
  kIMU = 0xda0,
  kICOM1 = 0xfa0,
  kICOM2 = 0x10a0,
  kICOM3 = 0x11a0,
  kNCOM1 = 0x12a0,
  kNCOM2 = 0x13a0,
  kNCOM3 = 0x14a0,
  kICOM4 = 0x15a0,
  kWCOM1 = 0x16a0,
  kCOM5 = 0x17a0,
  kCOM6 = 0x18a0,
  kBT1 = 0x19a0,
  kCOM7 = 0x1aa0,
  kCOM8 = 0x1ba0,
  kCOM9 = 0x1ca0,
  kCOM10 = 0x1da0,
  kCCOM1 = 0x1ea0,
  kCCOM2 = 0x1fa0,
  kCCOM3 = 0x20a0,
  kCCOM4 = 0x21a0,
  kCCOM5 = 0x22a0,
  kCCOM6 = 0x23a0,
  kICOM5 = 0x26a0,
  kICOM6 = 0x27a0,
  kICOM7 = 0x28a0,
  kSCOM1 = 0x29a0,
  kSCOM2 = 0x2aa0,
  kSCOM3 = 0x2ba0,
  kSCOM4 = 0x2ca0
};

// TODO Virtual ports?

struct MessageHeader {
  SyncType sync = kSync;
  uint8_t header_length{};
  MessageId message_id{};
  uint8_t message_type_part{};
  // TODO accessor functions to interpret the message_type_part
  uint8_t port_address{};
  uint16_t message_length{};  //! Does not include the header length or CRC.
  uint16_t sequence_count{};  //! Used for multiple related logs (like a memdump)
  uint8_t cpu_idle_time{};    //! 0-200; divide by two to get x/100%
  TimeStatus time_status{};
  uint16_t gps_week{};
  uint32_t msec_since_gps_ref_week{};
  uint32_t receiver_status{};
  // TODO accessor functions to interpret the receiver status
  uint16_t reserved{};
  uint16_t receiver_sw_version{};  //! "software build number"
};

enum class ResponseID : uint32_t {
  // todo...
};

struct ResponseMessage {
  MessageHeader header{};
  ResponseID response_id{};
  std::vector<char> ascii_response;
};

/**
 * Find the next OEM log message in a buffer containing at least one.
 *
 * This assumes the client wants to manage the memory. Client can rotate their
 * buffer given the end_idx of the next message. E.g. if this returns the expected
 * value (complete message found), if you had a std::vector you would rotate the
 * vector around the end_idx_payload after you've deserialized it and then call this
 * function again to process the next message.
 *
 * @param buf buffer containing at least one OEM7 binary data log.
 * @return {0, std::errc::no_message_available} if no message found at all
 * @return {-1, std::errc} if header was found but failed to deserialize (logic error or corrupt data,
 * clear the buffer)
 * @return {stat_idx_header, std::errc::no_buffer_space} if header found but message incomplete
 * @return (Header, start_idx_payload, end_idx_payload) complete message
 */
OEM7_EXPECTED<std::tuple<MessageHeader, std::size_t, std::size_t>, std::pair<ssize_t, std::errc>>
next_message(std::span<const std::byte> buf);

template <typename T>
concept HasMessageIdType = requires(T t) {
  { std::remove_cvref_t<decltype(T::kMessageId)>{} } -> std::same_as<MessageId>;
};

/**
 * Given a raw buffer starting at the beginning of the desired log message, deserialize the raw and
 * overlay onto the struct.
 * @tparam MessageId An OEM7 log message.
 * @param raw buffer starting at beginning of log message
 * @param id reference to struct onto which raw is overlaid
 */
template <HasMessageIdType Message>
OEM7_EXPECTED<void, std::errc> get_log(std::span<const std::byte> raw, Message& msg) {
  constexpr std::array kVariableLengthMessageIds = {MessageId::HWMONITOR, MessageId::SATXYZ2,
                                                    MessageId::RANGECMP};
  if constexpr (std::ranges::any_of(kVariableLengthMessageIds,
                                    [](const MessageId& id) { return id == Message::kMessageId; })) {
    return detail::get_variable_length_message<std::remove_cvref_t<decltype(msg)>,
                                               typename Message::RecordType>(raw, msg);
  } else {
    // Everything else is assumed to be "default" ser/deser.
    zpp::bits::in in(raw, zpp::bits::endian::little{});
    if (auto ser_result = in(msg); failure(ser_result)) {
      return OEM7_UNEXPECTED(ser_result);
    }
    return {};
  }
}

/**
 * Convenience function to check that a log message CRC is good.
 */
bool check_crc(std::span<const std::byte> log_msg);
}  // namespace novatel::oem7
