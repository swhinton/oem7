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

#include <gtest/gtest.h>
#include <spdlog/spdlog.h>

#include <magic_enum.hpp>

#include "base64.hpp"
#include "novatel/OEM7.hpp"
#include "spdlog/fmt/bin_to_hex.h"

// Log specializations for debugging
template <>
struct fmt::formatter<novatel::oem7::BESTXYZ> : fmt::formatter<std::string> {
  // Parse is inherited from fmt::formatter<std::string>.
  auto format(const novatel::oem7::BESTXYZ& c, fmt::format_context& ctx) const
      -> decltype(ctx.out()) {
    // clang-format off
    return formatter<std::string>::format(
      fmt::format(R"(
BESTXYZ
-----------------------------
position_solution_status = {}
position_type = {}
r_x_m = {}
r_y_m = {}
r_z_m = {}
r_x_stdev_m = {}
r_y_stdev_m = {}
r_z_stdev_m = {}
velocity_solution_status = {}
velocity_type = {}
v_x_mps = {}
v_y_mps = {}
v_z_mps = {}
v_x_stdev_mps = {}
v_y_stdev_mps = {}
v_z_stdev_mps = {}
base_station_id = {}
v_latency_s = {}
differential_age_s = {}
solution_age_s = {}
num_sats_tracked = {}
num_sats_in_solution = {}
num_sats_with_l1_e1_b1_signals_used_in_solution = {}
num_sats_multi_freq_used_in_solution = {}
extended_solution_status = {}
galilelo_beidou_signal_mask = {}
gps_glonass_signal_mask = {}
crc = {}
      )",
      magic_enum::enum_name(c.position_solution_status),
      magic_enum::enum_name(c.position_type),
      c.r_x_m,
      c.r_y_m,
      c.r_z_m,
      c.r_x_stdev_m,
      c.r_y_stdev_m,
      c.r_z_stdev_m,
      magic_enum::enum_name(c.velocity_solution_status),
      magic_enum::enum_name(c.velocity_type),
      c.v_x_mps,
      c.v_y_mps,
      c.v_z_mps,
      c.v_x_stdev_mps,
      c.v_y_stdev_mps,
      c.v_z_stdev_mps,
      spdlog::to_hex(c.base_station_id),
      c.v_latency_s,
      c.differential_age_s,
      c.solution_age_s,
      c.num_sats_tracked,
      c.num_sats_in_solution,
      c.num_sats_with_l1_e1_b1_signals_used_in_solution,
      c.num_sats_multi_freq_used_in_solution,
      c.extended_solution_status,
      c.galilelo_beidou_signal_mask,
      c.gps_glonass_signal_mask,
      c.crc
        ),
      ctx);
    // clang-format on
  }
};

template <>
struct fmt::formatter<novatel::oem7::TIME> : fmt::formatter<std::string> {
  // Parse is inherited from fmt::formatter<std::string>.
  auto format(const novatel::oem7::TIME& c, fmt::format_context& ctx) const -> decltype(ctx.out()) {
    // clang-format off
    return formatter<std::string>::format(
      fmt::format(R"(
TIME
-----------------------------
ClockStatus clock_status = {}
double receiver_offset_from_gps_time_s = {}
double offset_std_dev_s = {}
double receiver_offset_from_utc_time_s = {}
uint32_t utc_year = {}
uint8_t utc_month = {}
uint8_t utc_hour = {}
uint8_t utc_minute = {}
uint32_t utc_msec = {}
UTCStatus utc_status = {}
uint32_t crc = {}
      )",
        magic_enum::enum_name(c.clock_status),
        c.receiver_offset_from_gps_time_s,
        c.offset_std_dev_s,
        c.receiver_offset_from_utc_time_s,
        c.utc_year,
        c.utc_month,
        c.utc_hour,
        c.utc_minute,
        c.utc_msec,
        magic_enum::enum_name(c.utc_status),
        c.crc
      ),
      ctx);
    // clang-format on
  }
};

template <>
struct fmt::formatter<novatel::oem7::SATXYZ2> : fmt::formatter<std::string> {
  // Parse is inherited from fmt::formatter<std::string>.
  auto format(const novatel::oem7::SATXYZ2& c, fmt::format_context& ctx) const
      -> decltype(ctx.out()) {
    // clang-format off
    std::string result = R"(
SATXYZ2
-----------------------------
)";
    for (std::size_t i = 0; i < c.num_records; ++i) {
      result += fmt::format(R"(
SAT-{}
-----------------------------
  SatSystem system = {}
  uint32_t id = {}
  double r_x_ecef_m = {}
  double r_y_ecef_m = {}
  double r_z_ecef_m = {}
  double clock_correction_m = {}
  double ionosphere_delay_m = {}
  double troposphere_delay_m = {}
)",
      i,
      magic_enum::enum_name(c.records[i].system),
      c.records[i].id,
      c.records[i].r_x_ecef_m,
      c.records[i].r_y_ecef_m,
      c.records[i].r_z_ecef_m,
      c.records[i].clock_correction_m,
      c.records[i].ionosphere_delay_m,
      c.records[i].troposphere_delay_m
      );
    }

    return formatter<std::string>::format(
      result,
      ctx);
    // clang-format on
  }
};

template <>
struct fmt::formatter<novatel::oem7::RANGECMP> : fmt::formatter<std::string> {
  // Parse is inherited from fmt::formatter<std::string>.
  auto format(const novatel::oem7::RANGECMP& c, fmt::format_context& ctx) const
      -> decltype(ctx.out()) {
    // clang-format off
    std::string result = R"(
RANGECMP
-----------------------------
)";
    for (std::size_t i = 0; i < c.num_records; ++i) {
      result += fmt::format(R"(
RANGE-{}
-----------------------------
  uint32_t channel_tracking_status = {}
  uint32_t doppler_freq_hz = {}
  uint64_t pseudo_range_meas_m = {}
  uint32_t adr = {}
  uint8_t stddev_psr_m = {}
  uint8_t stddev_adr_cycles = {}
  uint8_t prn_slot = {}
  uint32_t lock_time_sec = {}
  uint8_t c_no = {}
  uint8_t glonass_freq_num = {}
)",
      i,
  c.records[i].channel_tracking_status,
  c.records[i].doppler_freq_hz(),
  c.records[i].pseudo_range_meas_m(),
  c.records[i].adr,
  c.records[i].stddev_psr_m(),
  c.records[i].stddev_adr_cycles(),
  c.records[i].prn_slot,
  c.records[i].lock_time_sec(),
  c.records[i].c_no(),
  c.records[i].glonass_freq_num()
      );
    }

    return formatter<std::string>::format(
      result,
      ctx);
    // clang-format on
  }
};

namespace novatel::oem7::testing {

// constexpr std::string_view kCommandResponse = "DQo8T0sNCltDT00xXQ==";
constexpr std::string_view kHwMonitorB64str =
    "qkQSHMMDACBMAAAAurQhCZDJ+"
    "wQAgAAC21J9QgkAAAANyPBBAAEAAJEMyTwAAgAADc1QQAAGAAAv6qJAAAcAAKAFmj8ACAAAEh1RQAAPAACnbuo/"
    "ABEAADdao0AAFQAA6FbuQQAWAABMh24l";
constexpr std::string_view kRangeCmpB64str =
    "qkQSHIwAACCEAQAAubQhCah3CAUAgAACkZZ9QhAAAAAE3AAIBjsFkPn8DgqqFYrkQwgDhCMDAAAk3AAI0Lj0f18iUgrTzm+"
    "4MgTozmMDAABE3AAYP4MMcFE1oQsNBFXcZA6WnsACAABk3AAYQT7736lmqgnqJ52mxQnDxSYCAACk3AAIfI0J8MtL5wrB3HP"
    "WMh5UloIDAADE3AAYMwQFsKn9IgrBgGbXQgfdWUUDAADk3AAIJ+X8D9nLaAoX84upMhuBhWQDAAAk3QAYpjn/"
    "DxfJogvV8kvbdBSABaICAABE3QAYs/"
    "rxH7QHRwsd65CXhBDig4gCAAAE30NIOnH0v1oPZw1OijWyYx1ogscCAAAk30NIP00IcE53EgxlUO+RMQaz3+"
    "IDAABE30NIubv4T5HyzgvR6Ui+MhW1vogDAABk30NI5aMJYJhEpQxu4ICxQx/"
    "MuQEDAACE30NIBKkKYJt9Hw0WCzjhQgk87kADAACk30NIqegDUGOpWAsbN/"
    "yLMgQ8QYUDAADk30NIzij4z2JNhAtuJVLvUxMqkMECAADParZM";
constexpr std::string_view kSatXYZ2B64str =
    "qkQSHKsFACCMAgAAubQhCdhpCQUAgAACE3V9QgkAAAAAAAAACAAAACFiE4Gu8llB3ckKtSR1dsEVoitxbTFiQavhI8zEkvxA"
    "AAAAACs9IkAAAADAPVUFQAAAAAAAAAAAAAAAAAAAAAAAAAAABAAAANI45qrSWBpBV+n2ipQrecHG+oou+"
    "R9AQVHYHsEuBwFBAAAAwBvQJEAAAABATBEIQAAAAAAAAAAAAAAAAAAAAAAAAAAADgAAACyGjJ8JPnbBS7nnee/"
    "sZ8GPmwMUQsBIwVtceaSjGgNBAAAAgOsoNEAAAAAA32whQAAAAAAAAAAAAAAAAAAAAAAAAAAACQAAANPDSHuK0lzBst9+"
    "CnbCdcE0Lr0gwj1lQcSylKd/1P1AAAAAACs8HkAAAAAgZPMAQAAAAAAAAAAAAAAAAAAAAAAAAAAAHgAAAANa2b/"
    "ve3LB2UmaW4GIUcHhzCDCSARxQcFsCNHO1vXAAAAA4K1SKEAAAABAqjANQAAAAAAAAAAAAAAAAAAAAAAAAAAABwAAAEQolXy"
    "8cWPBdiRx1ZlZZ8EBn4ivLs90QQ19466PYb7AAAAAYN9+"
    "H0AAAAAgVwgCQAAAAAAAAAAAAAAAAAAAAAAAAAAAGwAAAN40CFHGCGhBUNI1Fzq1bMG9kaWPPZZwQahfEsAeSsTAAAAAYLTB"
    "JEAAAADgGdwJQAAAAAAAAAAAAAAAAAAAAAAAAAAAFAAAAKUxTE76ZHPB2UrTuwDKUEEmK60lfnZvQW6NSCNXC/"
    "tAAAAAoF8dM0AAAAAAE9giQAAAAAAAAAAAAAAAAAAAAAAAAAAAEAAAAPRHsIRdd25BRWsBq4+"
    "BVcFcI9jHiUlzQdUC8K8V5uXAAAAAQOV+LEAAAADgFoMYQAAAAAAAAAAAAAAAAAAAAABYxMJD";
constexpr std::string_view kBestXYZB64str =
    "qkQSHPEAACBwAAAAubQhCTACCgUAgAACz0R9QgAAAAAQAAAASBK3I/N4M8GsTrOFVQtSwdTKWc1oEk9BjGTWP5R/"
    "J0DeKTtAAAAAAAgAAAAAZTHkYMhCv4AeZjr26nA/AI5tH3vOQL+q/"
    "0w+uCigPmv2sj4AAAAAAAAAAAAAAAAAAAAAEAkJAAACAAHXLORS";
constexpr std::string_view kTimeB64str =
    "qkQSHGUAACAsAAAAurQhCbC9CgUAgAACJJl9QgAAAAB9RJNmtc8gvqlsIXMY4hY+f377////"
    "McDoBwAAChQXHQB9AAABAAAAAE9r/w==";
constexpr std::string_view kRxStatusB64str =
    "qkQSHF0AACBYAAAAurQhCZBpCwUAgAAC4Sp9QgAAAAAFAAAAAIAAAgAAAAAAAAAAAAAAAIEABAAIEAAAAAAAAAAAAAAAAAAA"
    "AAAAAAAAAIAAAAAAAAAAgAAAAAAAAAAAAAAAAADAMAAAAAAA/////wAAAAABgOzt";

class TestOem7 : public ::testing::Test {
 protected:
  void SetUp() override {
    MessageHeader header;
    kMessageHeaderSize = zpp::bits::as_bytes(header).size_in_bytes();
    raw_hw_monitor_msg_ = std::move(base64::decode_into<std::vector<uint8_t>>(kHwMonitorB64str));
    raw_range_cmp_msg_ = std::move(base64::decode_into<std::vector<uint8_t>>(kRangeCmpB64str));
    raw_sat_xyz_2_msg_ = std::move(base64::decode_into<std::vector<uint8_t>>(kSatXYZ2B64str));
    raw_best_xyz_msg_ = std::move(base64::decode_into<std::vector<uint8_t>>(kBestXYZB64str));
    raw_time_msg_ = std::move(base64::decode_into<std::vector<uint8_t>>(kTimeB64str));
    raw_rx_status_msg_ = std::move(base64::decode_into<std::vector<uint8_t>>(kRxStatusB64str));
  }

  // This needs to be runtime because bitset is not a structural type.... fml
  size_t kMessageHeaderSize = zpp::bits::to_bytes<MessageHeader{}>().size();
  std::vector<uint8_t> raw_hw_monitor_msg_;
  std::vector<uint8_t> raw_range_cmp_msg_;
  std::vector<uint8_t> raw_sat_xyz_2_msg_;
  std::vector<uint8_t> raw_best_xyz_msg_;
  std::vector<uint8_t> raw_time_msg_;
  std::vector<uint8_t> raw_rx_status_msg_;
};

TEST_F(TestOem7, hwmonitorb) {
  std::span byte_span{reinterpret_cast<std::byte*>(raw_hw_monitor_msg_.data()),
                      raw_hw_monitor_msg_.size()};
  SPDLOG_INFO("hwmonitorb raw: {}", spdlog::to_hex(byte_span));

  auto next_msg_result = next_message(byte_span);
  ASSERT_TRUE(next_msg_result);

  MessageHeader header = std::get<0>(next_msg_result.value());
  ASSERT_EQ(header.message_id, MessageId::HWMONITOR);
  ASSERT_EQ(header.header_length, kMessageHeaderSize);

  size_t msg_start_idx = std::get<1>(next_msg_result.value());
  size_t msg_end_idx = std::get<2>(next_msg_result.value());

  EXPECT_EQ(msg_start_idx, kMessageHeaderSize);
  EXPECT_EQ(msg_end_idx, byte_span.size());

  HWMONITOR msg{};
  auto get_log_result = get_log(byte_span.subspan(msg_start_idx, (msg_end_idx - msg_start_idx)), msg);
  ASSERT_TRUE(get_log_result);
  ASSERT_EQ(msg.num_records, 9);

  EXPECT_NEAR(msg.records[0].reading, 30.097, 1e-2);
  EXPECT_EQ(msg.records[0].status.boundary_limit_status,
            BoundaryLimitStatus::kWithinAcceptableBounds);
  EXPECT_EQ(msg.records[0].status.hw_monitor_reading_type, HwMonitorReadingType::kTemperature);

  EXPECT_NEAR(msg.records[1].reading, 0.0245, 1e-2);
  EXPECT_EQ(msg.records[1].status.boundary_limit_status,
            BoundaryLimitStatus::kWithinAcceptableBounds);
  EXPECT_EQ(msg.records[1].status.hw_monitor_reading_type, HwMonitorReadingType::kAntennaCurrent);

  EXPECT_NEAR(msg.records[2].reading, 3.262, 1e-2);
  EXPECT_EQ(msg.records[2].status.boundary_limit_status,
            BoundaryLimitStatus::kWithinAcceptableBounds);
  EXPECT_EQ(msg.records[2].status.hw_monitor_reading_type, HwMonitorReadingType::kDigitalCore3v3);

  EXPECT_NEAR(msg.records[3].reading, 5.091, 1e-2);
  EXPECT_EQ(msg.records[3].status.boundary_limit_status,
            BoundaryLimitStatus::kWithinAcceptableBounds);
  EXPECT_EQ(msg.records[3].status.hw_monitor_reading_type, HwMonitorReadingType::kAntennaVoltage);

  EXPECT_NEAR(msg.records[4].reading, 1.203, 1e-2);
  EXPECT_EQ(msg.records[4].status.boundary_limit_status,
            BoundaryLimitStatus::kWithinAcceptableBounds);
  EXPECT_EQ(msg.records[4].status.hw_monitor_reading_type,
            HwMonitorReadingType::kDigital1v2CoreVoltage);

  EXPECT_NEAR(msg.records[5].reading, 3.267, 1e-2);
  EXPECT_EQ(msg.records[5].status.boundary_limit_status,
            BoundaryLimitStatus::kWithinAcceptableBounds);
  EXPECT_EQ(msg.records[5].status.hw_monitor_reading_type,
            HwMonitorReadingType::kRegulatedSupplyVoltage);

  EXPECT_NEAR(msg.records[6].reading, 1.831, 1e-2);
  EXPECT_EQ(msg.records[6].status.boundary_limit_status,
            BoundaryLimitStatus::kWithinAcceptableBounds);
  EXPECT_EQ(msg.records[6].status.hw_monitor_reading_type, HwMonitorReadingType::k1v8);

  EXPECT_NEAR(msg.records[7].reading, 5.104, 1e-2);
  EXPECT_EQ(msg.records[7].status.boundary_limit_status,
            BoundaryLimitStatus::kWithinAcceptableBounds);
  EXPECT_EQ(msg.records[7].status.hw_monitor_reading_type, HwMonitorReadingType::k5v);

  EXPECT_NEAR(msg.records[8].reading, 29.792, 1e-2);
  EXPECT_EQ(msg.records[8].status.boundary_limit_status,
            BoundaryLimitStatus::kWithinAcceptableBounds);
  EXPECT_EQ(msg.records[8].status.hw_monitor_reading_type,
            HwMonitorReadingType::kSecondaryTemperature);
}

TEST_F(TestOem7, rxstatusb) {
  std::span byte_span{reinterpret_cast<std::byte*>(raw_rx_status_msg_.data()),
                      raw_rx_status_msg_.size()};
  SPDLOG_INFO("rxstatusb raw: {}", spdlog::to_hex(byte_span));

  auto next_msg_result = next_message(byte_span);
  ASSERT_TRUE(next_msg_result);

  MessageHeader header = std::get<0>(next_msg_result.value());
  ASSERT_EQ(header.message_id, MessageId::RXSTATUS);
  ASSERT_EQ(header.header_length, kMessageHeaderSize);

  size_t msg_start_idx = std::get<1>(next_msg_result.value());
  size_t msg_end_idx = std::get<2>(next_msg_result.value());

  EXPECT_EQ(msg_start_idx, kMessageHeaderSize);
  EXPECT_EQ(msg_end_idx, byte_span.size());

  RxStatus msg{};
  auto get_log_result = get_log(byte_span.subspan(msg_start_idx, (msg_end_idx - msg_start_idx)), msg);
  ASSERT_TRUE(get_log_result);
  ASSERT_EQ(msg.num_stats, 5);
}

TEST_F(TestOem7, rangecmpb) {
  std::span byte_span{reinterpret_cast<std::byte*>(raw_range_cmp_msg_.data()),
                      raw_range_cmp_msg_.size()};
  SPDLOG_INFO("rangecmpb raw: {}", spdlog::to_hex(byte_span));

  auto next_msg_result = next_message(byte_span);
  ASSERT_TRUE(next_msg_result);

  MessageHeader header = std::get<0>(next_msg_result.value());
  ASSERT_EQ(header.message_id, MessageId::RANGECMP);
  ASSERT_EQ(header.header_length, kMessageHeaderSize);

  size_t msg_start_idx = std::get<1>(next_msg_result.value());
  size_t msg_end_idx = std::get<2>(next_msg_result.value());

  EXPECT_EQ(msg_start_idx, kMessageHeaderSize);
  EXPECT_EQ(msg_end_idx, byte_span.size());

  RANGECMP msg{};
  auto get_log_result = get_log(byte_span.subspan(msg_start_idx, (msg_end_idx - msg_start_idx)), msg);
  ASSERT_TRUE(get_log_result);

  SPDLOG_INFO("{}", msg);
}

TEST_F(TestOem7, satxyz2b) {
  std::span byte_span{reinterpret_cast<std::byte*>(raw_sat_xyz_2_msg_.data()),
                      raw_sat_xyz_2_msg_.size()};
  SPDLOG_INFO("satxyz2b raw: {}", spdlog::to_hex(byte_span));

  auto next_msg_result = next_message(byte_span);
  ASSERT_TRUE(next_msg_result);

  MessageHeader header = std::get<0>(next_msg_result.value());
  ASSERT_EQ(header.message_id, MessageId::SATXYZ2);
  ASSERT_EQ(header.header_length, kMessageHeaderSize);

  size_t msg_start_idx = std::get<1>(next_msg_result.value());
  size_t msg_end_idx = std::get<2>(next_msg_result.value());

  EXPECT_EQ(msg_start_idx, kMessageHeaderSize);
  EXPECT_EQ(msg_end_idx, byte_span.size());

  SATXYZ2 msg{};
  auto get_log_result = get_log(byte_span.subspan(msg_start_idx, (msg_end_idx - msg_start_idx)), msg);
  ASSERT_TRUE(get_log_result);

  SPDLOG_INFO("{}", msg);
}

TEST_F(TestOem7, bestxyzb) {
  std::span byte_span{reinterpret_cast<std::byte*>(raw_best_xyz_msg_.data()),
                      raw_best_xyz_msg_.size()};
  SPDLOG_INFO("bestxyzb raw: {}", spdlog::to_hex(byte_span));

  auto next_msg_result = next_message(byte_span);
  ASSERT_TRUE(next_msg_result);

  MessageHeader header = std::get<0>(next_msg_result.value());
  ASSERT_EQ(header.message_id, MessageId::BESTXYZ);
  ASSERT_EQ(header.header_length, kMessageHeaderSize);

  size_t msg_start_idx = std::get<1>(next_msg_result.value());
  size_t msg_end_idx = std::get<2>(next_msg_result.value());

  EXPECT_EQ(msg_start_idx, kMessageHeaderSize);
  EXPECT_EQ(msg_end_idx, byte_span.size());

  BESTXYZ msg{};
  auto get_log_result = get_log(byte_span.subspan(msg_start_idx, (msg_end_idx - msg_start_idx)), msg);
  ASSERT_TRUE(get_log_result);

  SPDLOG_INFO("{}", msg);
}

TEST_F(TestOem7, timeb) {
  std::span byte_span{reinterpret_cast<std::byte*>(raw_time_msg_.data()), raw_time_msg_.size()};
  SPDLOG_INFO("timeb raw: {}", spdlog::to_hex(byte_span));

  auto next_msg_result = next_message(byte_span);
  ASSERT_TRUE(next_msg_result);

  MessageHeader header = std::get<0>(next_msg_result.value());
  ASSERT_EQ(header.message_id, MessageId::TIME);
  ASSERT_EQ(header.header_length, kMessageHeaderSize);

  size_t msg_start_idx = std::get<1>(next_msg_result.value());
  size_t msg_end_idx = std::get<2>(next_msg_result.value());

  EXPECT_EQ(msg_start_idx, kMessageHeaderSize);
  EXPECT_EQ(msg_end_idx, byte_span.size());

  TIME msg{};
  auto get_log_result = get_log(byte_span.subspan(msg_start_idx, (msg_end_idx - msg_start_idx)), msg);
  ASSERT_TRUE(get_log_result);

  SPDLOG_INFO("{}", msg);
}

TEST_F(TestOem7, partial_packet_at_beginning_of_buffer) {
  std::vector<uint8_t> raw;
  raw.insert(raw.end(), raw_hw_monitor_msg_.begin() + 40, raw_hw_monitor_msg_.end());
  raw.insert(raw.end(), raw_range_cmp_msg_.begin(), raw_range_cmp_msg_.end());

  std::span byte_span{reinterpret_cast<std::byte*>(raw.data()), raw.size()};

  auto next_msg_result = next_message(byte_span);
  ASSERT_TRUE(next_msg_result);

  MessageHeader header = std::get<0>(next_msg_result.value());
  ASSERT_EQ(header.message_id, MessageId::RANGECMP);
  ASSERT_EQ(header.header_length, kMessageHeaderSize);

  size_t msg_start_idx = std::get<1>(next_msg_result.value());
  size_t msg_end_idx = std::get<2>(next_msg_result.value());

  EXPECT_EQ(msg_start_idx, 96);
  EXPECT_EQ(msg_end_idx, byte_span.size());

  RANGECMP msg{};
  auto get_log_result = get_log(byte_span.subspan(msg_start_idx, (msg_end_idx - msg_start_idx)), msg);
  ASSERT_TRUE(get_log_result);

  SPDLOG_INFO("{}", msg);
}

TEST_F(TestOem7, incomplete_packet) {
  std::vector<uint8_t> raw;
  raw.insert(raw.end(), raw_hw_monitor_msg_.begin() + 40, raw_hw_monitor_msg_.end());
  raw.insert(raw.end(), raw_range_cmp_msg_.begin(), raw_range_cmp_msg_.end() - 20);

  std::span byte_span{reinterpret_cast<std::byte*>(raw.data()), raw.size()};

  auto next_msg_result = next_message(byte_span);
  ASSERT_FALSE(next_msg_result);

  ASSERT_EQ(next_msg_result.error().first, 68);
  ASSERT_EQ(next_msg_result.error().second, std::errc::no_buffer_space);

  std::rotate(raw.begin(), raw.begin() + next_msg_result.error().first, raw.end());
  raw.resize(raw.size() - (next_msg_result.error().first));
  raw.insert(raw.end(), raw_range_cmp_msg_.end() - 20, raw_range_cmp_msg_.end());

  byte_span = {reinterpret_cast<std::byte*>(raw.data()), raw.size()};
  next_msg_result = next_message(byte_span);
  ASSERT_TRUE(next_msg_result);

  MessageHeader header = std::get<0>(next_msg_result.value());
  ASSERT_EQ(header.message_id, MessageId::RANGECMP);
  ASSERT_EQ(header.header_length, kMessageHeaderSize);

  size_t msg_start_idx = std::get<1>(next_msg_result.value());
  size_t msg_end_idx = std::get<2>(next_msg_result.value());

  EXPECT_EQ(msg_start_idx, kMessageHeaderSize);
  EXPECT_EQ(msg_end_idx, byte_span.size());

  RANGECMP msg{};
  auto get_log_result = get_log(byte_span.subspan(msg_start_idx, (msg_end_idx - msg_start_idx)), msg);
  ASSERT_TRUE(get_log_result);
}

TEST(TestCRC, Works) {
  // OEM7 docs example.
  std::array buffer = {
      std::byte{0xAA}, std::byte{0x44}, std::byte{0x12}, std::byte{0x1C}, std::byte{0x2A},
      std::byte{0x00}, std::byte{0x02}, std::byte{0x20}, std::byte{0x48}, std::byte{0x00},
      std::byte{0x00}, std::byte{0x00}, std::byte{0x90}, std::byte{0xB4}, std::byte{0x93},
      std::byte{0x05}, std::byte{0xB0}, std::byte{0xAB}, std::byte{0xB9}, std::byte{0x12},
      std::byte{0x00}, std::byte{0x00}, std::byte{0x00}, std::byte{0x00}, std::byte{0x45},
      std::byte{0x61}, std::byte{0xBC}, std::byte{0x0A}, std::byte{0x00}, std::byte{0x00},
      std::byte{0x00}, std::byte{0x00}, std::byte{0x10}, std::byte{0x00}, std::byte{0x00},
      std::byte{0x00}, std::byte{0x1B}, std::byte{0x04}, std::byte{0x50}, std::byte{0xB3},
      std::byte{0xF2}, std::byte{0x8E}, std::byte{0x49}, std::byte{0x40}, std::byte{0x16},
      std::byte{0xFA}, std::byte{0x6B}, std::byte{0xBE}, std::byte{0x7C}, std::byte{0x82},
      std::byte{0x5C}, std::byte{0xC0}, std::byte{0x00}, std::byte{0x60}, std::byte{0x76},
      std::byte{0x9F}, std::byte{0x44}, std::byte{0x9F}, std::byte{0x90}, std::byte{0x40},
      std::byte{0xA6}, std::byte{0x2A}, std::byte{0x82}, std::byte{0xC1}, std::byte{0x3D},
      std::byte{0x00}, std::byte{0x00}, std::byte{0x00}, std::byte{0x12}, std::byte{0x5A},
      std::byte{0xCB}, std::byte{0x3F}, std::byte{0xCD}, std::byte{0x9E}, std::byte{0x98},
      std::byte{0x3F}, std::byte{0xDB}, std::byte{0x66}, std::byte{0x40}, std::byte{0x40},
      std::byte{0x00}, std::byte{0x30}, std::byte{0x30}, std::byte{0x30}, std::byte{0x00},
      std::byte{0x00}, std::byte{0x00}, std::byte{0x00}, std::byte{0x00}, std::byte{0x00},
      std::byte{0x00}, std::byte{0x00}, std::byte{0x0B}, std::byte{0x0B}, std::byte{0x00},
      std::byte{0x00}, std::byte{0x00}, std::byte{0x06}, std::byte{0x00}, std::byte{0x03},
      std::byte{0x42}, std::byte{0xDC}, std::byte{0x4C}, std::byte{0x48}};
  ASSERT_TRUE(check_crc(buffer));
}
};