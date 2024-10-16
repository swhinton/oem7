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

#include <array>
#include <cstdint>

#include "MessageIds.hpp"

namespace novatel::oem7 {
enum class SolutionStatus : uint32_t {
  kSolutionComputed,
  kInsufficientObservations,
  kNoConvergence,
  kSingularity,
  kCovarinaceTraceExceedsMaximum,
  kTestDistanceExceeded,
  kColdStart,
  kVHLimit,
  kVarianceExceedsLimits,
  kResidualsTooLarge,
  kLargeResiduals = 13,
  kPending = 18,
  kInvalidFix,
  kUnauthorized,
  kInvalidRate = 22
};
enum class SolutionType : uint32_t {
  kNone,
  kFixedPos,
  kFixedHeight,
  kDopplerVelocity = 8,
  kSingle = 16,
  kPsrDiff,
  kWAAS,
  kPropagated,
  kL1Float = 32,
  kNarrowFloat = 34,
  kL1Int = 48,
  kWideInt,
  kNarrowInt,
  kRtkDirectIns,
  kInsSbas,
  kInsPsrsp,
  kInsPsrDiff,
  kInsRtkFloat,
  kInsRtkFixed,
  kPppConverging = 68,
  kPpp,
  kOperational,
  kWarning,
  kOutOfBounds,
  kInsPppConverging,
  kInsPpp,
  kPppBasicConverging = 77,
  kPppBasic,
  kInsPppBasicConverging,
  kInsPppBasic
};

struct BESTXYZ {
  static constexpr MessageId kMessageId = MessageId::BESTXYZ;
  SolutionStatus position_solution_status{};
  SolutionType position_type{};
  double r_x_m{};
  double r_y_m{};
  double r_z_m{};
  float r_x_stdev_m{};
  float r_y_stdev_m{};
  float r_z_stdev_m{};
  SolutionStatus velocity_solution_status{};
  SolutionType velocity_type{};
  double v_x_mps{};
  double v_y_mps{};
  double v_z_mps{};
  float v_x_stdev_mps{};
  float v_y_stdev_mps{};
  float v_z_stdev_mps{};
  std::array<char, 4> base_station_id;
  float v_latency_s{};
  float differential_age_s{};
  float solution_age_s{};
  uint8_t num_sats_tracked{};
  uint8_t num_sats_in_solution{};
  uint8_t num_sats_with_l1_e1_b1_signals_used_in_solution{};
  uint8_t num_sats_multi_freq_used_in_solution{};
  uint8_t reserved0;
  // These are obnoxious... interpret them elsewhere if important:
  uint8_t extended_solution_status{};
  uint8_t galilelo_beidou_signal_mask{};
  uint8_t gps_glonass_signal_mask{};
  uint32_t crc{};
};
};
