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
// TODO RxStat error code masks
struct ElementRxStatus {
  uint32_t rx_status_word{};
  uint32_t rx_priority_mask{};
  uint32_t rx_event_set_mask{};
  uint32_t rx_event_clear_mask{};
};

struct RxStatus {
  static constexpr MessageId kMessageId = MessageId::RXSTATUS;
  uint32_t error{};
  uint32_t num_stats{};
  std::vector<ElementRxStatus> rx_status{};
  uint32_t crc{};
};

}