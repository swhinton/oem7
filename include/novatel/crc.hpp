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

// This is straight out of the ICD... consider using the CRC lib instead, it's probably a lot faster.

namespace novatel::oem7 {
constexpr uint32_t kCrc32Polynomial = 0xEDB88320L;

using CRC_t = uint32_t;

/* --------------------------------------------------------------------------
Calculate a CRC value to be used by CRC calculation functions.
-------------------------------------------------------------------------- */
inline CRC_t CRC32Value(const int i) {
  CRC_t ulCRC = i;
  for (int j = 8; j > 0; j--) {
    if (ulCRC & 1)
      ulCRC = (ulCRC >> 1) ^ kCrc32Polynomial;
    else
      ulCRC >>= 1;
  }
  return ulCRC;
}

/* --------------------------------------------------------------------------
Calculates the CRC-32 of a block of data all at once
ulCount - Number of bytes in the data block
ucBuffer - Data block
-------------------------------------------------------------------------- */
inline CRC_t CalculateBlockCRC32(unsigned long ulCount, const unsigned char *ucBuffer) {
  CRC_t ulCRC = 0;
  while (ulCount-- != 0) {
    unsigned long ulTemp1 = (ulCRC >> 8) & 0x00FFFFFFL;
    unsigned long ulTemp2 = CRC32Value(((int)ulCRC ^ *ucBuffer++) & 0xFF);
    ulCRC = ulTemp1 ^ ulTemp2;
  }
  return ulCRC;
}
}  // namespace novatel::oem7
