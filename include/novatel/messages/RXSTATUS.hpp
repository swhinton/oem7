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

#include <bitset>
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

struct ReceiverError {
  std::bitset<32> status_word{};
  bool DRAMError() { return status_word[0]; };
  bool InvalidFirmware() { return status_word[1]; };
  bool ROMError() { return status_word[2]; };
  bool RxErrorReserved0() { return status_word[3]; };
  bool ESNAccessError() { return status_word[4]; };
  bool AuthCodeStatus() { return status_word[5]; };
  bool RxErrorReserved1() { return status_word[6]; };
  bool SupplyVoltageError() { return status_word[7]; };
  bool RxErrorReserved2() { return status_word[8]; };
  bool TemperatureError() { return status_word[9]; };
  bool MINOSError() { return status_word[10]; };
  bool PLLRFError() { return status_word[11]; };
  bool RxErrorReserved3() { return status_word[12]; };
  bool RxErrorReserved4() { return status_word[13]; };
  bool RxErrorReserved5() { return status_word[14]; };
  bool NVMError() { return status_word[15]; };
  bool SoftwareResourceLimitExceeded() { return status_word[16]; };
  bool ModelInvalidForReceiver() { return status_word[17]; };
  bool RxErrorReserved6() { return status_word[18]; };
  bool RxErrorReserved7() { return status_word[19]; };
  bool RemoteLoadingBegun() { return status_word[20]; };
  bool ExportRestrictionError() { return status_word[21]; };
  bool SafeModeError() { return status_word[22]; };
  bool RxErrorReserved8() { return status_word[23]; };
  bool RxErrorReserved15() { return status_word[24]; };
  bool RxErrorReserved9() { return status_word[25]; };
  bool RxErrorReserved10() { return status_word[26]; };
  bool RxErrorReserved11() { return status_word[27]; };
  bool RxErrorReserved12() { return status_word[28]; };
  bool RxErrorReserved13() { return status_word[29]; };
  bool RxErrorReserved14() { return status_word[30]; };
  bool HardwareFailure() { return status_word[31]; };
};

struct ReceiverStatus {
  std::bitset<32> status_word{};
  bool ErrorFlag() { return status_word[0]; }
  bool TemperatureWarning() { return status_word[1]; }
  bool VoltageSupplyWarning() { return status_word[2]; }
  bool PrimaryAntennaPowered() { return status_word[3]; }
  bool LNAFailure() { return status_word[4]; }
  bool PrimaryAntennaDisconnected() { return status_word[5]; }
  bool PrimaryAntennaShort() { return status_word[6]; }
  bool CPUOverload() { return status_word[7]; }
  bool COMBufferOverrun() { return status_word[8]; }
  bool SpoofingDetected() { return status_word[9]; }
  bool RxStatusReserved0() { return status_word[10]; }
  bool LinkOverrun() { return status_word[11]; }
  bool InputOverrun() { return status_word[12]; }
  bool AuxTransmitOverrun() { return status_word[13]; }
  bool AntennaGainOutOfRange() { return status_word[14]; }
  bool JammerDetected() { return status_word[15]; }
  bool INSReset() { return status_word[16]; }
  bool IMUCommFailure() { return status_word[17]; }
  bool GPSAlmanacUtcUnknown() { return status_word[18]; }
  bool PositionSolutionInvalid() { return status_word[19]; }
  bool PositionFixed() { return status_word[20]; }
  bool ClockSteeringDisabled() { return status_word[21]; }
  bool ClockModelInvalid() { return status_word[22]; }
  bool ExternalOscillatorLocked() { return status_word[23]; }
  bool SoftwareResourceWarning() { return status_word[24]; }
  bool VersionBit0() { return status_word[25]; }
  bool VersionBit1() { return status_word[26]; }
  bool HDRTracking() { return status_word[27]; }
  bool DigitalFiltringEnabled() { return status_word[28]; }
  bool Aux3Event() { return status_word[29]; }
  bool Aux2Event() { return status_word[30]; }
  bool Aux1Event() { return status_word[31]; }
};

struct Auxiliary1Status {
  std::bitset<32> status_word{};
  bool JammerDetectedOnRf1() { return status_word[0]; }
  bool JammerDetectedOnRf2() { return status_word[1]; }
  bool JammerDetectedOnRf3() { return status_word[2]; }
  bool PositionAveragingOn() { return status_word[3]; }
  bool JammerDetectedOnRf4() { return status_word[4]; }
  bool JammerDetectedOnRf5() { return status_word[5]; }
  bool JammerDetectedOnRf6() { return status_word[6]; }
  bool USBNotConnected() { return status_word[7]; }
  bool USB1Overrun() { return status_word[8]; }
  bool USB2Overrun() { return status_word[9]; }
  bool USB3Overrun() { return status_word[10]; }
  bool Aux1StatReserved0() { return status_word[11]; }
  bool ProfileActivationBitError() { return status_word[12]; }
  bool ThrottledEthernetReception() { return status_word[13]; }
  bool Aux1StatReserved1() { return status_word[14]; }
  bool Aux1StatReserved2() { return status_word[15]; }
  bool Aux1StatReserved3() { return status_word[16]; }
  bool Aux1StatReserved4() { return status_word[17]; }
  bool EthernetNotConnected() { return status_word[18]; }
  bool ICOM1BufferOverrun() { return status_word[19]; }
  bool ICOM2BufferOverrun() { return status_word[20]; }
  bool ICOM3BufferOverrun() { return status_word[21]; }
  bool NCOM1BufferOverrun() { return status_word[22]; }
  bool NCOM2BufferOverrun() { return status_word[23]; }
  bool NCOM3BufferOverrun() { return status_word[24]; }
  bool Aux1StatReserved5() { return status_word[25]; }
  bool Aux1StatReserved6() { return status_word[26]; }
  bool Aux1StatReserved7() { return status_word[27]; }
  bool Aux1StatReserved8() { return status_word[28]; }
  bool Aux1StatReserved9() { return status_word[29]; }
  bool StatusErrorReported() { return status_word[30]; }
  bool IMUOutlierDetected() { return status_word[31]; }
};

struct Auxiliary2Status {
  std::bitset<32> status_word{};
  bool SPICommFailure() { return status_word[0]; }
  bool I2CCommFailure() { return status_word[1]; }
  bool COM4BufferOverrun() { return status_word[2]; }
  bool COM5BufferOverrun() { return status_word[3]; }
  bool Aux2StatReserved0() { return status_word[4]; }
  bool Aux2StatReserved1() { return status_word[5]; }
  bool Aux2StatReserved2() { return status_word[6]; }
  bool Aux2StatReserved3() { return status_word[7]; }
  bool Aux2StatReserved4() { return status_word[8]; }
  bool COM1BufferOverrun() { return status_word[9]; }
  bool COM2BufferOverrun() { return status_word[10]; }
  bool COM3BufferOverrun() { return status_word[11]; }
  bool PLLRF1Unlock() { return status_word[12]; }
  bool PLLRF2Unlock() { return status_word[13]; }
  bool PLLRF3Unlock() { return status_word[14]; }
  bool PLLRF4Unlock() { return status_word[15]; }
  bool PLLRF5Unlock() { return status_word[16]; }
  bool PLLRF6Unlock() { return status_word[17]; }
  bool CCOM1BufferOverrun() { return status_word[18]; }
  bool CCOM2BufferOverrun() { return status_word[19]; }
  bool CCOM3BufferOverrun() { return status_word[20]; }
  bool CCOM4BufferOverrun() { return status_word[21]; }
  bool CCOM5BufferOverrun() { return status_word[22]; }
  bool CCOM6BufferOverrun() { return status_word[23]; }
  bool ICOM4BufferOverrun() { return status_word[24]; }
  bool ICOM5BufferOverrun() { return status_word[25]; }
  bool ICOM6BufferOverrun() { return status_word[26]; }
  bool ICOM7BufferOverrun() { return status_word[27]; }
  bool SecondaryAntennaNotPowered() { return status_word[28]; }
  bool SecondaryAntennaDisconnected() { return status_word[29]; }
  bool SecondaryAntennaShort() { return status_word[30]; }
  bool ResetDetected() { return status_word[31]; }
};

struct Auxiliary3Status {
  std::bitset<32> status_word{};
  bool SCOMBufferOverrun() { return status_word[0]; }
  bool WCOM1BufferOverrun() { return status_word[1]; }
  bool FILEBufferOverrun() { return status_word[2]; }
  bool Aux3StatReserved0() { return status_word[3]; }
  bool Antenna1GainState0() { return status_word[4]; }
  bool Antenna1GainState1() { return status_word[5]; }
  bool Antenna2GainState0() { return status_word[6]; }
  bool Antenna2GainState1() { return status_word[7]; }
  bool Aux3StatReserved4() { return status_word[8]; }
  bool Aux3StatReserved5() { return status_word[9]; }
  bool Aux3StatReserved6() { return status_word[10]; }
  bool Aux3StatReserved7() { return status_word[11]; }
  bool Aux3StatReserved8() { return status_word[12]; }
  bool Aux3StatReserved9() { return status_word[13]; }
  bool Aux3StatReserved10() { return status_word[14]; }
  bool Aux3StatReserved11() { return status_word[15]; }
  bool Aux3StatReserved12() { return status_word[16]; }
  bool Aux3StatReserved13() { return status_word[17]; }
  bool Aux3StatReserved14() { return status_word[18]; }
  bool Aux3StatReserved15() { return status_word[19]; }
  bool Aux3StatReserved16() { return status_word[20]; }
  bool Aux3StatReserved17() { return status_word[21]; }
  bool Aux3StatReserved18() { return status_word[22]; }
  bool Aux3StatReserved19() { return status_word[23]; }
  bool SpoofingCalFailed() { return status_word[24]; }
  bool SpoofingCalRequired() { return status_word[25]; }
  bool Aux3StatReserved20() { return status_word[26]; }
  bool Aux3StatReserved21() { return status_word[27]; }
  bool Aux3StatReserved22() { return status_word[28]; }
  bool WebContentCorruptOrDNE() { return status_word[29]; }
  bool RFCalDataHasAnError() { return status_word[30]; }
  bool RFCalDataPresent() { return status_word[31]; }
};

struct Auxiliary4Status {
  std::bitset<32> status_word{};
  bool LessThan60PctSatsTrackedWell() { return status_word[0]; }
  bool LessThan15PctSatsTrackedWell() { return status_word[1]; }
  bool Aux4StatReserved0() { return status_word[2]; }
  bool Aux4StatReserved1() { return status_word[3]; }
  bool Aux4StatReserved2() { return status_word[4]; }
  bool Aux4StatReserved3() { return status_word[5]; }
  bool Aux4StatReserved4() { return status_word[6]; }
  bool Aux4StatReserved5() { return status_word[7]; }
  bool Aux4StatReserved6() { return status_word[8]; }
  bool Aux4StatReserved7() { return status_word[9]; }
  bool Aux4StatReserved8() { return status_word[10]; }
  bool Aux4StatReserved9() { return status_word[11]; }
  bool ClockFreewheeling() { return status_word[12]; }
  bool Aux4StatReserved10() { return status_word[13]; }
  bool LessThan60PctRTKExpectedCorrectionsAvailable() { return status_word[14]; }
  bool LessThan15PctRTKExpectedCorrectionsAvailable() { return status_word[15]; }
  bool BadRTKGeometry() { return status_word[16]; }
  bool Aux4StatReserved11() { return status_word[17]; }
  bool Aux4StatReserved12() { return status_word[18]; }
  bool LongRTKBaseline() { return status_word[19]; }
  bool PoorRTKCOMLink() { return status_word[20]; }
  bool PoorALIGNCOMLink() { return status_word[21]; }
  bool GLIDNotActive() { return status_word[22]; }
  bool BadPDPGeometry() { return status_word[23]; }
  bool NoTerraStarSubscription() { return status_word[24]; }
  bool Aux4StatReserved13() { return status_word[25]; }
  bool Aux4StatReserved14() { return status_word[26]; }
  bool Aux4StatReserved15() { return status_word[27]; }
  bool BadPPPGeometry() { return status_word[28]; }
  bool Aux4StatReserved16() { return status_word[29]; }
  bool NoINSAlignment() { return status_word[30]; }
  bool INSNotConverged() { return status_word[31]; }
};

struct RxStatus {
  static constexpr MessageId kMessageId = MessageId::RXSTATUS;
  ReceiverError rx_error{};
  uint32_t num_stats{};

  ReceiverStatus receiver_status{};
  ReceiverStatus receiver_priority_mask{};
  ReceiverStatus receiver_event_set_mask{};
  ReceiverStatus receiver_event_clear_mask{};

  Auxiliary1Status aux1_status{};
  Auxiliary1Status aux1_priority_mask{};
  Auxiliary1Status aux1_event_set_mask{};
  Auxiliary1Status aux1_event_clear_mask{};

  Auxiliary2Status aux2_status{};
  Auxiliary2Status aux2_priority_mask{};
  Auxiliary2Status aux2_event_set_mask{};
  Auxiliary2Status aux2_event_clear_mask{};

  Auxiliary3Status aux3_status{};
  Auxiliary3Status aux3_priority_mask{};
  Auxiliary3Status aux3_event_set_mask{};
  Auxiliary3Status aux3_event_clear_mask{};

  Auxiliary4Status aux4_status{};
  Auxiliary4Status aux4_priority_mask{};
  Auxiliary4Status aux4_event_set_mask{};
  Auxiliary4Status aux4_event_clear_mask{};

  uint32_t crc{};
};

}