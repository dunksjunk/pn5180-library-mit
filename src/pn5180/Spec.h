// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Rich Dunkel
//
// PN5180 constants taken from NXP documentation. Portable core header.
//
// Citations:
//   DS      = PN5180A0xx/C3,C4 product data sheet, Rev. 4.3 (30 October 2025)
//   AN12650 = "Using the PN5180 without library", Rev. 1.0 (7 January 2020)
#pragma once

#include <cstddef>
#include <cstdint>

namespace pn5180 {
namespace spec {

// --------------------------------------------------------------------------
// Physical host interface (DS 11.4.1, 11.4.2)
// --------------------------------------------------------------------------
// "The maximum SPI speed is 7 Mbit/s and fixed to CPOL = 0 and CPHA = 0."  DS 11.4.1
constexpr uint32_t kSpiMaxClockHz = 7000000u;
// MOSI and MISO transfer the most significant bit first.                     DS 11.4.1
// Byte clocked out by the host while reading a response ("FF (data ignored)",
// Host TX "0xFF..." in the read frame).                                     DS 11.4.1 Fig. 6, Fig. 9
constexpr uint8_t kReadFillerByte = 0xFF;
// NSS HIGH time between frames, min 72 ns (GPIO toggling on supported hosts
// is far slower, so no explicit delay is inserted).                         DS 11.4.2 timing table

// --------------------------------------------------------------------------
// Reset and start-up (DS 11.2.2, Table 129, Table 141)
// --------------------------------------------------------------------------
// t(reset): RESET_N low for at least 10 us starts the reset.               DS 11.2.2, Table 129
constexpr uint32_t kResetLowMinUs = 10u;
// tboot: typ. 2.5 ms after RESET_N high; maximum depends on XTAL_BOOT_TIME
// in EEPROM, so the end of boot is detected by the IDLE IRQ.               DS Table 141, DS 11.2.2
constexpr uint32_t kBootTypicalMs = 3u;  // 2.5 ms typ. rounded up to whole ms

// --------------------------------------------------------------------------
// Host interface commands (DS 11.4.3.3 Table 5; AN12650 3.1-3.8)
// --------------------------------------------------------------------------
namespace cmd {
constexpr uint8_t WRITE_REGISTER = 0x00;           // DS Table 5, Table 6; AN12650 3.1
constexpr uint8_t WRITE_REGISTER_OR_MASK = 0x01;   // DS Table 5, Table 7; AN12650 3.2
constexpr uint8_t WRITE_REGISTER_AND_MASK = 0x02;  // DS Table 5, Table 8; AN12650 3.3
constexpr uint8_t READ_REGISTER = 0x04;            // DS Table 5, Table 10
constexpr uint8_t WRITE_EEPROM = 0x06;             // DS Table 5, Table 12
constexpr uint8_t READ_EEPROM = 0x07;              // DS Table 5, Table 13
constexpr uint8_t SEND_DATA = 0x09;                // DS Table 5, Table 15; AN12650 3.4
constexpr uint8_t READ_DATA = 0x0A;                // DS Table 5, Table 17; AN12650 3.5
constexpr uint8_t LOAD_RF_CONFIG = 0x11;           // DS Table 5, Table 31; AN12650 3.6
constexpr uint8_t RF_ON = 0x16;                    // DS Table 5, Table 37; AN12650 3.7
constexpr uint8_t RF_OFF = 0x17;                   // DS Table 5, Table 38; AN12650 3.8
}  // namespace cmd

// Command parameter limits
constexpr size_t kRegisterValueBytes = 4;         // 32-bit, little endian      DS 11.4.3.1, Table 6
constexpr uint8_t kEepromMaxAddress = 254;        // READ/WRITE_EEPROM addr 0..254  DS Table 12, Table 13
constexpr size_t kEepromMaxTransfer = 255;        // 1..255 bytes per command    DS Table 12, Table 13
constexpr size_t kSendDataMaxBytes = 260;         // Tx data 0..260 bytes        DS Table 15, 11.4.3.2
constexpr uint8_t kSendDataMaxValidBits = 7;      // valid bits in last byte 0..7  DS Table 15, Table 16
constexpr uint8_t kReadDataParam = 0x00;          // READ_DATA parameter         DS Table 17
constexpr size_t kReadDataMaxBytes = 508;         // reception buffer size       DS Table 17, 11.4.3.2
constexpr uint8_t kRfConfigTxMax = 0x1C;          // Transmitter config 0x00..0x1C  DS Table 31 condition
constexpr uint8_t kRfConfigRxMin = 0x80;          // Receiver config 0x80..0x9C  DS Table 31 condition
constexpr uint8_t kRfConfigRxMax = 0x9C;          //                             DS Table 31 condition
constexpr uint8_t kRfConfigUnchanged = 0xFF;      // keep current configuration  DS Table 31 description
constexpr uint8_t kRfOnDisableCollisionAvoidance = 0x01;  // RF_ON bit0           DS Table 37
constexpr uint8_t kRfOnActiveCommunication = 0x02;        // RF_ON bit1           DS Table 37
constexpr uint8_t kRfOffDummy = 0x00;             // "dummy byte, any value accepted"  DS Table 38

// --------------------------------------------------------------------------
// Registers (DS 11.9.1 Table 74; addresses confirmed by DS 11.9.2 Tables 75-118)
// --------------------------------------------------------------------------
namespace reg {
constexpr uint8_t SYSTEM_CONFIG = 0x00;              // DS Table 75
constexpr uint8_t IRQ_ENABLE = 0x01;                 // DS Table 76
constexpr uint8_t IRQ_STATUS = 0x02;                 // DS Table 77
constexpr uint8_t IRQ_CLEAR = 0x03;                  // DS Table 78
constexpr uint8_t TRANSCEIVE_CONTROL = 0x04;         // DS Table 79
constexpr uint8_t PADCONFIG = 0x05;                  // DS Table 80
// 0x06 is RFU                                         DS Table 74
constexpr uint8_t PAD_OUT = 0x07;                    // DS Table 81
constexpr uint8_t TIMER0_STATUS = 0x08;              // DS Table 82
constexpr uint8_t TIMER1_STATUS = 0x09;              // DS Table 83
constexpr uint8_t TIMER2_STATUS = 0x0A;              // DS Table 84
constexpr uint8_t TIMER0_RELOAD = 0x0B;              // DS Table 85
constexpr uint8_t TIMER1_RELOAD = 0x0C;              // DS Table 86
constexpr uint8_t TIMER2_RELOAD = 0x0D;              // DS Table 87
constexpr uint8_t TIMER0_CONFIG = 0x0E;              // DS Table 88
constexpr uint8_t TIMER1_CONFIG = 0x0F;              // DS Table 89
constexpr uint8_t TIMER2_CONFIG = 0x10;              // DS Table 90
constexpr uint8_t RX_WAIT_CONFIG = 0x11;             // DS Table 91
constexpr uint8_t CRC_RX_CONFIG = 0x12;              // DS Table 92
constexpr uint8_t RX_STATUS = 0x13;                  // DS Table 93
constexpr uint8_t TX_UNDERSHOOT_CONFIG = 0x14;       // DS Table 94
constexpr uint8_t TX_OVERSHOOT_CONFIG = 0x15;        // DS Table 95
constexpr uint8_t TX_DATA_MOD = 0x16;                // DS Table 96
constexpr uint8_t TX_WAIT_CONFIG = 0x17;             // DS Table 97
constexpr uint8_t TX_CONFIG = 0x18;                  // DS Table 98
constexpr uint8_t CRC_TX_CONFIG = 0x19;              // DS Table 99
constexpr uint8_t SIGPRO_CONFIG = 0x1A;              // DS Table 100
constexpr uint8_t SIGPRO_CM_CONFIG = 0x1B;           // DS Table 101
constexpr uint8_t SIGPRO_RM_CONFIG = 0x1C;           // DS Table 102
constexpr uint8_t RF_STATUS = 0x1D;                  // DS Table 103
constexpr uint8_t AGC_CONFIG = 0x1E;                 // DS Table 104
constexpr uint8_t AGC_VALUE = 0x1F;                  // DS Table 105
constexpr uint8_t RF_CONTROL_TX = 0x20;              // DS Table 106
constexpr uint8_t RF_CONTROL_TX_CLK = 0x21;          // DS Table 107
constexpr uint8_t RF_CONTROL_RX = 0x22;              // DS Table 108
constexpr uint8_t LD_CONTROL = 0x23;                 // DS Table 109
constexpr uint8_t SYSTEM_STATUS = 0x24;              // DS Table 110
constexpr uint8_t TEMP_CONTROL = 0x25;               // DS Table 111
constexpr uint8_t AGC_REF_CONFIG = 0x26;             // DS Table 112
constexpr uint8_t DPC_CONFIG = 0x27;                 // DS Table 113
constexpr uint8_t EMD_CONTROL = 0x28;                // DS Table 114
constexpr uint8_t ANT_CONTROL = 0x29;                // DS Table 115
constexpr uint8_t TX_CONTROL = 0x36;                 // DS Table 116
constexpr uint8_t SIGPRO_RM_CONFIG_EXTENSION = 0x39; // DS Table 117
constexpr uint8_t FELICA_EMD_CONTROL = 0x43;         // DS Table 118 (firmware >= 4.1)

// Register commands raise an exception for a non-existent address
// (DS Tables 6-10 "Condition"), so the driver rejects undocumented ones.
constexpr bool isDocumented(uint8_t a) {
    return (a <= ANT_CONTROL && a != 0x06) || a == TX_CONTROL || a == SIGPRO_RM_CONFIG_EXTENSION ||
           a == FELICA_EMD_CONTROL;
}

// Registers whose every bit is documented with Access "R". Writing them is
// rejected by the driver. (No register is entirely write-only: e.g. IRQ_CLEAR
// bits are R/W per DS Table 78, SYSTEM_CONFIG mixes W and R/W per Table 75.)
constexpr bool isReadOnly(uint8_t a) {
    return a == IRQ_STATUS        // DS Table 77
           || a == TIMER0_STATUS  // DS Table 82
           || a == TIMER1_STATUS  // DS Table 83
           || a == TIMER2_STATUS  // DS Table 84
           || a == RX_STATUS      // DS Table 93
           || a == RF_STATUS      // DS Table 103
           || a == SYSTEM_STATUS; // DS Table 110
}

// FELICA_EMD_CONTROL exists only from firmware 4.1 (major 4, minor 1).  DS Table 74, Table 118
constexpr uint8_t kFelicaEmdMinFwMajor = 4;
constexpr uint8_t kFelicaEmdMinFwMinor = 1;
}  // namespace reg

// SYSTEM_CONFIG bits
constexpr uint32_t kSystemConfigSoftReset = 1u << 8;  // SOFT_RESET              DS 11.2.1.2, Table 75
// SYSTEM_CONFIG.COMMAND (bits 2:0): transceive state machine command.       DS Table 75
constexpr uint32_t kSystemConfigCommandMask = 0x7u;   // bits 2:0              DS Table 75
constexpr uint32_t kCommandIdle = 0x0u;               // IDLE/StopCom          DS Table 75; AN12650 4.1.3 line 6
constexpr uint32_t kCommandTransceive = 0x3u;         // Transceive            DS Table 75, Table 15; AN12650 4.1.3 line 7

// CRC_TX_CONFIG bit 0 TX_CRC_ENABLE: Tx-Encoder computes and appends CRC.   DS Table 99; AN12650 4.1.3 line 3
constexpr uint32_t kCrcTxEnable = 1u << 0;
// CRC_RX_CONFIG bit 0 RX_CRC_ENABLE: Rx-Decoder checks the CRC.             DS Table 92; AN12650 4.1.3 line 4
constexpr uint32_t kCrcRxEnable = 1u << 0;

// RX_STATUS fields (DS Table 93)
namespace rxstatus {
constexpr uint32_t NUM_BYTES_MASK = 0x1FFu;           // bits 8:0 RX_NUM_BYTES_RECEIVED
constexpr unsigned NUM_LAST_BITS_SHIFT = 13;          // bits 15:13 RX_NUM_LAST_BITS
constexpr uint32_t NUM_LAST_BITS_MASK = 0x7u;
constexpr uint32_t DATA_INTEGRITY_ERROR = 1u << 16;   // wrong parity or CRC
constexpr uint32_t PROTOCOL_ERROR = 1u << 17;         // stop bit / SOF / EOF / length
constexpr uint32_t COLLISION_DETECTED = 1u << 18;     // collision in received frame
constexpr unsigned COLL_POS_SHIFT = 19;               // bits 25:19 RX_COLL_POS
constexpr uint32_t COLL_POS_MASK = 0x7Fu;
}  // namespace rxstatus

// RF_STATUS fields (DS Table 103)
namespace rfstatus {
constexpr uint32_t TX_ACTIVE = 1u << 11;              // TxEncoder transmitting
constexpr uint32_t RX_ENABLE = 1u << 12;              // RxDecoder enabled by the Transceive unit
constexpr unsigned ACTIVE_ERROR_CAUSE_SHIFT = 13;     // bits 15:13 RF_ACTIVE_ERROR_CAUSE
constexpr uint32_t ACTIVE_ERROR_CAUSE_MASK = 0x7u;
constexpr uint32_t RF_DET_STATUS = 1u << 16;          // external RF field detected
constexpr uint32_t TX_RF_STATUS = 1u << 17;           // own RF field (drivers) on
constexpr unsigned TRANSCEIVE_STATE_SHIFT = 24;       // bits 26:24 TRANSCEIVE_STATE
constexpr uint32_t TRANSCEIVE_STATE_MASK = 0x7u;      // 0 IDLE, 1 WaitTransmit, ... 5 Receiving
}  // namespace rfstatus

// LOAD_RF_CONFIG indexes for ISO/IEC 14443-A 106 kbit/s.     DS Table 32; AN12650 4.1.3 line 1
constexpr uint8_t kRfConfigIso14443a106Tx = 0x00;
constexpr uint8_t kRfConfigIso14443a106Rx = 0x80;

// RF_CONTROL_RX bits 1:0 RX_GAIN: 0 = 33 dB, 1 = 40 dB, 2 = 50 dB, 3 = 57 dB.  DS Table 108
constexpr uint32_t kRfControlRxGainMask = 0x3u;
// RX_GAIN of the documented ISO 14443-A 106 receiver default RF_CONTROL_RX
// = 0x1E (bits 1:0 = 10b).                                   DS Table 172, Table 108
constexpr uint32_t kRxGainIso14443a106 = 0x2u;
// ISO 14443-A 106 receiver SIGPRO_RM_CONFIG: the documented default 0x430DC
// (DS Table 172) with MIN_LEVEL (bits 15:12) raised from 3 to 5; all other
// fields unchanged. DS Table 102: "The MinLevel should be higher than the
// noise level in the system." Measured on the reference test bed (NTAG215 +
// two ISO 15693 cards stacked, 110 interleaved activations per setting,
// 2026-11-04): MIN_LEVEL 3 failed 7/110 (garbage frames flagged as collision
// / parity errors), MIN_LEVEL 5 failed 0/110.            DS Table 102, Table 172
constexpr uint32_t kSigproMinLevelMask = 0xF000u;           // MIN_LEVEL  DS Table 102
constexpr uint32_t kSigproMinLevelShift = 12;               // DS Table 102
constexpr uint32_t kSigproMinLevelIso14443a106 = 5;         // measured, see above
constexpr uint32_t kSigproRmConfigIso14443a106 =
    (0x430DCu & ~kSigproMinLevelMask) | (kSigproMinLevelIso14443a106 << kSigproMinLevelShift);
static_assert(kSigproRmConfigIso14443a106 == 0x450DCu, "SIGPRO_RM_CONFIG for 14443-A");

// LOAD_RF_CONFIG indexes for ISO/IEC 15693 ASK100, 26 kbit/s.  DS Table 32; AN12650 4.2.3 line 1
constexpr uint8_t kRfConfigIso15693Ask100Tx = 0x0D;
constexpr uint8_t kRfConfigIso15693Ask100Rx = 0x8D;

// TX_CONFIG fields.                                          DS Table 98
namespace txconfig {
constexpr uint32_t START_SYMBOL_MASK = 0x3u << 6;  // bits 7:6 TX_START_SYMBOL
constexpr uint32_t STOP_SYMBOL_MASK = 0x3u << 8;   // bits 9:8 TX_STOP_SYMBOL
constexpr uint32_t DATA_ENABLE = 1u << 10;         // bit 10 TX_DATA_ENABLE
// AND mask that leaves only the stop symbol (EOF) to be sent by a
// zero-length SEND_DATA: clears TX_START_SYMBOL and TX_DATA_ENABLE.
// Equals AN12650's 0xFFFFFB3F.                               AN12650 4.2.3 line 11; DS Table 15
constexpr uint32_t kEofOnlyAndMask = ~(START_SYMBOL_MASK | DATA_ENABLE);
}  // namespace txconfig

// ISO/IEC 15693 card -> reader bit duration at 26.48 kbit/s (single
// subcarrier, Manchester): 37.76 us.                         DS Table 65
constexpr uint32_t kIso15693BitNs = 37760u;

// --------------------------------------------------------------------------
// IRQ_STATUS bits (DS Table 77). IRQ_CLEAR (DS Table 78) and IRQ_ENABLE
// (DS Table 76) use the same bit positions.
// --------------------------------------------------------------------------
namespace irq {
constexpr uint32_t RX = 1u << 0;               // End of RF reception
constexpr uint32_t TX = 1u << 1;               // End of RF transmission
constexpr uint32_t IDLE = 1u << 2;             // IDLE (also: boot finished, DS 11.2.2)
constexpr uint32_t MODE_DETECTED = 1u << 3;    // External modulation scheme detected
constexpr uint32_t CARD_ACTIVATED = 1u << 4;   // Activated as a card
constexpr uint32_t STATE_CHANGE = 1u << 5;     // Transceive state machine state change
constexpr uint32_t RFOFF_DET = 1u << 6;        // RF field OFF detected
constexpr uint32_t RFON_DET = 1u << 7;         // RF field ON detected
constexpr uint32_t TX_RFOFF = 1u << 8;         // RF field OFF in PCD
constexpr uint32_t TX_RFON = 1u << 9;          // RF field ON in PCD
constexpr uint32_t RF_ACTIVE_ERROR = 1u << 10; // RF active error
constexpr uint32_t TIMER0 = 1u << 11;          // Timer0
constexpr uint32_t TIMER1 = 1u << 12;          // Timer1
constexpr uint32_t TIMER2 = 1u << 13;          // Timer2
constexpr uint32_t RX_SOF_DET = 1u << 14;      // RX SOF detection
constexpr uint32_t RX_SC_DET = 1u << 15;       // RX subcarrier detection
constexpr uint32_t TEMPSENS_ERROR = 1u << 16;  // Temperature sensor
constexpr uint32_t GENERAL_ERROR = 1u << 17;   // General error (command exceptions, DS 11.4.1)
constexpr uint32_t HV_ERROR = 1u << 18;        // EEPROM failure during programming
constexpr uint32_t LPCD = 1u << 19;            // Low-power card detection
constexpr uint32_t ALL = 0x000FFFFFu;          // bits 19:0; 31:20 are RFU
}  // namespace irq

// --------------------------------------------------------------------------
// EEPROM map (DS 11.5.2 Table 51)
// --------------------------------------------------------------------------
namespace eeprom {
constexpr uint8_t DIE_IDENTIFIER = 0x00;    // 16 bytes, R    DS Table 51
constexpr uint8_t PRODUCT_VERSION = 0x10;   // 2 bytes, R     DS Table 51
constexpr uint8_t FIRMWARE_VERSION = 0x12;  // 2 bytes, R     DS Table 51
constexpr uint8_t EEPROM_VERSION = 0x14;    // 2 bytes, R     DS Table 51
constexpr size_t kDieIdentifierBytes = 16;  //                DS Table 51
constexpr size_t kVersionBytes = 2;         //                DS Table 51
// Version byte order: lower address = minor, higher address = major
// (e.g. FW 4.1: 0x12 = 0x01, 0x13 = 0x04).                  DS Table 51
constexpr size_t kVersionMinorOffset = 0;
constexpr size_t kVersionMajorOffset = 1;
// 0x00..0x15 are documented read-only (Access "R"); the first RW entry is
// 0x16 IDLE_IRQ_AFTER_BOOT.                                 DS Table 51
constexpr uint8_t kFirstWritableAddress = 0x16;
}  // namespace eeprom

}  // namespace spec
}  // namespace pn5180
