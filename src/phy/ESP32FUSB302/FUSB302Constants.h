//
// USB Power Delivery for Arduino
// Copyright (c) 2024 Manuel Bleichenbacher
//
// Licensed under MIT License
// https://opensource.org/licenses/MIT
//
// FUSB302 registers
//

#pragma once

#include <stdint.h>

namespace FUSB302 {

/// FUSB302 register addresses
struct Reg {
    static constexpr uint8_t DeviceId = 0x01;
    static constexpr uint8_t Switches0 = 0x02;
    static constexpr uint8_t Switches1 = 0x03;
    static constexpr uint8_t Measure = 0x04;
    static constexpr uint8_t Slice = 0x05;
    static constexpr uint8_t Control0 = 0x06;
    static constexpr uint8_t Control1 = 0x07;
    static constexpr uint8_t Control2 = 0x08;
    static constexpr uint8_t Control3 = 0x09;
    static constexpr uint8_t Mask = 0x0a;
    static constexpr uint8_t Power = 0x0b;
    static constexpr uint8_t Reset = 0x0c;
    static constexpr uint8_t OcpReg = 0x0d;
    static constexpr uint8_t MaskA = 0x0e;
    static constexpr uint8_t MaskB = 0x0f;
    static constexpr uint8_t Control4 = 0x10;
    static constexpr uint8_t Status0A = 0x3c;
    static constexpr uint8_t Status1A = 0x3d;
    static constexpr uint8_t InterruptA = 0x3e;
    static constexpr uint8_t InterruptB = 0x3f;
    static constexpr uint8_t Status0 = 0x40;
    static constexpr uint8_t Status1 = 0x41;
    static constexpr uint8_t Interrupt = 0x42;
    static constexpr uint8_t FIFOS = 0x43;
};

/// FUSB302 register SWITCHES0 values
struct Switches0 {
    static constexpr uint8_t PullUpEn2 = 0x01 << 7;
    static constexpr uint8_t PullUpEn1 = 0x01 << 6;
    static constexpr uint8_t VCONN_CC2 = 0x01 << 5;
    static constexpr uint8_t VCONN_CC1 = 0x01 << 4;
    static constexpr uint8_t MeasCC2 = 0x01 << 3;
    static constexpr uint8_t MeasCC1 = 0x01 << 2;
    static constexpr uint8_t PullDown2 = 0x01 << 1;
    static constexpr uint8_t PullDown1 = 0x01 << 0;
    static constexpr uint8_t None = 0x00;
};

/// FUSB302 register SWITCHES1 values
struct Switches1 {
    static constexpr uint8_t None = 0;
    static constexpr uint8_t PowerRole = 0x01 << 7;
    static constexpr uint8_t SpecRevMask = 0x03 << 5;
    static constexpr uint8_t SpecRevRev_1_0 = 0x00 << 5;
    static constexpr uint8_t SpecRevRev_2_0 = 0x01 << 5;
    static constexpr uint8_t DataRole = 0x01 << 4;
    static constexpr uint8_t AutoCRC = 0x01 << 2;
    static constexpr uint8_t TxCC2 = 0x01 << 1;
    static constexpr uint8_t TxCC1 = 0x01 << 0;
};

/// FUSB302 register MEASURE values
struct Measure {
    static constexpr uint8_t MeasVbus = 0x01 << 6;
    static constexpr uint8_t MeasMDACMask = 0x3f;
};

/// FUSB302 register SLICE values
struct Slice {
    static constexpr uint8_t SDACHysMask = 0x03 << 6;
    static constexpr uint8_t SDACHys_255mv = 0x03 << 6;
    static constexpr uint8_t SDACHys_170mv = 0x02 << 6;
    static constexpr uint8_t SDACHys_085mv = 0x01 << 6;
    static constexpr uint8_t SDACHys_None = 0x00 << 6;
    static constexpr uint8_t SDACMask = 0x3f << 0;
};

/// FUSB302 register CONTROL0 values
struct Control0 {
    static constexpr uint8_t TxFlush = 0x01 << 6;
    static constexpr uint8_t IntMask = 0x01 << 5;
    static constexpr uint8_t HostCurMask = 0x03 << 2;
    static constexpr uint8_t HostCur_No = 0x00 << 2;
    static constexpr uint8_t HostCur_USBDef = 0x01 << 2;
    static constexpr uint8_t HostCur_1_5A = 0x02 << 2;
    static constexpr uint8_t HostCur_3_0A = 0x03 << 2;
    static constexpr uint8_t AutoPre = 0x01 << 1;
    static constexpr uint8_t TxStart = 0x01 << 0;
    static constexpr uint8_t None = 0;
};

/// FUSB302 register CONTROL1 values
struct Control1 {
    static constexpr uint8_t EnSOP2Debug = 0x01 << 6;
    static constexpr uint8_t EnSOP1Debug = 0x01 << 5;
    static constexpr uint8_t BISTMode2 = 0x01 << 4;
    static constexpr uint8_t RxFlush = 0x01 << 2;
    static constexpr uint8_t EnSOP2 = 0x01 << 1;
    static constexpr uint8_t EnSOP1 = 0x01 << 0;
};

/// FUSB302 register CONTROL2 values
struct Control2 {
    static constexpr uint8_t TogSavePwrMask = 0x03 << 6;
    static constexpr uint8_t TogRdOnly = 0x01 << 5;
    static constexpr uint8_t WakeEn = 0x01 << 3;
    static constexpr uint8_t ModeMask = 0x03 << 1;
    static constexpr uint8_t ModeSrcPolling = 0x03 << 1;
    static constexpr uint8_t ModeSinkPolling = 0x02 << 1;
    static constexpr uint8_t ModeDRPPolling = 0x01 << 1;
    static constexpr uint8_t Toggle = 0x01 << 0;
};

/// FUSB302 register CONTROL3 values
struct Control3 {
    static constexpr uint8_t SendHardReset = 0x03 << 6;
    static constexpr uint8_t BISTMode = 0x01 << 5;
    static constexpr uint8_t AutoHardReset = 0x01 << 4;
    static constexpr uint8_t AutoSoftReset = 0x01 << 3;
    static constexpr uint8_t NRetriesMask = 0x03 << 1;
    static constexpr uint8_t NRetries_3 = 0x03 << 1;
    static constexpr uint8_t NRetries_2 = 0x02 << 1;
    static constexpr uint8_t NRetries_1 = 0x01 << 1;
    static constexpr uint8_t NRetries_0 = 0x00 << 1;
    static constexpr uint8_t AutoRetry = 0x01 << 0;
};

/// FUSB302 register MASK values
struct Mask {
    static constexpr uint8_t M_All = 0xff;
    static constexpr uint8_t M_VbusOk = 0x01 << 7;
    static constexpr uint8_t M_Activity = 0x01 << 6;
    static constexpr uint8_t M_CompChange = 0x01 << 5;
    static constexpr uint8_t M_CRCCheck = 0x01 << 4;
    static constexpr uint8_t M_Alert = 0x01 << 3;
    static constexpr uint8_t M_Wake = 0x01 << 2;
    static constexpr uint8_t M_Collision = 0x01 << 1;
    static constexpr uint8_t M_BCLevel = 0x01 << 0;
};

/// FUSB302 register POWER values
struct Power {
    static constexpr uint8_t PwrMask = 0x0f << 0;
    static constexpr uint8_t PwrAll = 0x0f << 0;
    static constexpr uint8_t PwrIntOsc = 0x01 << 3;
    static constexpr uint8_t PwrReceiver = 0x01 << 2;
    static constexpr uint8_t PwrMeasure = 0x01 << 1;
    static constexpr uint8_t PwrBandgap = 0x01 << 0;
};

/// FUSB302 register RESET values
struct Reset {
    static constexpr uint8_t PDReset = 0x01 << 1;
    static constexpr uint8_t SWReset = 0x01 << 0;
};

/// FUSB302 register MASKA values
struct MaskA {
    static constexpr uint8_t M_All = 0xff;
    static constexpr uint8_t M_None = 0x00;
    static constexpr uint8_t M_OCPTemp = 0x01 << 7;
    static constexpr uint8_t M_TogDone = 0x01 << 6;
    static constexpr uint8_t M_SoftFail = 0x01 << 5;
    static constexpr uint8_t M_RetryFail = 0x01 << 4;
    static constexpr uint8_t M_HardSent = 0x01 << 3;
    static constexpr uint8_t M_TxSent = 0x01 << 2;
    static constexpr uint8_t M_SoftReset = 0x01 << 1;
    static constexpr uint8_t M_HardReset = 0x01 << 0;
};

/// FUSB302 register MASKB values
struct MaskB {
    static constexpr uint8_t M_All = 0x01;
    static constexpr uint8_t M_None = 0x00;
    static constexpr uint8_t M_GRCSent = 0x01 << 0;
};

/// FUSB302 register STATUS1A values
struct Status1A {
    static constexpr uint8_t TogssMask = 0x07 << 3;
    static constexpr uint8_t TogssToggleRunning = 0x00 << 3;
    static constexpr uint8_t TogssSrcOnCC1 = 0x01 << 3;
    static constexpr uint8_t TogssSrcOnCC2 = 0x02 << 3;
    static constexpr uint8_t TogssSnkOnCC1 = 0x05 << 3;
    static constexpr uint8_t TogssSnkOnCC2 = 0x06 << 3;
    static constexpr uint8_t TogssAutoAccessory = 0x07 << 3;
    static constexpr uint8_t RxSOP2Debug = 0x01 << 2;
    static constexpr uint8_t RxSOP1Debug = 0x01 << 1;
    static constexpr uint8_t RxSOP = 0x01 << 0;
};

/// FUSB302 register INTERRUPTA values
struct InterruptA {
    static constexpr uint8_t I_OCPTemp = 0x01 << 7;
    static constexpr uint8_t I_TogDone = 0x01 << 6;
    static constexpr uint8_t I_SoftFail = 0x01 << 5;
    static constexpr uint8_t I_RetryFail = 0x01 << 4;
    static constexpr uint8_t I_HardSent = 0x01 << 3;
    static constexpr uint8_t I_TxSent = 0x01 << 2;
    static constexpr uint8_t I_SoftReset = 0x01 << 1;
    static constexpr uint8_t I_HardReset = 0x01 << 0;
};

/// FUSB302 register INTERRUPTB values
struct InterruptB {
    static constexpr uint8_t I_GRCSent = 0x01 << 0;
};

/// FUSB302 register STATUS0 values
struct Status0 {
    static constexpr uint8_t VbusOk = 0x01 << 7;
    static constexpr uint8_t Activity = 0x01 << 6;
    static constexpr uint8_t Comp = 0x01 << 5;
    static constexpr uint8_t CRCCheck = 0x01 << 4;
    static constexpr uint8_t AlertCheck = 0x01 << 3;
    static constexpr uint8_t Wake = 0x01 << 2;
    static constexpr uint8_t BCLevelMask = 0x03 << 0;
};

/// FUSB302 register STATUS1 values
struct Status1 {
    static constexpr uint8_t RxSOP2 = 0x01 << 7;
    static constexpr uint8_t RxSOP1 = 0x01 << 6;
    static constexpr uint8_t RxEmpty = 0x01 << 5;
    static constexpr uint8_t RxFull = 0x01 << 4;
    static constexpr uint8_t TxEmpty = 0x01 << 3;
    static constexpr uint8_t TxFull = 0x01 << 2;
    static constexpr uint8_t OverTemp = 0x01 << 1;
    static constexpr uint8_t OCP = 0x01 << 0;
};

/// FUSB302 register INTERRUPT values
struct Interrupt {
    static constexpr uint8_t None = 0;
    static constexpr uint8_t I_VbusOk = 0x01 << 7;
    static constexpr uint8_t I_Activity = 0x01 << 6;
    static constexpr uint8_t I_CompChange = 0x01 << 5;
    static constexpr uint8_t I_CRCCheck = 0x01 << 4;
    static constexpr uint8_t I_Alert = 0x01 << 3;
    static constexpr uint8_t I_Wake = 0x01 << 2;
    static constexpr uint8_t I_Collision = 0x01 << 1;
    static constexpr uint8_t I_BCLevel = 0x01 << 0;
};

/// Tokens used in FUSB302B FIFO
struct Token {
    static constexpr uint8_t TxOn = 0xa1;
    static constexpr uint8_t SOP1 = 0x12;
    static constexpr uint8_t SOP2 = 0x13;
    static constexpr uint8_t SOP3 = 0x1b;
    static constexpr uint8_t Reset1 = 0x15;
    static constexpr uint8_t Reset2 = 0x16;
    static constexpr uint8_t PackSym = 0x80;
    static constexpr uint8_t JamCRC = 0xff;
    static constexpr uint8_t EOP = 0x14;
    static constexpr uint8_t TxOff = 0xfe;
};

}
