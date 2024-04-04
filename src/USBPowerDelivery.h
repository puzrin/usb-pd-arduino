//
// USB Power Delivery for Arduino
// Copyright (c) 2023 Manuel Bleichenbacher
//
// Licensed under MIT License
// https://opensource.org/licenses/MIT
//
// USB Power Delivery
//
// Include this header file for working with the library classes.
//

#pragma once

#include "PDController.h"
#include "PDSink.h"
#include "PDProtocolAnalyzer.h"


#define USB_PD_PHY_FUSB302 100
// Use UCPD1 only (instance 1)
#define USB_PD_PHY_UCPD1 201
// Use UCPD2 only (instance 2)
#define USB_PD_PHY_UCPD2 202
// Use UCPD1 and UCPD2 (instance 1 and 2)
#define USB_PD_PHY_UCPD1_2 203


#if defined(ARDUINO_ARCH_ESP32)

#include "phy/ESP32FUSB302/PDPhyFUSB302.h"

#elif defined(ARDUINO_ARCH_STM32)

#include "phy/STM32UCPD/PDPhySTM32UCPD.h"
#include "phy/NucleoSNK1MK1/NucleoSNK1MK1.h"

#endif
