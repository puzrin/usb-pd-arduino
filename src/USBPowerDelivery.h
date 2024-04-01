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

#if defined(ARDUINO_ARCH_ESP32)

#include "phy/ESP32FUSB302/PDPhyFUSB302.h"

#elif defined(ARDUINO_ARCH_STM32)

#include "phy/STM32UCPD/PDPhySTM32UCPD.h"
#include "phy/NucleoSNK1MK1/NucleoSNK1MK1.h"

#endif
