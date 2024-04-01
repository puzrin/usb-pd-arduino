//
// USB Power Delivery for Arduino
// Copyright (c) 2023 Manuel Bleichenbacher
//
// Licensed under MIT License
// https://opensource.org/licenses/MIT
//

//
// Simple trigger board selecting 12V at 1A when the power supply is
// connected and if 12V is available.
//
// Please see https://github.com/manuelbl/usb-pd-arduino/wiki for
// instructions how to wire your specific board to a USB C connector.
// For this sketch, "Sink Mode" is relevant.
//

#include "USBPowerDelivery.h"

#if defined(ARDUINO_ARCH_ESP32)

typedef PDPhyFUSB302 PDPhy;

#elif defined(ARDUINO_ARCH_STM32)

#if defined(STM32G0xx) || defined(STM32G4xx)
typedef PDPhySTM32UCPD PDPhy;
#endif

#endif

static PDPhy pdPhy;
static PDController<PDPhy> powerController(&pdPhy);
static PDSink<PDController<PDPhy>> sink(&powerController);


void setup() {
  sink.start();
  // request 12V @ 1A once power supply is connected
  sink.requestPower(12000, 1000);

  // Uncomment if using X-NUCLEO-SNK1MK1 shield
  // NucleoSNK1MK1.init();
}

void loop() {
}
