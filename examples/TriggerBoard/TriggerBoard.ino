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
#define USB_PD_PHY USB_PD_PHY_FUSB302

#elif defined(ARDUINO_ARCH_STM32)

typedef PDPhySTM32UCPD PDPhy;
#define USB_PD_PHY USB_PD_PHY_UCPD1

#endif

#include "USBPowerDeliveryCode.h"


static PDPhy pdPhy;
static PDController<PDPhy> powerController(&pdPhy);
static PDSink<PDController<PDPhy>> sink(&powerController);


void setup() {
  // configure PHY (if needed)
  #if defined(ARDUINO_ARCH_ESP32)
    Wire.begin(SDA, SCL, 1000000);
    pdPhy.setTwoWire(&Wire);
    pdPhy.setInterruptPin(10);
  #elif defined(ARDUINO_ARCH_STM32)
    pdPhy.setInstance(1);
  #endif

  sink.start();
  // request 12V @ 1A once power supply is connected
  sink.requestPower(12000, 1000);

  // Uncomment if using X-NUCLEO-SNK1MK1 shield
  // NucleoSNK1MK1.init();
}

void loop() {
}
