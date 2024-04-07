//
// USB Power Delivery for Arduino
// Copyright (c) 2023 Manuel Bleichenbacher
//
// Licensed under MIT License
// https://opensource.org/licenses/MIT
//

//
// Selects 12V if available, and 15V otherwise.

// Please see https://github.com/manuelbl/usb-pd-arduino/wiki for
// instructions how to wire your specific board to a USB C connector.
// For this sketch, "Sink Mode" is relevant.
//

#include "USBPowerDelivery.h"


#if defined(ARDUINO_ARCH_ESP32)

typedef PDPhyFUSB302 Phy;
#define USB_PD_PHY USB_PD_PHY_FUSB302

#elif defined(ARDUINO_ARCH_STM32)

typedef PDPhySTM32UCPD Phy;
#define USB_PD_PHY USB_PD_PHY_UCPD1

#endif

#include "USBPowerDeliveryCode.h"


static Phy pdPhy;
static PDController powerController(&pdPhy);
static PDSink sink(&powerController);


void setup() {
  // configure PHY (if needed)
  #if defined(ARDUINO_ARCH_ESP32)
    Wire.begin(SDA, SCL, 1000000);
    pdPhy.setTwoWire(&Wire);
    pdPhy.setInterruptPin(10);
  #elif defined(ARDUINO_ARCH_STM32)
    pdPhy.setInstance(1);
  #endif

  Serial.begin(115200);
  sink.start(handleEvent);

  // Uncomment if using X-NUCLEO-SNK1MK1 shield
  // NucleoSNK1MK1.init();
}

void loop() {
  sink.poll();
}

void handleEvent(PDSinkEventType eventType) {

  if (eventType == PDSinkEventType::sourceCapabilitiesChanged) {
    // source capabilities have changed
    if (sink.isConnected()) {
      // USB PD supply is connected
      requestVoltage();
  
    } else {
      // no supply or no USB PD capable supply is connected
      sink.requestPower(5000); // reset to 5V
    }

  } else if (eventType == PDSinkEventType::voltageChanged) {
    // voltage has changed
    if (sink.activeVoltage != 0) {
      Serial.printf("Voltage: %d mV @ %d mA (max)", sink.activeVoltage, sink.activeCurrent);
      Serial.println();
    } else {
      Serial.println("Disconnected");
    }

  } else if (eventType == PDSinkEventType::powerRejected) {
    // rare case: power supply rejected requested power
    Serial.println("Power request rejected");
    Serial.printf("Voltage: %d mV @ %d mA (max)", sink.activeVoltage, sink.activeCurrent);
  }
}

void requestVoltage() {
  // check if 12V is supported
  for (int i = 0; i < sink.numSourceCapabilities; i += 1) {
    if (sink.sourceCapabilities[i].minVoltage <= 12000
        && sink.sourceCapabilities[i].maxVoltage >= 12000) {
      sink.requestPower(12000);
      return;
    }
  }

  // check if 15V is supported
  for (int i = 0; i < sink.numSourceCapabilities; i += 1) {
    if (sink.sourceCapabilities[i].minVoltage <= 15000
        && sink.sourceCapabilities[i].maxVoltage >= 15000) {
      sink.requestPower(15000);
      return;
    }
  }

  Serial.println("Neither 12V nor 15V is supported");
}

