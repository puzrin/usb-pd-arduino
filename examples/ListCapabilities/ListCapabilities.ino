//
// USB Power Delivery for Arduino
// Copyright (c) 2023 Manuel Bleichenbacher
//
// Licensed under MIT License
// https://opensource.org/licenses/MIT
//

//
// Lists the capabilities of the connected power supply.
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
  Serial.begin(115200);

  // configure PHY (if needed)
  #if defined(ARDUINO_ARCH_ESP32)
    Wire.begin(SDA, SCL, 1000000);
    pdPhy.setTwoWire(&Wire);
    pdPhy.setInterruptPin(10);
  #elif defined(ARDUINO_ARCH_STM32)
    pdPhy.setInstance(1);
  #endif

  sink.start(handleEvent);

  // Uncomment if using X-NUCLEO-SNK1MK1 shield
  // NucleoSNK1MK1.init();
}

void loop() {
  sink.poll();
}

void handleEvent(PDSinkEventType eventType) {
  if (eventType == PDSinkEventType::sourceCapabilitiesChanged && sink.isConnected())
    listCapabilities();
}

void listCapabilities() {
  Serial.println("USB PD capabilities:");
  Serial.println("__Type_________Vmin____Vmax____Imax");

  for (int i = 0; i < sink.numSourceCapabilities; i += 1) {
    auto cap = sink.sourceCapabilities[i];
    Serial.printf("  %-9s  %6d  %6d  %6d", getSupplyTypeName(cap.supplyType), cap.minVoltage, cap.maxVoltage, cap.maxCurrent);
    Serial.println();
  }

  Serial.println("(voltage in mV, current in mA)");
  Serial.println();
}

static const char* const SupplyTypeNames[] = {
  [(int)PDSupplyType::fixed] = "Fixed",
  [(int)PDSupplyType::battery] = "Battery",
  [(int)PDSupplyType::variable] = "Variable",
  [(int)PDSupplyType::pps] = "PPS",
};

const char* getSupplyTypeName(PDSupplyType type) {
  unsigned int arraySize = sizeof(SupplyTypeNames) / sizeof(SupplyTypeNames[0]);
  if ((unsigned int)type < arraySize)
    return SupplyTypeNames[(unsigned int)type];
  else
    return "<unknown>";
}
