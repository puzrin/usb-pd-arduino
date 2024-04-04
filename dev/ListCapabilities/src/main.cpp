//
// USB Power Delivery for Arduino
// Copyright (c) 2023 Manuel Bleichenbacher
//
// Licensed under MIT License
// https://opensource.org/licenses/MIT
//

//
// --- List the capabilities of the connected power supply
//

#include <Arduino.h>
#include <Wire.h>
#include "USBPowerDelivery.h"


#if defined(ARDUINO_ARCH_ESP32)

typedef PDPhyFUSB302 PDPhy;
#define USB_PD_PHY USB_PD_PHY_FUSB302

#elif defined(ARDUINO_ARCH_STM32)

typedef PDPhySTM32UCPD PDPhy;
#define USB_PD_PHY USB_PD_PHY_UCPD1

#endif

#include "USBPowerDeliveryCode.h"


static void handleEvent(PDSinkEventType eventType);
static void listCapabilities();
static const char* getSupplyTypeName(PDSupplyType type);

static PDPhy pdPhy;
static PDController<PDPhy> powerController(&pdPhy);
static PDSink<PDController<PDPhy>> sink(&powerController);

void setup() {
    Serial.begin(115200);
    while (!Serial)
        delay(10);
    Serial.println("USB BP for Arduino - List Capabilities");

    // configure PHY (if needed)
    #if defined(ARDUINO_ARCH_ESP32)
        Wire.begin(SDA, SCL, 1000000);
        pdPhy.setTwoWire(&Wire);
        pdPhy.setInterruptPin(10);
    #elif defined(ARDUINO_ARCH_STM32)
        pdPhy.setInstance(1);
    #endif

    sink.start(handleEvent);

    #if defined(SNK1M1_SHIELD)
        NucleoSNK1MK1.init();
    #endif
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
