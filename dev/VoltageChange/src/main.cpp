//
// USB Power Delivery for Arduino
// Copyright (c) 2023 Manuel Bleichenbacher
//
// Licensed under MIT License
// https://opensource.org/licenses/MIT
//

//
// --- Test program switching between the available fixed voltages
//

#include <Arduino.h>
#include <Wire.h>
#include "USBPowerDelivery.h"


#if defined(ARDUINO_ARCH_ESP32)

typedef PDPhyFUSB302 Phy;
#define USB_PD_PHY USB_PD_PHY_FUSB302

#elif defined(ARDUINO_ARCH_STM32)

typedef PDPhySTM32UCPD Phy;
#define USB_PD_PHY USB_PD_PHY_UCPD1

#endif

#include "USBPowerDeliveryCode.h"


static void handleEvent(PDSinkEventType eventType);
static void switchVoltage();
static bool hasExpired(uint32_t time);

static bool isUSBPDSource = false;
static uint32_t nextVoltageChangeTime = 0;
static int voltageIndex = 0;

static Phy pdPhy;
static PDController powerController(&pdPhy);
static PDProtocolAnalyzer protocolAnalyzer(&powerController);
static PDSink sink(&powerController);

void setup() {
    Serial.begin(115200);
    while (!Serial)
        delay(10);
    Serial.println("USB PD for Arduino - Volatage Change Test");

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
    protocolAnalyzer.poll();

    if (isUSBPDSource && hasExpired(nextVoltageChangeTime))
        switchVoltage();
}

void switchVoltage() {
    // select next fixed voltage
    do {
        voltageIndex += 1;
        if (voltageIndex >= sink.numSourceCapabilities)
            voltageIndex = 0;
    } while (sink.sourceCapabilities[voltageIndex].supplyType != PDSupplyType::fixed);

    sink.requestPower(sink.sourceCapabilities[voltageIndex].maxVoltage);
    nextVoltageChangeTime += 3000;
}

void handleEvent(PDSinkEventType eventType) {
    switch (eventType) {
    case PDSinkEventType::sourceCapabilitiesChanged:
        if (sink.isConnected()) {
            Serial.println("New source capabilities (USB PD supply)");
            isUSBPDSource = true;
            voltageIndex = 0;
            nextVoltageChangeTime = millis() + 3000;
        } else {
            isUSBPDSource = false;
            Serial.println("New source capabilities (no USB PD supply connected)");
        }
        break;

    case PDSinkEventType::voltageChanged:
        Serial.printf("Voltage changed: %5dmV  %5dmA (max)", sink.activeVoltage, sink.activeCurrent);
        Serial.println();
        break;

    case PDSinkEventType::powerRejected:
        Serial.println("Power request rejected");
        break;
    }
}

bool hasExpired(uint32_t time) {
    return (int32_t)(time - millis()) <= 0;
}
