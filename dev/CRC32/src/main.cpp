//
// USB Power Delivery for Arduino
// Copyright (c) 2024 Manuel Bleichenbacher
//
// Licensed under MIT License
// https://opensource.org/licenses/MIT
//

//
// --- Test program switching between the available fixed voltages
//

#include <Arduino.h>
#include "CRC32.h"

static constexpr uint32_t expectedCRC = 0xF191DF63;

void setup() {
    Serial.begin(115200);
    while (!Serial)
        delay(10);
    Serial.println("CRC 32 Verification");

    const uint8_t data[] = { 0x3b, 0x25, 0x77, 0xce, 0x4e, 0xb3, 0x4e, 0x28, 0xce, 0x8e, 0x63, 0xdf, 0x91, 0xf1 };
    uint32_t crc = CRC32::compute(data, sizeof(data) - 4);
    Serial.print("CRC32: 0x");
    Serial.println(crc, HEX);
    if (crc == expectedCRC) {
        Serial.println("CRC calculation: OK");
    } else {
        Serial.print("Error: CRC32 invalid. Expected: 0x");
        Serial.println(expectedCRC, HEX);
    }

    if (CRC32::isValid(data, sizeof(data))) {
        Serial.println("CRC32 check: OK");
    } else {
        Serial.println("Error: CRC32 check returned false");
    }
}

void loop() {
}
