//
// USB Power Delivery for Arduino
// Copyright (c) 2024 Manuel Bleichenbacher
//
// Licensed under MIT License
// https://opensource.org/licenses/MIT
//

#pragma

#include <Arduino.h>
#include <Wire.h>
#include "Continuation.h"

extern "C" void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c);


struct I2C {
public:
    I2C(TwoWire* twowire, Continuation* continuation);
    void startReadRegister(uint8_t address, uint8_t reg, uint8_t* result);
    bool hasCompleted() { return completed; }
    bool isRunning() { return !completed; }

private:
    static I2C* instance;
    bool completed;
    I2C_HandleTypeDef* i2c;
    Continuation* continuation;

    void onInterrupt();
    friend void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c);
};


/// Wait until I2C transation has completed (and yield)
#define CT_WAIT_I2C(i2c) CT_WAIT_UNTIL((i2c)->hasCompleted())
