//
// USB Power Delivery for Arduino
// Copyright (c) 2024 Manuel Bleichenbacher
//
// Licensed under MIT License
// https://opensource.org/licenses/MIT
//

#include "I2C.h"

I2C* I2C::instance = nullptr;

I2C::I2C(TwoWire* twoWire, Continuation* continuation)
    : completed(false), i2c(twoWire->getHandle()), continuation(continuation) {
    instance = this;
}

void I2C::startReadRegister(uint8_t address, uint8_t reg, uint8_t* result)
{
    completed = false;
    HAL_I2C_Mem_Read_IT(i2c, address << 1, reg, I2C_MEMADD_SIZE_8BIT, result, 1);
}

void I2C::onInterrupt()
{
    completed = true;
    continuation->notifyFromInterrupt();
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    I2C::instance->onInterrupt();
}
