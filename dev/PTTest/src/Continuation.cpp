//
// USB Power Delivery for Arduino
// Copyright (c) 2024 Manuel Bleichenbacher
//
// Licensed under MIT License
// https://opensource.org/licenses/MIT
//

#include <Arduino.h>
#include "Continuation.h"

void Continuation::start()
{
    _ctLine = 0;
    run();
}

void Continuation::notifyFromApp()
{
    __disable_irq();
    run();
    __enable_irq();
}

void Continuation::notifyFromInterrupt()
{
    run();
}
