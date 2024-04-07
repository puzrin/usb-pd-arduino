//
// USB Power Delivery for Arduino
// Copyright (c) 2024 Manuel Bleichenbacher
//
// Licensed under MIT License
// https://opensource.org/licenses/MIT
//
// Physical layer for USB PD communication
//

#pragma once

#include "PDMessage.h"

class PDController;


/**
 * @brief Physical layer for USB PD communication.
 * 
 */
class PDPhy {
public:
    /**
     * @brief Initializes the USB PD PHY as a sink.
     * 
     * In the sink role, the PHY will interact in the USB PD communication.
     * For PHYs with controllable pull-up/down resistors, it will
     * activate the pull-down resistors to present itself as a power sink.
     * 
     * @param controller the PD controller to be notified about events
     */
    virtual void startSink(PDController* controller) = 0;

    /**
     * @brief Sets the message and buffer to be used for the next incoming message.
     * 
     * @param msg the message
     */
    virtual void prepareRead(PDMessage* msg) = 0;

    /**
     * @brief Transmits a message.
     * 
     * The method is asynchronous. It will start the transmissoin when the CC
     * line is idle and trigger an event to report when the message has been
     * transmitted or the transmission has failed.
     * 
     * @param msg the message
     * @return `true` if transmission was started, `false` if it failed
     *      (TX activity by this or the other device, no active CC line)
     */
    virtual bool transmitMessage(const PDMessage* msg) = 0;
};