//
// USB Power Delivery for Arduino
// Copyright (c) 2023 Manuel Bleichenbacher
//
// Licensed under MIT License
// https://opensource.org/licenses/MIT
//

#pragma once

#if defined(STM32G0xx) || defined(STM32G4xx)

#include "PDController.h"

extern "C" void UCPD1_IRQHandler();
extern "C" void UCPD1_2_IRQHandler();

/**
 * @brief Physical layer for USB PD communication.
 *
 */
struct PDPhySTM32UCPD {

    /**
     * @brief Creates a new PDPhySTM32UCPD instance
     */
    PDPhySTM32UCPD();
    
    /**
     * @brief Start the USB PD PHY as a sink.
     * 
     * In the sink role, the PHY will interact in the USB PD communication.
     * For PHYs with controllable pull-up/down resistors, it will
     * activate the pull-down resistors to present itself as a power sink.
     * 
     * @param controller the PD controller to be notified about events
     */
    void startSink(PDController<PDPhySTM32UCPD>* controller);

    /**
     * @brief Sets the message and buffer to be used for the next incoming message.
     * 
     * @param msg the message
     */
    void prepareRead(PDMessage* msg);

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
    bool transmitMessage(const PDMessage* msg);

private:
    PDController<PDPhySTM32UCPD>* controller;
    PDMessage* rxMessage;
    int ccActive;

    void init(bool isMonitor);
    void enableCommunication(int cc);
    void disableCommunication();
    void enableRead();
    void handleInterrupt();

    static PDSOPSequence mapSOPSequence(uint32_t orderedSet);

    friend PDController<PDPhySTM32UCPD>;
    friend void UCPD1_IRQHandler();
    friend void UCPD1_2_IRQHandler();
};

#endif
