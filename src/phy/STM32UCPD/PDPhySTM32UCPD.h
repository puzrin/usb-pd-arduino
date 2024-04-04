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
     * @brief Set the UCPD peripheral instance.
     * 
     * For the G0 family, valid vaalues are 1 and 2 (for UCPD1 and UCPD2).
     * For the G4 family, the only valid value is 1.
     * The default value is 1.
     * 
     * @param instance 
     */
    void setInstance(int instance) { this->instance = instance - 1; }
    
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
    struct Peripheral {
        UCPD_TypeDef* ucpd;
        GPIO_TypeDef* gpioCC1;
        GPIO_TypeDef* gpioCC2;
        uint16_t pinCC1;
        uint16_t pinCC2;
        uint8_t arduinoPinCC1;
        uint8_t arduinoPinCC2;
        DMA_TypeDef* dmaRx;
        DMA_TypeDef* dmaTx;
        uint8_t dmaChannelRx;
        uint8_t dmaRequestRx;
        uint8_t dmaChannelTx;
        uint8_t dmaRequestTx;
        int16_t ucpdIrq;
        uint16_t dbatt;
    };

#if defined(STM32G0xx)
    static constexpr int NumInstances = 2;
#else
    static constexpr int NumInstances = 1;
#endif

    static const Peripheral peripherals[NumInstances];
    static PDPhySTM32UCPD* instances[NumInstances];

    const Peripheral* peripheral;
    PDController<PDPhySTM32UCPD>* controller;
    PDMessage* rxMessage;
    int ccActive;
    int instance;

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
