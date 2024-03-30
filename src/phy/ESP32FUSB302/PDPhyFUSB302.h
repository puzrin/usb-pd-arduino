//
// USB Power Delivery for Arduino
// Copyright (c) 2024 Manuel Bleichenbacher
//
// Licensed under MIT License
// https://opensource.org/licenses/MIT
//
// FUSB302B PHY (for ESP32 only)
//

#pragma once

#if defined(ARDUINO_ARCH_ESP32)

#include "PDPhy.h"

/// FUSB302 state
enum class FUSB302State {
    /// not started
    NotStarted,
    /// Disconnected, monitoring for attachment
    Monitoring,
    /// Attached, waiting for VBUS to be present
    Measuring,
    /// VBUS is present
    Attached,
    /// Wait period after a failure
    RetryWaiting
};

enum class FUSB302Event {
    Interrupt,
    PrepareRead,
    TransmitMessage,
    TransitionToMonitoring,
    TransitionToRetryWaiting
};


/**
 * USB PD PHY implementation for FUSB302B.
 * 
 * USB PD sink is supported only. Monitoring is not implemented.
 * 
 * This implementation uses FreeRTOS tasks and queues.
 * It is mainly intended for the ESP32.
 * 
 * The FUSB302B is configured to send GoodCRC responses automatically on
 * receipt of a valid message, and to retry transmission of a message
 * when no GoodCRC is received. As a result, GoodCRC messages are not
 * visible in the log.
 * 
 * For I2C communication, the global `Wire` object is used. It should
 * be configured with a frequency of at least 1 MHz.
 */
struct PDPhyFUSB302 : PDPhy {
    PDPhyFUSB302();

    void getDeviceId(char* deviceIdBuffer);  

private:
    static const uint8_t I2CAddress = 0x22;
    static const uint8_t InterruptPin = 10;

    FUSB302State state;
    QueueHandle_t eventQueue;
    PDMessage* rxMessage;
    int activeCC;

    void init();
    void startSink();
    static void sinkTaskStatic(void* param);
    void sinkTask();

    void postEvent(FUSB302Event event, PDMessage* message = nullptr);
    ARDUINO_ISR_ATTR void postEventFromISR(FUSB302Event event, PDMessage* message = nullptr);
    ARDUINO_ISR_ATTR static void onInterrupt();
    void processEvent(FUSB302Event event, PDMessage* message);
    void handleInterrupts();
    void handleInterrupt();

    void transitionToMonitoring();
    void transitionToMeasuring(int cc);
    void transitionToAttached();
    void transitionToEstablished();
    void transitionToRetryWaiting();

    static void sendMsgToTransitionToMonitoring();
    static void sendMsgToTransitionToRetryWaiting();

    void checkForMessage();
    int readMessage();
    void sendMessage(const PDMessage* msg);

    uint8_t readRegister(uint8_t r);
    void readRegister(uint8_t firstReg, int n, uint8_t* data);
    void writeRegister(uint8_t r, uint8_t value);
    void writeRegister(uint8_t firstReg, int n, const uint8_t* data);

    static PDSOPSequence mapSOPSequence(uint32_t sop);

    friend class PDPhy;
};

#endif
