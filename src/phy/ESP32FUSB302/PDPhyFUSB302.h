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

#include <Wire.h>
#include "PDController.h"

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
struct PDPhyFUSB302 {

    /**
     * @brief Construct a new instance.
     */
    PDPhyFUSB302();

    /**
     * @brief Sets the I2C address of the FUSB302 device.
     * 
     * The default is 0x22.
     * 
     * @param address I2C address
     */
    void setI2CAddress(uint8_t address) {
        i2CAddress = address;
    }

    /**
     * @brief Set the `TwoWire` instance to use for I2C communication.
     * 
     * Defaults to `Wire`.
     * 
     * The object should be configured with a frequency of at least 1 MHz.
     * 
     * @param wire I2C communication object
     */
    void setTwoWire(TwoWire* wire) {
        this->wire = wire;
    }

    /**
     * @brief Sets the pin used for the interrupt signal.
     * 
     * The default is pin 10.
     * 
     * @param pin interrupt pin
     */
    void setInterruptPin(uint8_t pin) {
        interruptPin = pin;
    }

    /**
     * @brief Get the FUSB302 device ID.
     * 
     * The device ID describes the exact revision of the FUSB302 chip.
     * 
     * @param deviceIdBuffer character buffer, at least 20 bytes long
     */
    void getDeviceId(char* deviceIdBuffer);

    /**
     * @brief Start the USB PD PHY as a sink.
     * 
     * In the sink role, the PHY will interact in the USB PD communication.
     * For PHYs with controllable pull-up/down resistors, it will
     * activate the pull-down resistors to present itself as a power sink.
     * 
     * @param controller the PD controller to be notified about events
     */
    void startSink(PDController<PDPhyFUSB302>* controller);

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
    typedef std::function<void(void)> InterruptHandlerFunction;

    static constexpr int TaskIdMeasuringExpired = 611;
    static constexpr int TaskIdRetryWaitingDone = 621;

    TwoWire* wire;
    QueueHandle_t eventQueue;
    PDMessage* rxMessage;
    PDController<PDPhyFUSB302>* controller;
    FUSB302State state;
    uint8_t i2CAddress;
    uint8_t interruptPin;
    int activeCC;

    void init();
    static void sinkTaskStatic(void* param);
    void sinkTask();

    void postEvent(FUSB302Event event, PDMessage* message = nullptr);
    ARDUINO_ISR_ATTR void postEventFromISR(FUSB302Event event, PDMessage* message = nullptr);
    ARDUINO_ISR_ATTR void onInterrupt();
    void processEvent(FUSB302Event event, PDMessage* message);
    void handleInterrupts();
    void handleInterrupt();

    void transitionToMonitoring();
    void transitionToMeasuring(int cc);
    void transitionToAttached();
    void transitionToEstablished();
    void transitionToRetryWaiting();

    int readMessage();
    void sendMessage(const PDMessage* msg);

    uint8_t readRegister(uint8_t r);
    void readRegister(uint8_t firstReg, int n, uint8_t* data);
    void writeRegister(uint8_t r, uint8_t value);
    void writeRegister(uint8_t firstReg, int n, const uint8_t* data);

    static PDSOPSequence mapSOPSequence(uint32_t sop);

    struct QueueItem {
        FUSB302Event event;
        PDMessage* message;
    };
};

#endif
