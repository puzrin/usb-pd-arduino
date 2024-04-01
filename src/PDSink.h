//
// USB Power Delivery for Arduino
// Copyright (c) 2023 Manuel Bleichenbacher
//
// Licensed under MIT License
// https://opensource.org/licenses/MIT
//
// USB PD sink
//

#pragma once

#include <functional>
#include "PDSourceCapability.h"
#include "PDController.h"

enum class PDSinkEventType {
    sourceCapabilitiesChanged,
    voltageChanged,
    powerRejected
};

/**
 * Power Delivery Sink
 *
 * Communicates with a power source (power supply) to control the voltage
 * and current being supplied.
 * 
 * @tparam Controller PD controller class
 */
template <class Controller>
class PDSink {
public:
    /**
     * Type of function that will be called when an event has occurred.
     */
    typedef std::function<void(PDSinkEventType eventType)> EventCallbackFunction;

    /**
     * @brief Construct a new instance
     * 
     * For correct voltage reporting, the power sink needs to know if it is
     * bus powered (MCU is powered from the same USB connection that it
     * interacts with for the USB-PD communication), or self-powered (MCU has
     * power supply separate from the one it interacts with).
     * 
     * The specified PD controller instance talks to actual hardware, be it an internal
     * USB PD peripheral, an external USB PD controller chip or a software implementation
     * using comparators and bit banging.
     * 
     * @param controller PD controller
     */
    PDSink(Controller* controller);

    /**
     * @brief Set if this sink is bus powered.
     * 
     * For correct voltage reporting, the power sink needs to know if it is
     * bus powered (MCU is powered from the same USB connection that it
     * interacts with for the USB-PD communication), or self-powered (MCU has
     * power supply separate from the one it interacts with).
     * 
     * If not set, the sink defaults to bus powered.
     * 
     * @param busPowered indicates if the sink is bus powered
     */
    void setBusPowered(bool busPowered);

    /**
     * Starts USB Power Delivery as a power sink.
     *
     * The callback function will be called when 'poll()' is called and
     * an event has occurred since the last call of 'poll()'.
     *
     * @param callback event callback function
     */
    void start(EventCallbackFunction callback = nullptr);

    /**
     * Polls for new events.
     * 
     * Call this function frequently from 'loop()'.
     */
    void poll();

    /// Indicates if the sink is connected to a USB PD power supply
    bool isConnected() {
        return numSourceCapabilities > 0;
    }

    /**
     * Requests a new voltage and optionally a maximum current.
     * 
     * If the method is successful, itwill also set 'requestedVoltage' and
     * 'requestedCurrent' to reflect what has actually been requested from
     * the power supply. These values might differ because of rounding and
     * unspecified current.
     * 
     * If the method is not successful, it will still remember the voltage
     * and current values. If the power supply becomes available or changes,
     * the sink will try to request these values.
     * 
     * @param voltage voltage, in mV
     * @param maxCurrent maximum current, in mA (or 0 to selected maximum current)
     * @return 'true' if the desired voltage and current are available
     */
    bool requestPower(int voltage, int maxCurrent = 0);

    /// Number of valid elements in `sourceCapabilities` array
    int numSourceCapabilities = 0;

    /// Array of supply capabilities
    PDSourceCapability sourceCapabilities[7];

    /// Active voltage in mV
    int activeVoltage;

    /// Active maximum current in mA
    int activeCurrent;

    /// Requested voltage in mV
    int requestedVoltage;

    /// Requested current in mA
    int requestedCurrent;

private:
    enum ErrorCode {
        ok,
        notConnected,
        noMatchingCapability,
        controllerBusy
    };

    static constexpr int TaskIdRequestPPS = 1001;

    Controller* controller;
    EventCallbackFunction eventCallback;

    bool isBusPowered;
    int ppsIndex;
    int desiredVoltage;
    int desiredCurrent;

    bool capabilitiesChanged;

    // event flags
    bool flagSourceCapChanged;
    bool flagVoltageChanged;
    bool flagPowerRejected;

    ErrorCode requestPowerCore(int voltage, int maxCurrent);

    void handleEvent(const PDControllerEvent& event);
    void reset(bool connected);

    void onMessageReceived(const PDMessage* message);
    void onSourceCapabilities(const PDMessage* message);
    void onPsReady();
    void onRerequestPPS();
};
