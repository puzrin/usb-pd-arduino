//
// USB Power Delivery for Arduino
// Copyright (c) 2023 Manuel Bleichenbacher
//
// Licensed under MIT License
// https://opensource.org/licenses/MIT
//
// USB PD sink
//

#include "PDSink.h"
#include "TaskScheduler.h"


template <class Controller>
PDSink<Controller>::PDSink(Controller* controller) :
    controller(controller), eventCallback(nullptr), isBusPowered(true), ppsIndex(-1), desiredVoltage(5000),
    desiredCurrent(0), capabilitiesChanged(false), flagSourceCapChanged(false),
    flagVoltageChanged(false), flagPowerRejected(false) {}

template <class Controller>
void PDSink<Controller>::setBusPowered(bool busPowered) {
    isBusPowered = busPowered;
}

template <class Controller>
void PDSink<Controller>::reset(bool connected) {
    activeVoltage = isBusPowered || connected ? 5000 : 0;
    activeCurrent = isBusPowered || connected ? 900 : 0;
    requestedVoltage = 5000;
    requestedCurrent = 900;
    numSourceCapabilities = 0;
    flagSourceCapChanged = true;
    flagVoltageChanged = true;
    flagPowerRejected = false;
    capabilitiesChanged = false;
    Scheduler.cancelTask(TaskIdRequestPPS);
}

template <class Controller>
void PDSink<Controller>::start(EventCallbackFunction callback) {
    eventCallback = callback;
    reset(false);
    controller->startController([this](const PDControllerEvent& event) { handleEvent(event); });
}

template <class Controller>
void PDSink<Controller>::poll() {
    if (eventCallback == nullptr)
        return;

    while (flagSourceCapChanged || flagVoltageChanged || flagPowerRejected) {
        if (flagSourceCapChanged) {
            flagSourceCapChanged = false;
            eventCallback(PDSinkEventType::sourceCapabilitiesChanged);
        }
        if (flagVoltageChanged) {
            flagVoltageChanged = false;
            eventCallback(PDSinkEventType::voltageChanged);
        }
        if (flagPowerRejected) {
            flagPowerRejected = false;
            eventCallback(PDSinkEventType::powerRejected);
        }
    }
}

template <class Controller>
bool PDSink<Controller>::requestPower(int voltage, int maxCurrent) {
    desiredVoltage = voltage;
    desiredCurrent = maxCurrent;

    while (true) {
        ErrorCode err = requestPowerCore(voltage, maxCurrent);

        if (err == ok)
            return true;

        if (err == controllerBusy) {
            // retry after delay
            delay(1);
            continue;
        }

        return false;
    }

    // loop until message could be transmitted
    // (controller could be busy)
    while (isConnected()) {
        bool successful = requestPowerCore(voltage, maxCurrent);
        if (successful)
            break;
        delay(1);
    }
}

template <class Controller>
typename PDSink<Controller>::ErrorCode PDSink<Controller>::requestPowerCore(int voltage, int maxCurrent) {

    if (!isConnected())
        return notConnected;

    // create 'Request' message
    int capabilityIndex = -1;
    uint32_t requestObject = PDSourceCapability::powerRequestObject(capabilityIndex, voltage, maxCurrent,
                                                                    numSourceCapabilities, sourceCapabilities);

    if (requestObject == 0)
        return noMatchingCapability;

    // send message
    bool successful = controller->sendDataMessage(PDMessageType::dataRequest, 1, &requestObject);
    if (!successful)
        return controllerBusy;

    Scheduler.cancelTask(TaskIdRequestPPS);

    ppsIndex = -1;
    if (sourceCapabilities[capabilityIndex].supplyType == PDSupplyType::pps)
        ppsIndex = capabilityIndex;

    requestedVoltage = voltage;
    requestedCurrent = maxCurrent;

    return ok;
}

template <class Controller>
void PDSink<Controller>::handleEvent(const PDControllerEvent& event) {
    switch (event.type) {

    case PDControllerEventType::reset:
    case PDControllerEventType::connected:
    case PDControllerEventType::disconnected:
        reset(event.type == PDControllerEventType::connected);
        break;

    case PDControllerEventType::messageReceived:
        onMessageReceived(event.message);
        break;

    default:; // ignore
    }
}

template <class Controller>
void PDSink<Controller>::onMessageReceived(const PDMessage* message) {
    switch (message->type()) {

    case PDMessageType::dataSourceCapabilities:
        onSourceCapabilities(message);
        break;

    case PDMessageType::controlReject:
        flagPowerRejected = true;
        break;

    case PDMessageType::controlPsReady:
        onPsReady();
        break;

    default:; // nothing to do
    }
}

template <class Controller>
void PDSink<Controller>::onSourceCapabilities(const PDMessage* message) {
    // parse source capabilities message
    PDSourceCapability::parseMessage(message, numSourceCapabilities, sourceCapabilities);

    capabilitiesChanged = true;

    ErrorCode err = requestPowerCore(desiredVoltage, desiredCurrent);
    if (err != ok)
        requestPowerCore(5000, 0); // fallback: select 5V
}

template <class Controller>
void PDSink<Controller>::onPsReady() {
    if (capabilitiesChanged) {
        flagSourceCapChanged = true;
        capabilitiesChanged = false;
    }
    
    if (activeVoltage != requestedVoltage || activeCurrent != requestedCurrent)
        flagVoltageChanged = true;
    activeVoltage = requestedVoltage;
    activeCurrent = requestedCurrent;

    // PPS voltages need to be requested at least every 10s; otherwise
    // the power supply returns to 5V
    if (ppsIndex >= 0)
        Scheduler.scheduleTaskAfter(TaskIdRequestPPS, [this](){ onRerequestPPS(); }, 8000000);
}

template <class Controller>
void PDSink<Controller>::onRerequestPPS() {
    // request the same voltage again
    uint32_t requestObject = PDSourceCapability::powerRequestObject(ppsIndex, requestedVoltage, requestedCurrent,
                                                                    numSourceCapabilities, sourceCapabilities);
    bool successful = controller->sendDataMessage(PDMessageType::dataRequest, 1, &requestObject);
    if (!successful)
        Scheduler.scheduleTaskAfter(TaskIdRequestPPS, [this](){ onRerequestPPS(); }, 100000);
}


// template instantiation

#if defined(ARDUINO_ARCH_ESP32)

#include "phy/ESP32FUSB302/PDPhyFUSB302.h"
template class PDSink<PDController<PDPhyFUSB302>>;

#elif defined(ARDUINO_ARCH_STM32)

#if defined(STM32G0xx) || defined(STM32G4xx)
#include "phy/STM32UCPD/PDPhySTM32UCPD.h"
template class PDSink<PDController<PDPhySTM32UCPD>>;
#endif

#endif
