//
// USB Power Delivery for Arduino
// Copyright (c) 2024 Manuel Bleichenbacher
//
// Licensed under MIT License
// https://opensource.org/licenses/MIT
//
// FUSB302B PHY (for ESP32 only)
//

#if defined(ARDUINO_ARCH_ESP32)

#include <Arduino.h>
#include <FunctionalInterrupt.h>
#include <Wire.h>
#include <string.h>
#include "FUSB302Constants.h"
#include "PDController.h"
#include "PDPhyFUSB302.h"
#include "TaskScheduler.h"

using namespace FUSB302;

static char deviceId[20];

PDPhyFUSB302::PDPhyFUSB302()
    : wire(&Wire), rxMessage(nullptr), controller(nullptr), state(FUSB302State::NotStarted), 
        i2CAddress(0x22), interruptPin(10), activeCC(0) {}

void PDPhyFUSB302::startSink(PDController* controller) {
    controller->setGoodCrcHandling(true);
    this->controller = controller;

    eventQueue = xQueueCreate(10, sizeof(QueueItem));
    init();

    xTaskCreate(sinkTaskStatic, "USB PD", 4000, this, 10, nullptr);
}

void PDPhyFUSB302::prepareRead(PDMessage* msg) {
    postEvent(FUSB302Event::PrepareRead, msg);
}

bool PDPhyFUSB302::transmitMessage(const PDMessage* msg) {
    postEvent(FUSB302Event::TransmitMessage, (PDMessage*)msg);
    return true;
}

void PDPhyFUSB302::init() {
    state = FUSB302State::NotStarted;
    activeCC = 0;

    xQueueReset(eventQueue);

    // configure interrupt pin
    pinMode(interruptPin, INPUT_PULLUP);
    attachInterrupt(interruptPin, [this](){ onInterrupt(); }, FALLING);

    // full reset
    writeRegister(Reg::Reset, Reset::SWReset | Reset::PDReset);
    delay(10);

    getDeviceId(deviceId);

    // power up everyting except oscillator
    writeRegister(Reg::Power, Power::PwrAll & ~Power::PwrIntOsc);
    // Mask all interrupts
    writeRegister(Reg::Mask, Mask::M_All);
    // Mask all interrupts
    writeRegister(Reg::MaskA, MaskA::M_All);
    // Mask all interrupts
    writeRegister(Reg::MaskB, MaskB::M_All);
}

void PDPhyFUSB302::sinkTaskStatic(void* param) {
    PDPhyFUSB302* instance = (PDPhyFUSB302*)param;
    instance->sinkTask();
}

void PDPhyFUSB302::sinkTask() {
    transitionToMonitoring();

    while (true) {
        // Wait for next event
        QueueItem event;
        if (xQueueReceive(eventQueue, &event, portMAX_DELAY) == pdTRUE)
            processEvent(event.event, event.message);
    }
}

void PDPhyFUSB302::processEvent(FUSB302Event event, PDMessage* message) {

    switch (event) {
        case FUSB302Event::Interrupt:
            handleInterrupts();
            break;
        case FUSB302Event::PrepareRead:
            rxMessage = message;
            break;
        case FUSB302Event::TransmitMessage:
            sendMessage(message);
            break;
        case FUSB302Event::TransitionToMonitoring:
            transitionToMonitoring();
            break;
        case FUSB302Event::TransitionToRetryWaiting:
            transitionToRetryWaiting();
            break;
    }
}

void PDPhyFUSB302::handleInterrupts() {
    do {
        handleInterrupt();
    } while (digitalRead(interruptPin) == LOW);
}

// handle single interrupt event
void PDPhyFUSB302::handleInterrupt() {
    uint8_t interrupt = readRegister(Reg::Interrupt);
    uint8_t interruptA = readRegister(Reg::InterruptA);

    // hard reset
    if ((interruptA & InterruptA::I_HardReset) != 0) {
        Scheduler.cancelTask(TaskIdRetryWaitingDone);
        Scheduler.cancelTask(TaskIdMeasuringExpired);
        transitionToRetryWaiting();
        controller->onReset(PDSOPSequence::hardReset);
        return;
    }

    // toggling done
    if ((interruptA & InterruptA::I_TogDone) != 0) {
        uint8_t togss = readRegister(Reg::Status1A) & Status1A::TogssMask;
        if (togss == Status1A::TogssSnkOnCC1) {
            transitionToMeasuring(1);
        } else if (togss == Status1A::TogssSnkOnCC2) {
            transitionToMeasuring(2);
        } else {
            transitionToRetryWaiting();
        }
    }

    // VBUS OK
    if ((interrupt & Interrupt::I_VbusOk) != 0) {
        bool vbusok = (readRegister(Reg::Status0) & Status0::VbusOk) != 0;
        bool isAttached = state != FUSB302State::RetryWaiting
                && state != FUSB302State::Monitoring
                && state != FUSB302State::Measuring;
        if (!vbusok && isAttached) {
            transitionToRetryWaiting();
            return;
        } else if (vbusok && !isAttached) {
            transitionToAttached();
            return;
        }
    }

    if ((interruptA & InterruptA::I_RetryFail) != 0) {
        controller->onMessageTransmitted(false);
    }

    if ((interruptA & InterruptA::I_TxSent) != 0) {
        // turn off internal oscillator if TX FIFO is empty
        uint8_t Status1 = readRegister(Reg::Status1);
        if ((Status1 & Status1::TxEmpty) != 0)
            writeRegister(Reg::Power, Power::PwrAll & ~Power::PwrIntOsc);
        controller->onMessageTransmitted(true);
    }

    // CRC check (message received)
    if ((interrupt & Interrupt::I_CRCCheck) != 0) {

        uint8_t Status0 = readRegister(Reg::Status0);
        if ((Status0 & Status0::CRCCheck) == 0) {
            writeRegister(Reg::Control1, Control1::RxFlush);
        } else {
            readMessage();
        }
    }
}

void PDPhyFUSB302::postEvent(FUSB302Event event, PDMessage* message) {
    QueueItem item;
    item.event = event;
    item.message = message;
    xQueueSendToBack(eventQueue, &item, portMAX_DELAY);
}

void PDPhyFUSB302::postEventFromISR(FUSB302Event event, PDMessage* message) {
    QueueItem item;
    item.event = event;
    item.message = message;
    BaseType_t higherPriorityTaskWoken = pdFALSE;
    xQueueSendToBackFromISR(eventQueue, &item, &higherPriorityTaskWoken);
    if (higherPriorityTaskWoken)
        portYIELD_FROM_ISR();
}

void PDPhyFUSB302::onInterrupt() {
    postEventFromISR(FUSB302Event::Interrupt);
}

void PDPhyFUSB302::transitionToMonitoring() {
    // enable interrupts, default current
    writeRegister(Reg::Control0, Control0::HostCur_USBDef);
    // no VBUS measuring
    writeRegister(Reg::Measure, 0);
    // unmask interrrupt for TOGDONE
    writeRegister(Reg::MaskA, MaskA::M_All & ~MaskA::M_TogDone);
    // enable sink toggling
    writeRegister(Reg::Control2, Control2::ModeSinkPolling | Control2::Toggle);

    state = FUSB302State::Monitoring;
}

void PDPhyFUSB302::transitionToMeasuring(int cc) {
    // Turn off toggling
    writeRegister(Reg::Control2, 0);
    // Enable interrupts for VBUS OK
    writeRegister(Reg::Mask, Mask::M_All & ~Mask::M_VbusOk);
    // Unmask interrupts for hard reset
    writeRegister(Reg::MaskA, MaskA::M_All & ~MaskA::M_HardReset);
    // Enable CC monitoring (and pull downs)
    writeRegister(Reg::Switches0, Switches0::PullDown1 | Switches0::PullDown2 |
            (cc == 1 ? Switches0::MeasCC1 : Switches0::MeasCC2));

    state = FUSB302State::Measuring;
    activeCC = cc;
    Scheduler.scheduleTaskAfter(TaskIdMeasuringExpired, [this](){ postEvent(FUSB302Event::TransitionToRetryWaiting); }, 300000);
}

void PDPhyFUSB302::transitionToAttached() {
    state = FUSB302State::Attached;
    Scheduler.cancelTask(TaskIdMeasuringExpired);

    // Enable interrupts for VBUS OK, BC level, alert and CRC check OK
    writeRegister(Reg::Mask, Mask::M_All & ~(Mask::M_VbusOk | Mask::M_CRCCheck));
    // Unmask interrupts for hard reset, TX sent, retry failed 
    writeRegister(Reg::MaskA, MaskA::M_All & ~(MaskA::M_HardReset | MaskA::M_TxSent | MaskA::M_RetryFail));
    // Configure: BMC transmit on CC pin and enable auto CRC
    writeRegister(Reg::Switches1, Switches1::SpecRevRev_2_0 | (activeCC == 1 ? Switches1::TxCC1 : Switches1::TxCC2) | Switches1::AutoCRC);
    // Configure auto retry for packets withtout GoodCRC acknowledgement
    writeRegister(Reg::Control3, Control3::NRetries_3 | Control3::AutoRetry);

    controller->onVoltageChanged(activeCC);
}

void PDPhyFUSB302::transitionToRetryWaiting() {
    // Reset FUSB302
    init();
    state = FUSB302State::RetryWaiting;
    Scheduler.scheduleTaskAfter(TaskIdRetryWaitingDone, [this](){ postEvent(FUSB302Event::TransitionToMonitoring); }, 500000);
    controller->onVoltageChanged(0);
}

int PDPhyFUSB302::readMessage() {
    // Read token and header
    uint8_t buf[3];
    readRegister(Reg::FIFOS, 3, buf);

    PDSOPSequence sopSequence = mapSOPSequence(buf[0]);
    if (sopSequence == PDSOPSequence::invalid) {
        // Flush RX FIFO
        writeRegister(Reg::Control1, Control1::RxFlush);
        return 0;
    }

    rxMessage->sopSequence = sopSequence;
    rxMessage->cc = activeCC;
    rxMessage->header = buf[1] | (buf[2] << 8);

    // read payload and CRC
    uint8_t len = rxMessage->payloadSize() + 2; // objects - header + CRC
    readRegister(Reg::FIFOS, len, rxMessage->payload() + 2);

    controller->onMessageReceived(rxMessage);

    return len;
}

void PDPhyFUSB302::sendMessage(const PDMessage* message) {
    // Enable internal oscillator
    writeRegister(Reg::Power, Power::PwrAll);
    // Flush TX FIFO
    writeRegister(Reg::Control0, Control0::TxFlush);

    int payloadLen = message->payloadSize();
    const uint8_t* payload = message->payload();

    uint8_t buf[40];

    // Create token stream
    buf[1] = Token::SOP1;
    buf[0] = Token::SOP1;
    buf[2] = Token::SOP1;
    buf[3] = Token::SOP2;
    buf[4] = static_cast<uint8_t>(Token::PackSym | payloadLen);
    memcpy(buf + 5, payload, payloadLen);
    int n = 5 + payloadLen;
    buf[n++] = Token::JamCRC;
    buf[n++] = Token::EOP;
    buf[n++] = Token::TxOff;
    buf[n++] = Token::TxOn;
    // The sequence TxOff/TxOn might seem strange, but TxOn is immediately executed during
    // I2C communication while TxOff is executed when the token stream is processed.

    writeRegister(Reg::FIFOS, n, buf);
}


static const char* const BaseProductIds[] = { "FUSB302BxxX", "FUSB302B01MPX", "FUSB302B10MPX", "FUSB302B11MPX" };

static const char* Versions = "????????ABCDEFGH";

void PDPhyFUSB302::getDeviceId(char* deviceIdBuffer) {
    uint8_t deviceId = readRegister(Reg::DeviceId);
    uint8_t versionId = deviceId >> 4;
    uint8_t productId = (deviceId >> 2) & 0x03;
    uint8_t revisionId = deviceId & 0x03;

    strcpy(deviceIdBuffer, BaseProductIds[productId]);
    char piece[8] = " ._rev.";
    piece[1] = Versions[versionId];
    piece[6] = 'A' + revisionId;
    strcat(deviceIdBuffer, piece);
}

uint8_t PDPhyFUSB302::readRegister(uint8_t r) {
    wire->beginTransmission(i2CAddress);
    wire->write(r);
    wire->endTransmission(false);
    wire->requestFrom(i2CAddress, (uint8_t)1);
    return (uint8_t) wire->read();
}

void PDPhyFUSB302::readRegister(uint8_t firstReg, int n, uint8_t* data) {
    wire->beginTransmission(i2CAddress);
    wire->write(firstReg);
    wire->endTransmission(false);
    wire->requestFrom(i2CAddress, (uint8_t)n);
    wire->readBytes(data, n);
}

void PDPhyFUSB302::writeRegister(uint8_t r, uint8_t value) {
    wire->beginTransmission(i2CAddress);
    wire->write(r);
    wire->write(value);
    wire->endTransmission();
}

void PDPhyFUSB302::writeRegister(uint8_t firstReg, int n, const uint8_t* data) {
    wire->beginTransmission(i2CAddress);
    wire->write(firstReg);
    wire->write(data, n);
    wire->endTransmission();
}

static const PDSOPSequence SopSequenceMap[] = {
    PDSOPSequence::sop2Debug,
    PDSOPSequence::sop1Debug,
    PDSOPSequence::sop2,
    PDSOPSequence::sop1,
    PDSOPSequence::sop
};

PDSOPSequence PDPhyFUSB302::mapSOPSequence(uint32_t sop) {
    sop >>= 5;
    if (sop < 3 || sop > 7)
        return PDSOPSequence::invalid;

    return SopSequenceMap[sop - 3];
}

#endif
