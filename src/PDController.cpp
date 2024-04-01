//
// USB Power Delivery for Arduino
// Copyright (c) 2023 Manuel Bleichenbacher
//
// Licensed under MIT License
// https://opensource.org/licenses/MIT
//

#include <Arduino.h>
#include "PDController.h"
#include "TaskScheduler.h"


template <class Phy>
PDController<Phy>::PDController(Phy* phy) :
    ccPin(0), eventHandler(nullptr), phy(phy), isMonitorOnly(true), isPhyGoodCrc(false),
    rxMessageHead(rxBuffer), txMessage((PDMessage*)txBuffer),
    txMessageId(0), txRetryCount(0), lastRxMessageId(-1), lastSpecRev(1), lastMessage(nullptr) {}

template <class Phy>
void PDController<Phy>::setGoodCrcHandling(bool phyGoodCrc) {
    isPhyGoodCrc = phyGoodCrc;
}

template <class Phy>
void PDController<Phy>::startController(EventHandlerFunction handler) {
    eventHandler = handler;
    isMonitorOnly = false;
    phy->startSink(this);
    reset();
}

template <class Phy>
const PDLogEntry* PDController<Phy>::popLogEntry() {    
    bool found = logEntries.get(currentLogEntry);
    return found ? &currentLogEntry : nullptr;
}

template <class Phy>
void PDController<Phy>::log(PDLogEntryType type, const PDMessage* message) {
    PDLogEntry entry;
    entry.type = type;
    entry.time = micros();
    entry.message = message;
    logEntries.put(entry);
}

template <class Phy>
void PDController<Phy>::reset() {
    lastRxMessageId = -1;
    txMessageId = 0;
    txRetryCount = 0;
    lastMessage = nullptr;

    phy->prepareRead(reinterpret_cast<PDMessage*>(rxMessageHead));

    Scheduler.cancelTask(TaskIdNoGoodCrcReceived);

    if (eventHandler != nullptr)
        eventHandler(PDControllerEvent(PDControllerEventType::reset));
}

template <class Phy>
void PDController<Phy>::onReset(PDSOPSequence seq) {
    reset();
    log(seq == PDSOPSequence::hardReset ? PDLogEntryType::hardReset : PDLogEntryType::cableReset);
    if (eventHandler != nullptr)
        eventHandler(PDControllerEvent(PDControllerEventType::reset));
}

template <class Phy>
bool PDController<Phy>::sendControlMessage(PDMessageType messageType) {
    if (isTransmitting())
        return false;
    txMessage->initControl(messageType, lastSpecRev);
    txRetryCount = paramNRetryCount + 1;
    return sendMessage();
}

template <class Phy>
bool PDController<Phy>::sendDataMessage(PDMessageType messageType, int numObjects, const uint32_t* objects) {
    if (isTransmitting())
        return false;
    txMessage->initData(messageType, numObjects, lastSpecRev);
    memcpy(txMessage->objects, objects, 4 * numObjects);
    txRetryCount = paramNRetryCount + 1;
    return sendMessage();
}

template <class Phy>
bool PDController<Phy>::sendMessage() {
    // add message ID
    if (isPhyGoodCrc) {
        txMessage->setMessageId(txMessageId);
    } else {
        bool isGoodCrc = txMessage->type() == PDMessageType::controlGoodCrc;
        txMessage->setMessageId(isGoodCrc ? lastRxMessageId : txMessageId);
    }

    txMessage->cc = ccPin;
    if (!phy->transmitMessage(txMessage))
        return false;

    log(PDLogEntryType::transmissionStarted, txMessage);
    return true;
}

template <class Phy>
void PDController<Phy>::onMessageTransmitted(bool successful) {

    if (!successful) {
        log(PDLogEntryType::transmissionFailed);
        prepareNextTxMessage();
        return;
    }

    log(PDLogEntryType::transmissionCompleted);

    if (isPhyGoodCrc) {
        prepareNextTxMessage();        

    } else {
        // For regular messages, we will start timer for the GoodCRC message (so the TX buffer is not yet changed).
        // For GoodCRC messages, the transmission is completed.

        if (txMessage->type() != PDMessageType::controlGoodCrc) {
            // scheduled timer to check for GoodCRC
            Scheduler.scheduleTaskAfter(TaskIdNoGoodCrcReceived, [this](){ onNoGoodCrcReceived(); }, paramCRCReceiveTimer);

        } else {
            prepareNextTxMessage();

            // If a GoodCRC message for a message was successfully sent, the event handler is notified
            if (lastMessage != nullptr) {
                if (eventHandler != nullptr)
                    eventHandler(PDControllerEvent(PDControllerEventType::messageReceived, lastMessage));
                lastMessage = nullptr;
            }
        }
    }
}

template <class Phy>
void PDController<Phy>::onNoGoodCrcReceived() {
    Scheduler.cancelTask(TaskIdNoGoodCrcReceived);

    txRetryCount -= 1;
    if (txRetryCount > 0 && txMessage->type() != PDMessageType::controlGoodCrc) {
        // retry
        sendMessage();

    } else {
        // transmission has failed - no retry
        prepareNextTxMessage();
    }
}

template <class Phy>
void PDController<Phy>::onMessageReceived(PDMessage* message) {
    int messageId = message->messageId();
    PDMessageType type = message->type();
    PDSOPSequence sopSeq = message->sopSequence;
    bool reportTransmissionFailed = false;

    log(PDLogEntryType::messageReceived, message);

    // prepare for next read
    rxMessageHead = message->end();
    if (rxMessageHead + MaxMessageSize > rxBuffer + sizeof(rxBuffer))
        rxMessageHead = rxBuffer;

    phy->prepareRead(reinterpret_cast<PDMessage*>(rxMessageHead));

    if (isPhyGoodCrc) {
        // notify that message has been received
        if (eventHandler != nullptr)
            eventHandler(PDControllerEvent(PDControllerEventType::messageReceived, message));

    } else {
        if (isTransmitting()) {
            // a GoodCRC message is expected
            if (type != PDMessageType::controlGoodCrc || sopSeq != PDSOPSequence::sop || messageId != txMessageId) {
                log(PDLogEntryType::transmissionFailed);
                reportTransmissionFailed = true;
            }

            Scheduler.cancelTask(TaskIdNoGoodCrcReceived);
            prepareNextTxMessage();
            lastMessage = nullptr;
        }

        if (type != PDMessageType::controlGoodCrc && sopSeq == PDSOPSequence::sop && !isMonitorOnly) {

            // if the message has the same message ID like the previous one,
            // 'lastMessage' is not set to prevent notifying the event handler twice
            if (messageId != lastRxMessageId) {
                lastMessage = message;
                lastRxMessageId = messageId;
                lastSpecRev = message->specRev();
            }

            // send GoodCRC to confirm received message
            sendControlMessage(PDMessageType::controlGoodCrc);
        }

        // deliberately call event handler after GoodCRC transmission has started
        if (reportTransmissionFailed && eventHandler != nullptr)
            eventHandler(PDControllerEvent(PDControllerEventType::transmissionFailed));
    }
}

template <class Phy>
void PDController<Phy>::prepareNextTxMessage() {
    txRetryCount = 0;

    if (isPhyGoodCrc) {
        txMessageId += 1;
        if (txMessageId >= 8)
            txMessageId = 0;

    } else {
        if (txMessage->type() != PDMessageType:: controlGoodCrc) {
            txMessageId += 1;
            if (txMessageId >= 8)
                txMessageId = 0;
        }
    }

    // prepare new TX message buffer
    uint8_t* head = txMessage->end();
    if (head + MaxMessageSize >= txBuffer + sizeof(txBuffer))
        head = txBuffer;
    txMessage = (PDMessage*)head;
}

template <class Phy>
void PDController<Phy>::onError() {
    log(PDLogEntryType::error);

    // re-setup same buffer for reception
    phy->prepareRead(reinterpret_cast<PDMessage*>(rxMessageHead));
}

template <class Phy>
void PDController<Phy>::onVoltageChanged(int cc) {
    ccPin = cc;
    log(cc == 0 ? PDLogEntryType::sinkSourceDisconnected : PDLogEntryType::sinkSourceConnected);
    if (eventHandler != nullptr)
        eventHandler(PDControllerEvent(cc == 0 ? PDControllerEventType::disconnected : PDControllerEventType::connected));
}


// template instantiation

#if defined(ARDUINO_ARCH_ESP32)

#include "phy/ESP32FUSB302/PDPhyFUSB302.h"
template class PDController<PDPhyFUSB302>;

#elif defined(ARDUINO_ARCH_STM32)

#if defined(STM32G0xx) || defined(STM32G4xx)
#include "phy/STM32UCPD/PDPhySTM32UCPD.h"
template class PDController<PDPhySTM32UCPD>;
#endif

#endif
