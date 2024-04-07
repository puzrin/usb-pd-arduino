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


PDController::PDController(PDPhy* phy) :
    ccPin(0), eventHandler(nullptr), phy(phy), isMonitorOnly(true), isPhyGoodCrc(false),
    rxMessageHead(rxBuffer), txMessage((PDMessage*)txBuffer),
    txMessageId(0), txRetryCount(0), lastRxMessageId(-1), lastSpecRev(1), lastMessage(nullptr) {}

void PDController::setGoodCrcHandling(bool phyGoodCrc) {
    isPhyGoodCrc = phyGoodCrc;
}

void PDController::startController(EventHandlerFunction handler) {
    eventHandler = handler;
    isMonitorOnly = false;
    phy->startSink(this);
    reset();
}

const PDLogEntry* PDController::popLogEntry() {    
    bool found = logEntries.get(currentLogEntry);
    return found ? &currentLogEntry : nullptr;
}

void PDController::log(PDLogEntryType type, const PDMessage* message) {
    PDLogEntry entry;
    entry.type = type;
    entry.time = micros();
    entry.message = message;
    logEntries.put(entry);
}

void PDController::reset() {
    lastRxMessageId = -1;
    txMessageId = 0;
    txRetryCount = 0;
    lastMessage = nullptr;

    phy->prepareRead(reinterpret_cast<PDMessage*>(rxMessageHead));

    Scheduler.cancelTask(TaskIdNoGoodCrcReceived);

    if (eventHandler != nullptr)
        eventHandler(PDControllerEvent(PDControllerEventType::reset));
}

void PDController::onReset(PDSOPSequence seq) {
    reset();
    log(seq == PDSOPSequence::hardReset ? PDLogEntryType::hardReset : PDLogEntryType::cableReset);
    if (eventHandler != nullptr)
        eventHandler(PDControllerEvent(PDControllerEventType::reset));
}

bool PDController::sendControlMessage(PDMessageType messageType) {
    if (isTransmitting())
        return false;
    txMessage->initControl(messageType, lastSpecRev);
    txRetryCount = paramNRetryCount + 1;
    return sendMessage();
}

bool PDController::sendDataMessage(PDMessageType messageType, int numObjects, const uint32_t* objects) {
    if (isTransmitting())
        return false;
    txMessage->initData(messageType, numObjects, lastSpecRev);
    memcpy(txMessage->objects, objects, 4 * numObjects);
    txRetryCount = paramNRetryCount + 1;
    return sendMessage();
}

bool PDController::sendMessage() {
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

void PDController::onMessageTransmitted(bool successful) {

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

void PDController::onNoGoodCrcReceived() {
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

void PDController::onMessageReceived(PDMessage* message) {
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

void PDController::prepareNextTxMessage() {
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

void PDController::onError() {
    log(PDLogEntryType::error);

    // re-setup same buffer for reception
    phy->prepareRead(reinterpret_cast<PDMessage*>(rxMessageHead));
}

void PDController::onVoltageChanged(int cc) {
    ccPin = cc;
    log(cc == 0 ? PDLogEntryType::sinkSourceDisconnected : PDLogEntryType::sinkSourceConnected);
    if (eventHandler != nullptr)
        eventHandler(PDControllerEvent(cc == 0 ? PDControllerEventType::disconnected : PDControllerEventType::connected));
}
