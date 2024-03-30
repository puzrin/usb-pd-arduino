//
// USB Power Delivery for Arduino
// Copyright (c) 2023 Manuel Bleichenbacher
//
// Licensed under MIT License
// https://opensource.org/licenses/MIT
//

#include <Arduino.h>
#include "PDController.h"
#include "PDPhy.h"
#include "TaskScheduler.h"


template<bool AUTO_GOOD_CRC, bool AUTO_TX_RETRY>
PDController<AUTO_GOOD_CRC, AUTO_TX_RETRY>::PDController()
: ccPin(0), isMonitorOnly(true), eventHandler(nullptr), rxMessageHead(rxBuffer), txMessage((PDMessage*)txBuffer),
    logHead(0), logTail(0), txMessageId(0), txRetryCount(0), lastRxMessageId(-1), lastSpecRev(1), lastMessage(nullptr)
{
    PDPhy::prepareRead(reinterpret_cast<PDMessage*>(rxMessageHead));
}

template<bool AUTO_GOOD_CRC, bool AUTO_TX_RETRY>
void PDController<AUTO_GOOD_CRC, AUTO_TX_RETRY>::startController(EventHandlerFunction handler) {
    eventHandler = handler;
    isMonitorOnly = false;
    PDPhy::initSink();
    reset();
}

template<bool AUTO_GOOD_CRC, bool AUTO_TX_RETRY>
void PDController<AUTO_GOOD_CRC, AUTO_TX_RETRY>::startMonitor() {
    eventHandler = nullptr;
    isMonitorOnly = true;
    PDPhy::initMonitor();
    reset();
}

template<bool AUTO_GOOD_CRC, bool AUTO_TX_RETRY>
const PDLogEntry* PDController<AUTO_GOOD_CRC, AUTO_TX_RETRY>::popLogEntry() {
    if (logHead == logTail)
        return nullptr;

    uint32_t index = logTail % LogSize;
    logTail += 1;
    return &logEntries[index];
}

template<bool AUTO_GOOD_CRC, bool AUTO_TX_RETRY>
void PDController<AUTO_GOOD_CRC, AUTO_TX_RETRY>::log(PDLogEntryType type, const PDMessage* message) {
    uint32_t index = logHead % LogSize;
    logEntries[index].type = type;
    logEntries[index].time = micros();
    logEntries[index].message = message;
    logHead += 1;
}

template<bool AUTO_GOOD_CRC, bool AUTO_TX_RETRY>
void PDController<AUTO_GOOD_CRC, AUTO_TX_RETRY>::reset() {
    lastRxMessageId = -1;
    txMessageId = 0;
    txRetryCount = 0;
    lastMessage = nullptr;
    Scheduler.cancelTask(noGoodCrcReceivedCallback);

    if (eventHandler != nullptr)
        eventHandler(PDControllerEvent(PDControllerEventType::reset));
}

template<bool AUTO_GOOD_CRC, bool AUTO_TX_RETRY>
void PDController<AUTO_GOOD_CRC, AUTO_TX_RETRY>::onReset(PDSOPSequence seq) {
    reset();
    log(seq == PDSOPSequence::hardReset ? PDLogEntryType::hardReset : PDLogEntryType::cableReset);
    if (eventHandler != nullptr)
        eventHandler(PDControllerEvent(PDControllerEventType::reset));
}

template<bool AUTO_GOOD_CRC, bool AUTO_TX_RETRY>
bool PDController<AUTO_GOOD_CRC, AUTO_TX_RETRY>::sendControlMessage(PDMessageType messageType) {
    if (isTransmitting())
        return false;
    txMessage->initControl(messageType, lastSpecRev);
    txRetryCount = paramNRetryCount + 1;
    return sendMessage();
}

template<bool AUTO_GOOD_CRC, bool AUTO_TX_RETRY>
bool PDController<AUTO_GOOD_CRC, AUTO_TX_RETRY>::sendDataMessage(PDMessageType messageType, int numObjects, const uint32_t* objects) {
    if (isTransmitting())
        return false;
    txMessage->initData(messageType, numObjects, lastSpecRev);
    memcpy(txMessage->objects, objects, 4 * numObjects);
    txRetryCount = paramNRetryCount + 1;
    return sendMessage();
}

template<bool AUTO_GOOD_CRC, bool AUTO_TX_RETRY>
bool PDController<AUTO_GOOD_CRC, AUTO_TX_RETRY>::sendMessage() {
    // add message ID
    if (AUTO_GOOD_CRC) {
        txMessage->setMessageId(txMessageId);
    } else {
        bool isGoodCrc = txMessage->type() == PDMessageType::controlGoodCrc;
        txMessage->setMessageId(isGoodCrc ? lastRxMessageId : txMessageId);
    }

    txMessage->cc = ccPin;
    if (!PDPhy::transmitMessage(txMessage))
        return false;

    log(PDLogEntryType::transmissionStarted, txMessage);
    return true;
}

template<bool AUTO_GOOD_CRC, bool AUTO_TX_RETRY>
void PDController<AUTO_GOOD_CRC, AUTO_TX_RETRY>::onMessageTransmitted(bool successful) {

    if (!successful) {
        log(PDLogEntryType::transmissionFailed);
        prepareNextTxMessage();
        return;
    }

    log(PDLogEntryType::transmissionCompleted);

    if (AUTO_TX_RETRY) {
        prepareNextTxMessage();        

    } else {
        // For regular messages, we will start timer for the GoodCRC message (so the TX buffer is not yet changed).
        // For GoodCRC messages, the transmission is completed.

        if (txMessage->type() != PDMessageType::controlGoodCrc) {
            // scheduled timer to check for GoodCRC
            Scheduler.scheduleTaskAfter(noGoodCrcReceivedCallback, paramCRCReceiveTimer);

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

template<bool AUTO_GOOD_CRC, bool AUTO_TX_RETRY>
void PDController<AUTO_GOOD_CRC, AUTO_TX_RETRY>::noGoodCrcReceivedCallback() {
    if (!AUTO_TX_RETRY) {
        PowerController.onNoGoodCrcReceived();
    }
}

template<bool AUTO_GOOD_CRC, bool AUTO_TX_RETRY>
void PDController<AUTO_GOOD_CRC, AUTO_TX_RETRY>::onNoGoodCrcReceived() {
    if (!AUTO_TX_RETRY) {
        Scheduler.cancelTask(noGoodCrcReceivedCallback);

        txRetryCount -= 1;
        if (txRetryCount > 0 && txMessage->type() != PDMessageType::controlGoodCrc) {
            // retry
            sendMessage();

        } else {
            // transmission has failed - no retry
            prepareNextTxMessage();
        }
    }
}

template<bool AUTO_GOOD_CRC, bool AUTO_TX_RETRY>
void PDController<AUTO_GOOD_CRC, AUTO_TX_RETRY>::onMessageReceived(PDMessage* message) {
    int messageId = message->messageId();
    PDMessageType type = message->type();
    PDSOPSequence sopSeq = message->sopSequence;
    bool reportTransmissionFailed = false;

    log(PDLogEntryType::messageReceived, message);

    // prepare for next read
    rxMessageHead = message->end();
    if (rxMessageHead + MaxMessageSize > rxBuffer + sizeof(rxBuffer))
        rxMessageHead = rxBuffer;

    PDPhy::prepareRead(reinterpret_cast<PDMessage*>(rxMessageHead));

    if (AUTO_GOOD_CRC) {
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

            Scheduler.cancelTask(noGoodCrcReceivedCallback);
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

template<bool AUTO_GOOD_CRC, bool AUTO_TX_RETRY>
void PDController<AUTO_GOOD_CRC, AUTO_TX_RETRY>::prepareNextTxMessage() {
    txRetryCount = 0;

    if (AUTO_GOOD_CRC) {
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

template<bool AUTO_GOOD_CRC, bool AUTO_TX_RETRY>
void PDController<AUTO_GOOD_CRC, AUTO_TX_RETRY>::onError() {
    log(PDLogEntryType::error);

    // re-setup same buffer for reception
    PDPhy::prepareRead(reinterpret_cast<PDMessage*>(rxMessageHead));
}

template<bool AUTO_GOOD_CRC, bool AUTO_TX_RETRY>
void PDController<AUTO_GOOD_CRC, AUTO_TX_RETRY>::onVoltageChanged(int cc) {
    ccPin = cc;
    log(cc == 0 ? PDLogEntryType::sinkSourceDisconnected : PDLogEntryType::sinkSourceConnected);
    if (eventHandler != nullptr)
        eventHandler(PDControllerEvent(cc == 0 ? PDControllerEventType::disconnected : PDControllerEventType::connected));
}

#if defined(ARDUINO_ARCH_ESP32)

template class PDController<true, true>;
PDController<true, true> PowerController{};

#else

template class PDController<false, false>;
PDController<false, false> PowerController{};

#endif
