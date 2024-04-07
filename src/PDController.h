//
// USB Power Delivery for Arduino
// Copyright (c) 2023 Manuel Bleichenbacher
//
// Licensed under MIT License
// https://opensource.org/licenses/MIT
//

#pragma once

#include <functional>
#include "PDPhy.h"
#include "RingBuffer.h"


/// Power Delivery log entry type
enum class PDLogEntryType {
    /// Power sink or source has been connected ('ccPin' indicates CC line for communication).
    sinkSourceConnected,
    /// Power sink or source has been disconnected.
    sinkSourceDisconnected,
    /// The USB PD communication has been reset.
    hardReset,
    /// The USB PD communication between DFP and cable has been reset.
    cableReset,
    /// A USB PD message has been received (message is in field 'message').
    messageReceived,
    /// The transmission of a USB PD message has been started.
    transmissionStarted,
    /// The transmission of a USB PD message has completed.
    transmissionCompleted,
    /// The transmission of a USB PD message has failed.
    transmissionFailed,
    /// An error has occurred (invalid message received).
    error
};

/// Power Delivery log entry
struct PDLogEntry {
    /// Log entry type
    PDLogEntryType type;
    /// Time the event occured (in µs, same as 'micros()')
    unsigned long time;
    /// PD message if 'type == PDLogEntryType::messageReceived'
    const PDMessage* message;
};

/// Event types reported to event handler callback
enum class PDControllerEventType {
    /// The communication has been reset
    reset,
    /// Two USB PD devices have been connected
    connected,
    /// The two USB PD devices have been disconnected
    disconnected,
    /// A message has been received
    messageReceived,
    /// The transmission of the previously submitted message has failed
    transmissionFailed
};

/// Event reported to event handler callback
struct PDControllerEvent {
    /// Event type
    PDControllerEventType type;
    /// USB PD message (if `type == messageRecieved`)
    const PDMessage* message;

    /// Create a new event with the specified properties 
    PDControllerEvent(PDControllerEventType t, const PDMessage* m = nullptr) : type(t), message(m) {}
};

/**
 * USB Power Delivery (USB-PD) controller.
 *
 * Using this controller, power delivery can be monitored or controlled.
 * This class supports the basic features for participating as power sink
 * or monitoring USB-PD communication (passively).
 *
 * A single global instance of this class exists. It is called 'PowerController'.
 *
 * USB-PD communication is asynchronous. Thus this class provides two means to
 * inform client code about events.
 *
 * - Client code can register an event handler function, which will be called on events,
 * e.g. if a message has been received or the power supplied has be disconnected.
 * The callback handler will be called from an interrupt handler. Thus it must
 * not block execution, e.g. by writing to serial output.
 *
 * - This class also writes low-level events to a log (with a circular buffer).
 * Client code can read the log entries and then process them. These log
 * entries are rather low-level, e.g. they include the GoodCRC messages, which
 * are used to confirm each non-GoodCRC message.
 *
 * As USB PD communication is timing sensitive, most code will run as part of
 * an interrupt handler.
 */
class PDController {
public:
    /// Event handler function to be called when an event occurs (called from interrupt)
    typedef std::function<void(const PDControllerEvent& event)> EventHandlerFunction;

    /**
     * @brief Construct a new PDController object
     * 
     * @param phy Physical layer instance
     */
    PDController(PDPhy* phy);

    /**
     * @brief Set the GoodCRC handling.
     * 
     * The controller usually takes care of sending GoodCRC messages for received messages,
     * and retrying to send messages when no GoodCRC is received. If the PHY already handles
     * it, the controller can be configured to not handle it.
     * 
     * @param phyGoodCrc 'true' if PHY sends GoodCRC messages automatically and retransmits if no GoodCRC is received, 'false' otherwise
     */
    void setGoodCrcHandling(bool phyGoodCrc);

    /**
     * @brief Starts the controller as a source
     * 
     * @param handler handler to be called when an event occurs
     */
    void startController(EventHandlerFunction handler);

    /**
     * Takes oldest log entry from queue (if any is available)
     *
     * @return pointer to log entry, or 'nullptr' if no event is available
     */
    const PDLogEntry* popLogEntry();

    /// CC pin used for communication
    volatile int ccPin;

    /**
     * @brief Indicates if controller is currently transmitting a message.
     * 
     * The transmission includes waiting for the GoodCRC message and possibly
     * retransmitting the last message.
     * 
     * @return 'true' if this controller is currently transmitting, 'false' otherwise
     */
    bool isTransmitting() { return txRetryCount > 0; }

    /// Starts sending a control message
    bool sendControlMessage(PDMessageType messageType);

    /// Starts sending a data message
    bool sendDataMessage(PDMessageType messageType, int numObjects, const uint32_t* objects);

    // --- handlers called from PDPhy IRQ handlers

    /// Called when the voltage has changed
    /// @param cc indicates the active CC line (1 or 2), or 0 if no USB PD communication is active
    void onVoltageChanged(int cc);

    /// Called when an error has occurred
    void onError();

    /// Called when a message has been transmitted
    void onMessageTransmitted(bool successful);

    /// Called when a message has been received
    void onMessageReceived(PDMessage* message);

    /// Called when a reset has been detected (hard reset, soft reset etc.) 
    void onReset(PDSOPSequence seq);

private:
    static constexpr int LogSize = 32;
    static constexpr int RxBufferLength = 512;
    static constexpr int TxBufferLength = 128;
    static constexpr int MaxMessageSize = 7 * 4 + 4;

    static constexpr int paramNRetryCount = 2;
    static constexpr int paramCRCReceiveTimer = 1200; // in µs

    static constexpr int TaskIdNoGoodCrcReceived = 801;

    EventHandlerFunction eventHandler;
    PDPhy* phy;
    bool isMonitorOnly;
    bool isPhyGoodCrc;

    uint8_t rxBuffer[RxBufferLength] __attribute__((aligned(4)));
    uint8_t* volatile rxMessageHead;
    uint8_t txBuffer[TxBufferLength] __attribute__((aligned(4)));
    PDMessage* volatile txMessage;

    RingBuffer<PDLogEntry, LogSize> logEntries;
    PDLogEntry currentLogEntry;

    int txMessageId;
    int txRetryCount;
    int lastRxMessageId;
    int lastSpecRev;
    const PDMessage* lastMessage;

    void reset();

    void log(PDLogEntryType type, const PDMessage* message = nullptr);

    /// Starts sending the message prepared in 'txMessage'
    bool sendMessage();


    void prepareNextTxMessage();

    void onNoGoodCrcReceived();
};
