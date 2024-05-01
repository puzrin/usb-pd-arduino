//
// USB Power Delivery for Arduino
// Copyright (c) 2024 Manuel Bleichenbacher
//
// Licensed under MIT License
// https://opensource.org/licenses/MIT
//
// FUSB302B PHY (for ESP32 only)
//

#if defined(ARDUINO_ARCH_ESP32) && defined(PD_USE_FUSB302_PT)

#include <Arduino.h>
#include <FunctionalInterrupt.h>
#include <functional>
#include <atomic>
#include <Wire.h>
#include <string.h>
#include "FUSB302Constants.h"
#include "PDController.h"
#include "PDPhyFUSB302PT.h"
#include "TaskScheduler.h"

#define LC_INCLUDE "lc-addrlabels.h"
#include "pt/pt.h"

using namespace FUSB302;

static constexpr int TaskIdMeasuringExpired = 611;
static constexpr int TaskIdRetryWaitingDone = 621;


/******************************************************************************/

/*
 * Platform-specific code. Temporary here, because current target is
 * protothreads. Should be moved to separate file, with appropriate
 * API updates.
 * 
 * - Generic FUSB302 logic MUST be platform-independent.
 * - If this approach is accepted, probably it will replace RTOS version,
 *   because no need to support both.
 */

static PDPhyFUSB302* phy = nullptr;

static TwoWire* drv_wire = nullptr;
static uint8_t drv_interrupt_pin = 0;

static volatile bool _drv_i2c_busy = false;

static bool drv_i2c_done() { return !_drv_i2c_busy; }

static void (*_drv_i2c_callback)() = nullptr;
static std::function<void(void)> _drv_interrupt_handler = nullptr;

static void drv_init(void (*i2c_callback)(), std::function<void(void)> interrupt_handler) {
    _drv_i2c_callback = i2c_callback;
    _drv_interrupt_handler = interrupt_handler;

    // configure interrupt pin
    pinMode(drv_interrupt_pin, INPUT_PULLUP);
    attachInterrupt(drv_interrupt_pin, interrupt_handler, FALLING);
}

static void drv_i2c_read(uint8_t i2cAddr, uint8_t firstReg, int len, uint8_t* data) {
    _drv_i2c_busy = true;

    drv_wire->beginTransmission(i2cAddr);
    drv_wire->write(firstReg);
    drv_wire->endTransmission(false);
    drv_wire->requestFrom(i2cAddr, (uint8_t)len);
    drv_wire->readBytes(data, len);

    // Since this implementation is synchronous, emulate callback stuff.
    _drv_i2c_busy = false;
    if (_drv_i2c_callback) _drv_i2c_callback();
}

static void drv_i2c_write(uint8_t i2cAddr, uint8_t firstReg, int len, const uint8_t* data) {
    _drv_i2c_busy = true;

    drv_wire->beginTransmission(i2cAddr);
    drv_wire->write(firstReg);
    drv_wire->write(data, len);
    drv_wire->endTransmission();

    // Since this implementation is synchronous, emulate callback stuff.
    _drv_i2c_busy = false;
    if (_drv_i2c_callback) _drv_i2c_callback();
}

static bool drv_int_pin_active() { return digitalRead(10) == LOW; }

/*
 * Prototype of platform-specific fusb302 connector. This feature is
 * exclusive to fusb302. Don't try to propagate it to other PHYs.
 * 
 * This struct characterizes only data/events flow. The concrete implementation
 * left on programmer's choice. It can be class, template or simple struct
 * as now. Names can be updated too.
 */
typedef struct fusb302_connector {
    void (*init)(void (*i2c_callback)(), std::function<void(void)> interrupt_handler);
    void (*i2c_read)(uint8_t i2cAddr, uint8_t firstReg, int len, uint8_t* data);
    void (*i2c_write)(uint8_t i2cAddr, uint8_t firstReg, int len, const uint8_t* data);
    bool (*i2c_done)();
    bool (*int_pin_active)();
} fusb302_connector_t;

// Temporary local connector implementation, until done properly.
static fusb302_connector_t drv = {
    .init = drv_init,
    .i2c_read = drv_i2c_read,
    .i2c_write = drv_i2c_write,
    .i2c_done = drv_i2c_done,
    .int_pin_active = drv_int_pin_active
};

/******************************************************************************/

/*
 * Protothreaded part of driver. Since we expect only single driver instance,
 * it's safe to do this as a set of singletones. Doing this as class methods
 * is possible, but just increase code complexity/mess without real value.
 */

// Helpers
static uint8_t _tmp_val;
#define WRITE_REG_SYNC(reg, val) \
    _tmp_val = val; \
    drv.i2c_write(i2CAddress, reg, 1, &_tmp_val); \
    while(!drv.i2c_done());

#define READ_REG_SYNC(reg, val_ptr) \
    drv.i2c_read(i2CAddress, reg, 1, val_ptr); \
    while(!drv.i2c_done());

#define WRITE_REG(reg, val) \
    _tmp_val = val; \
    drv.i2c_write(phy->i2CAddress, reg, 1, &_tmp_val); \
    PT_WAIT_UNTIL(pt, drv.i2c_done())

#define WRITE_BLK(at, len, buf) \
    drv.i2c_write(phy->i2CAddress, at, len, buf); \
    PT_WAIT_UNTIL(pt, drv.i2c_done())

#define READ_REG(reg, val_ptr) \
    drv.i2c_read(phy->i2CAddress, reg, 1, val_ptr); \
    PT_WAIT_UNTIL(pt, drv.i2c_done())

#define READ_BLK(at, len, buf) \
    drv.i2c_read(phy->i2CAddress, at, len, buf); \
    PT_WAIT_UNTIL(pt, drv.i2c_done())

static void pt_loop();

// Route state change to scheduler
static void i2c_callback() { pt_loop(); }

// extended `pt` to carry extra data for convenience
struct default_pt {
    lc_t lc;
    bool called;
};


static struct default_pt read_message_pt;
// Internal, for PT_SPAWN() calls only.
PT_THREAD(read_message_thread(default_pt* pt)) {
    PT_BEGIN(pt);

    // Read token and header
    static uint8_t buf[3];
    READ_BLK(Reg::FIFOS, 3, buf);

    PDSOPSequence sopSequence = phy->mapSOPSequence(buf[0]);
    if (sopSequence == PDSOPSequence::invalid) {
        // Flush RX FIFO
        WRITE_REG(Reg::Control1, Control1::RxFlush);
        PT_EXIT(pt);
    }

    phy->rxMessage->sopSequence = sopSequence;
    phy->rxMessage->cc = phy->activeCC;
    phy->rxMessage->header = buf[1] | (buf[2] << 8);

    // read payload and CRC
    uint8_t len = phy->rxMessage->payloadSize() + 2; // objects - header + CRC
    READ_BLK(Reg::FIFOS, len, phy->rxMessage->payload() + 2);

    phy->controller->onMessageReceived(phy->rxMessage);

    PT_END(pt);
}


static struct default_pt transmit_message_pt;
static PDMessage* transmit_message_data;

PT_THREAD(transmit_message_thread(default_pt* pt, PDMessage* message)) {
    PT_BEGIN(pt);

    while (true) {
        PT_WAIT_UNTIL(pt, pt->called);
        pt->called = false;

        // Enable internal oscillator
        WRITE_REG(Reg::Power, Power::PwrAll);
        // Flush TX FIFO
        WRITE_REG(Reg::Control0, Control0::TxFlush);

        static int payloadLen = message->payloadSize();
        static const uint8_t* payload = message->payload();

        static uint8_t buf[40];

        // Create token stream
        buf[1] = Token::SOP1;
        buf[0] = Token::SOP1;
        buf[2] = Token::SOP1;
        buf[3] = Token::SOP2;
        buf[4] = static_cast<uint8_t>(Token::PackSym | payloadLen);
        memcpy(buf + 5, payload, payloadLen);
        static int n = 5 + payloadLen;
        buf[n++] = Token::JamCRC;
        buf[n++] = Token::EOP;
        buf[n++] = Token::TxOff;
        buf[n++] = Token::TxOn;
        // The sequence TxOff/TxOn might seem strange, but TxOn is immediately executed during
        // I2C communication while TxOff is executed when the token stream is processed.

        WRITE_BLK(Reg::FIFOS, n, buf);
    }

    PT_END(pt);
}


static struct default_pt interrupt_pt;

PT_THREAD(interrupt_thread(default_pt* pt)) {
    PT_BEGIN(pt);

    while (true) {
        PT_WAIT_UNTIL(pt, pt->called || drv.int_pin_active());
        pt->called = false;

        static uint8_t interrupt;
        READ_REG(Reg::Interrupt, &interrupt);
        static uint8_t interruptA;
        READ_REG(Reg::InterruptA, &interruptA);

        // hard reset
        if ((interruptA & InterruptA::I_HardReset) != 0) {
            Scheduler.cancelTask(TaskIdRetryWaitingDone);
            Scheduler.cancelTask(TaskIdMeasuringExpired);
            phy->transitionToRetryWaiting();
            phy->controller->onReset(PDSOPSequence::hardReset);
            continue;
        }

        // toggling done
        if ((interruptA & InterruptA::I_TogDone) != 0) {
            static uint8_t togss;        
            READ_REG(Reg::Status1A, &togss);
            togss &= Status1A::TogssMask;

            if (togss == Status1A::TogssSnkOnCC1) {
                phy->transitionToMeasuring(1);
            } else if (togss == Status1A::TogssSnkOnCC2) {
                phy->transitionToMeasuring(2);
            } else {
                phy->transitionToRetryWaiting();
            }
        }

        // VBUS OK
        if ((interrupt & Interrupt::I_VbusOk) != 0) {
            static uint8_t Status0;
            READ_REG(Reg::Status0, &Status0);
            static bool vbusok = (Status0 & Status0::VbusOk) != 0;
            static bool isAttached = phy->state != FUSB302State::RetryWaiting
                    && phy->state != FUSB302State::Monitoring
                    && phy->state != FUSB302State::Measuring;
            if (!vbusok && isAttached) {
                phy->transitionToRetryWaiting();
                continue;
            } else if (vbusok && !isAttached) {
                phy->transitionToAttached();
                continue;
            }
        }

        if ((interruptA & InterruptA::I_RetryFail) != 0) {
            phy->controller->onMessageTransmitted(false);
        }

        if ((interruptA & InterruptA::I_TxSent) != 0) {
            // turn off internal oscillator if TX FIFO is empty
            static uint8_t Status1;
            READ_REG(Reg::Status1, &Status1);
            if ((Status1 & Status1::TxEmpty) != 0) {
                WRITE_REG(Reg::Power, Power::PwrAll & ~Power::PwrIntOsc);
            }
            phy->controller->onMessageTransmitted(true);
        }

        // CRC check (message received)
        if ((interrupt & Interrupt::I_CRCCheck) != 0) {

            static uint8_t Status0;
            READ_REG(Reg::Status0, &Status0);
            if ((Status0 & Status0::CRCCheck) == 0) {
                WRITE_REG(Reg::Control1, Control1::RxFlush);
            } else {
                PT_SPAWN(pt, &read_message_pt, read_message_thread(&read_message_pt));
            }
        }

    }

    PT_END(pt);
}


#define init_default_pt(pt) \
    PT_INIT(pt); \
    (pt)->called = false;

// init control structures for all threads
void pt_loop_init() {
    init_default_pt(&interrupt_pt);
    init_default_pt(&transmit_message_pt);
}

/*
 * Interrupt-safe pt scheduler with built-in flattening. Should be driven
 * by all events, causing driver stage change.
 * 
 * Since driver has very few methods - the most simple solution is
 * to pre-create static threads stuff call it directly.
 * 
 * More advanced approach would be to create protothread/context for each
 * driver call, but it's crazy overkill for this case.
 */
std::atomic_flag is_running = ATOMIC_FLAG_INIT;
std::atomic_flag deferred_call = ATOMIC_FLAG_INIT;

static void pt_loop() {
    bool should_run = false;

    do {
        if (is_running.test_and_set()) {
            deferred_call.test_and_set();
            return;
        }

        // Iterate threads
        interrupt_thread(&interrupt_pt);
        transmit_message_thread(&transmit_message_pt, transmit_message_data);

        is_running.clear();
        if (deferred_call.test_and_set()) {
            deferred_call.clear();
            should_run = true;
        }
        else should_run = false;
    } while (should_run);
}

/******************************************************************************/

static char deviceId[20];

PDPhyFUSB302::PDPhyFUSB302()
    : wire(&Wire), rxMessage(nullptr), controller(nullptr), state(FUSB302State::NotStarted), 
        i2CAddress(0x22), interruptPin(10), activeCC(0) {}

void PDPhyFUSB302::startSink(PDController* controller) {
    controller->setGoodCrcHandling(true);
    this->controller = controller;

    // Temporary forwarding for class signature compatibility
    drv_wire = wire;
    drv_interrupt_pin = interruptPin;

    phy = this;
    pt_loop_init();
    drv.init(i2c_callback, [this](){ onInterrupt(); });

    init();
    transitionToMonitoring();
}

void PDPhyFUSB302::prepareRead(PDMessage* msg) {
    rxMessage = msg;
}

bool PDPhyFUSB302::transmitMessage(const PDMessage* msg) {
    transmit_message_data = (PDMessage*)msg;
    transmit_message_pt.called = true;
    pt_loop();

    // TODO: Suspicious. Can't return value. Seems signalled in EC via flags.
    return true;
}

// Not sure about this stuff, left sync temporary 
void PDPhyFUSB302::init() {
    state = FUSB302State::NotStarted;
    activeCC = 0;

    // full reset
    WRITE_REG_SYNC(Reg::Reset, Reset::SWReset | Reset::PDReset);

    // TODO:
    // - check if that's safe, because init() is called from several places
    // - fixme to be platform-independent
    delay(10);

    // TODO: seems should be called only once or dropped at all
    getDeviceId(deviceId);

    // power up everyting except oscillator
    WRITE_REG_SYNC(Reg::Power, Power::PwrAll & ~Power::PwrIntOsc);
    // Mask all interrupts
    WRITE_REG_SYNC(Reg::Mask, Mask::M_All);
    // Mask all interrupts
    WRITE_REG_SYNC(Reg::MaskA, MaskA::M_All);
    // Mask all interrupts
    WRITE_REG_SYNC(Reg::MaskB, MaskB::M_All);
}

void PDPhyFUSB302::onInterrupt() {
    // Interrupt
    interrupt_pt.called = true;
    pt_loop();
}

void PDPhyFUSB302::transitionToMonitoring() {
    // enable interrupts, default current
    WRITE_REG_SYNC(Reg::Control0, Control0::HostCur_USBDef);
    // no VBUS measuring
    WRITE_REG_SYNC(Reg::Measure, 0);
    // unmask interrrupt for TOGDONE
    WRITE_REG_SYNC(Reg::MaskA, MaskA::M_All & ~MaskA::M_TogDone);
    // enable sink toggling
    WRITE_REG_SYNC(Reg::Control2, Control2::ModeSinkPolling | Control2::Toggle);

    state = FUSB302State::Monitoring;
}

void PDPhyFUSB302::transitionToMeasuring(int cc) {
    // Turn off toggling
    WRITE_REG_SYNC(Reg::Control2, 0);
    // Enable interrupts for VBUS OK
    WRITE_REG_SYNC(Reg::Mask, Mask::M_All & ~Mask::M_VbusOk);
    // Unmask interrupts for hard reset
    WRITE_REG_SYNC(Reg::MaskA, MaskA::M_All & ~MaskA::M_HardReset);
    // Enable CC monitoring (and pull downs)
    WRITE_REG_SYNC(Reg::Switches0, Switches0::PullDown1 | Switches0::PullDown2 |
            (cc == 1 ? Switches0::MeasCC1 : Switches0::MeasCC2));

    state = FUSB302State::Measuring;
    activeCC = cc;
    Scheduler.scheduleTaskAfter(TaskIdMeasuringExpired, [this](){ transitionToRetryWaiting(); }, 300000);
}

void PDPhyFUSB302::transitionToAttached() {
    state = FUSB302State::Attached;
    Scheduler.cancelTask(TaskIdMeasuringExpired);

    // Enable interrupts for VBUS OK, BC level, alert and CRC check OK
    WRITE_REG_SYNC(Reg::Mask, Mask::M_All & ~(Mask::M_VbusOk | Mask::M_CRCCheck));
    // Unmask interrupts for hard reset, TX sent, retry failed 
    WRITE_REG_SYNC(Reg::MaskA, MaskA::M_All & ~(MaskA::M_HardReset | MaskA::M_TxSent | MaskA::M_RetryFail));
    // Configure: BMC transmit on CC pin and enable auto CRC
    WRITE_REG_SYNC(Reg::Switches1, Switches1::SpecRevRev_2_0 | (activeCC == 1 ? Switches1::TxCC1 : Switches1::TxCC2) | Switches1::AutoCRC);
    // Configure auto retry for packets withtout GoodCRC acknowledgement
    WRITE_REG_SYNC(Reg::Control3, Control3::NRetries_3 | Control3::AutoRetry);

    controller->onVoltageChanged(activeCC);
}

void PDPhyFUSB302::transitionToRetryWaiting() {
    // Reset FUSB302
    init();
    state = FUSB302State::RetryWaiting;
    Scheduler.scheduleTaskAfter(TaskIdRetryWaitingDone, [this](){ transitionToMonitoring(); }, 500000);
    controller->onVoltageChanged(0);
}

static const char* const BaseProductIds[] = { "FUSB302BxxX", "FUSB302B01MPX", "FUSB302B10MPX", "FUSB302B11MPX" };

static const char* Versions = "????????ABCDEFGH";

void PDPhyFUSB302::getDeviceId(char* deviceIdBuffer) {
    // Keep this in sync-style, since used from sync init only.
    uint8_t deviceId;
    READ_REG_SYNC(Reg::DeviceId, &deviceId);
    uint8_t versionId = deviceId >> 4;
    uint8_t productId = (deviceId >> 2) & 0x03;
    uint8_t revisionId = deviceId & 0x03;

    strcpy(deviceIdBuffer, BaseProductIds[productId]);
    char piece[8] = " ._rev.";
    piece[1] = Versions[versionId];
    piece[6] = 'A' + revisionId;
    strcat(deviceIdBuffer, piece);
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
