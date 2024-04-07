//
// USB Power Delivery for Arduino
// Copyright (c) 2024 Manuel Bleichenbacher
//
// Licensed under MIT License
// https://opensource.org/licenses/MIT
//

#pragma once

#include <Arduino.h>
#include <HardwareTimer.h>
#include "Continuation.h"

/**
 * @brief Timer.
 * 
 * When the scheduled timer is reached, it will resume the specified continuation.
 */
struct Timer {
    Timer(Continuation* continuation)
        : continuation(continuation), expired(false) { }
    bool isRunning() { return !expired; }
    bool hasExpired() { return expired; }
    void scheduleAt(uint32_t time);
    void scheduleIn(uint32_t delay);
    void cancel();

private:
    Continuation* continuation;
    bool expired;

    friend class Scheduler;
};

struct Scheduler {
    static void setHardwareTimer(HardwareTimer* timer);
    static void start();

    static void scheduleTimer(Timer* timer, uint32_t time);
    static void cancelTimer(Timer* timer);

private:
    struct TimerSchedule {
        uint32_t time;
        Timer* timer;
    };

    static constexpr int MAX_TIMERS = 12;
    static HardwareTimer* hardwareTimer;
    static int numSchedules;
    static TimerSchedule schedules[MAX_TIMERS];
    static bool isInInterruptHandler;

    static void onInterrupt();
    static void insertTimer(Timer* timer, uint32_t time);
    static void removeTimer(Timer* timer);
    static void removeAt(int index);
    static void reschedule();
};
