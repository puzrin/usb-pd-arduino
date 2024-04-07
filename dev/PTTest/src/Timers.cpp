//
// USB Power Delivery for Arduino
// Copyright (c) 2024 Manuel Bleichenbacher
//
// Licensed under MIT License
// https://opensource.org/licenses/MIT
//

#include <string.h>
#include <Arduino.h>
#include "Timers.h"

static constexpr uint32_t TIMER_FREQUENCY = 10000; // 0.1 ms
static constexpr uint32_t MS_SCALE = TIMER_FREQUENCY / 1000;


inline static uint32_t timeDifference(uint32_t time, uint32_t now) {
    return time - now;
}

inline static bool hasExpired(uint32_t time, uint32_t now) {
    return (time - now - 1) > 0xfff0000;
}

void Timer::scheduleAt(uint32_t time) {
    expired = false;
    Scheduler::scheduleTimer(this, time);
}

void Timer::scheduleIn(uint32_t delay) {
    expired = false;
    Scheduler::scheduleTimer(this, millis() + delay);
}

void Timer::cancel() {
    Scheduler::cancelTimer(this);
}



HardwareTimer* Scheduler::hardwareTimer = nullptr;
int Scheduler::numSchedules = 0;
Scheduler::TimerSchedule Scheduler::schedules[MAX_TIMERS];
bool Scheduler::isInInterruptHandler = false;


void Scheduler::setHardwareTimer(HardwareTimer* timer)
{
    hardwareTimer = timer;
}

void Scheduler::start()
{
    hardwareTimer->setPrescaleFactor(F_CPU / TIMER_FREQUENCY);
    hardwareTimer->attachInterrupt(onInterrupt);
}

void Scheduler::scheduleTimer(Timer* timer, uint32_t time) {
    hardwareTimer->pause();
    removeTimer(timer);
    insertTimer(timer, time);
    reschedule();
}

void Scheduler::cancelTimer(Timer* timer) {
    hardwareTimer->pause();
    removeTimer(timer);
    reschedule();
}

void Scheduler::onInterrupt() {
    isInInterruptHandler = true;
    hardwareTimer->pause();

    uint32_t now = millis();
    while (numSchedules > 0) {
        if (!hasExpired(schedules[0].time, now))
            break;

        Timer* timer = schedules[0].timer;
        removeAt(0);
        timer->expired = true;
        timer->continuation->notifyFromInterrupt();
    }

    isInInterruptHandler = false;
    reschedule();
}

void Scheduler::reschedule() {
    if (isInInterruptHandler || numSchedules == 0)
        return;
    
    uint32_t now = millis();
    uint32_t dealyToFirstTimer = timeDifference(schedules[0].time, now);
    hardwareTimer->setCount(0);
    hardwareTimer->setOverflow(dealyToFirstTimer * MS_SCALE + 1, TICK_FORMAT);
    hardwareTimer->resume();
}

void Scheduler::insertTimer(Timer* timer, uint32_t time) {
    if (numSchedules == MAX_TIMERS)
        __builtin_trap();

    uint32_t now = millis();
    
    // find insertion index
    // (tasks are sorted by time but time wraps around)
    int index = 0;
    uint32_t delay = timeDifference(time, now);
    while (index < numSchedules) {
        if (delay < timeDifference(schedules[index].time, now))
            break;
        index += 1;
    }

    if (index < numSchedules)
        memmove(&schedules[index + 1], &schedules[index], (numSchedules - index) * sizeof(TimerSchedule));

    schedules[index].time = time;
    schedules[index].timer = timer;
    numSchedules += 1;
}

void Scheduler::removeTimer(Timer* timer) {
    // find task to remove
    int index;
    for (index = 0; index < numSchedules; index += 1) {
        if (schedules[index].timer == timer)
            break;
    }

    if (index != numSchedules)
        removeAt(index);
}

void Scheduler::removeAt(int index) {
    numSchedules -= 1;
    if (index < numSchedules)
        memmove(&schedules[index], &schedules[index + 1], (numSchedules - index) * sizeof(TimerSchedule));
}
