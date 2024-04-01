//
// USB Power Delivery for Arduino
// Copyright (c) 2023 Manuel Bleichenbacher
//
// Licensed under MIT License
// https://opensource.org/licenses/MIT
//

#include <Arduino.h>
#include "TaskScheduler.h"

#if defined(ARDUINO_ARCH_STM32)
#include <HardwareTimer.h>
#endif

#define _countof(a) (sizeof(a) / sizeof(*(a)))

inline static uint32_t timeDifference(uint32_t time, uint32_t now) {
    return time - now;
}

inline static bool hasExpired(uint32_t time, uint32_t now) {
    return (time - now) > 0xfff0000;
}

TaskScheduler Scheduler{};

#if defined(ARDUINO_ARCH_STM32)

// STM32: As the Arudino and the underlying HAL library incur a considerable overhead,
// direct access to the timer would be preferred. However, the Arduino library
// must be used for the interrupt handler. Otherwise, the linker will encounter
// duplicate symbols.

#if defined(STM32L4xx) || defined(STM32G0xx) || defined(STM32G4xx)
#define TIMER TIM7
#elif defined(STM32F103xB) || defined(STM32F4xx)
#define TIMER TIM3
#endif

HardwareTimer timer(TIMER);

#elif defined(ARDUINO_ARCH_ESP32)

hw_timer_t* timer = nullptr;

#endif


TaskScheduler::TaskScheduler() : numScheduledTasks(-1) { }

void TaskScheduler::start() {
    numScheduledTasks = 0;

#if defined(ARDUINO_ARCH_STM32)

    // configure timer (advances every millisecond)
    timer.setPrescaleFactor((timer.getTimerClkFreq() + 500000) / 1000000);
    timer.attachInterrupt(onInterrupt);
    // one-pulse mode
    TIMER->CR1 |= TIM_CR1_OPM;
    // disable ARR buffering
    TIMER->CR1 &= ~TIM_CR1_ARPE_Msk;

#elif defined(ARDUINO_ARCH_ESP32)

    timer = timerBegin(0, (uint16_t)((APB_CLK_FREQ + 500000) / 1000000), true);
    timerAttachInterrupt(timer, &onInterrupt, true);

#endif
}

void TaskScheduler::scheduleTaskAfter(int taskId, TaskFunction taskFunction, uint32_t delay) {
    scheduleTaskAt(taskId, taskFunction, micros() + delay);
}

void TaskScheduler::scheduleTaskAt(int taskId, TaskFunction taskFunction, uint32_t time) {
    if (numScheduledTasks == -1)
        start();

    if (numScheduledTasks >= static_cast<int>(_countof(scheduledTasks)))
        __builtin_trap();

    pause();
    uint32_t now = micros();

    // find insertion index
    // (tasks are sorted by time but time wraps around)
    int index = 0;
    uint32_t delay = timeDifference(time, now);
    while (index < numScheduledTasks) {
        if (delay < timeDifference(scheduledTasks[index].scheduledTime, now))
            break;
        index += 1;
    }

    // move elements after insertion point
    for (int i = numScheduledTasks; i > index; i -= 1)
        scheduledTasks[i] = scheduledTasks[i - 1];

    numScheduledTasks += 1;

    // set values
    scheduledTasks[index].scheduledTime = time;
    scheduledTasks[index].id = taskId;
    scheduledTasks[index].function = taskFunction;

    checkPendingTasks();
}

void TaskScheduler::cancelTask(int taskId) {
    if (numScheduledTasks == -1)
        return;

    pause();

    // find task to remove
    int index;
    for (index = 0; index < numScheduledTasks; index += 1) {
        if (scheduledTasks[index].id == taskId)
            break;
    }

    if (index < numScheduledTasks) {
        numScheduledTasks -= 1;

        // move remaining elements
        for (int i = index; i < numScheduledTasks; i += 1)
            scheduledTasks[i] = scheduledTasks[i + 1];
    }

    checkPendingTasks();
}

void TaskScheduler::cancelAllTasks() {
    if (numScheduledTasks == -1)
        return;

    pause();
    numScheduledTasks = 0;
}

void TaskScheduler::checkPendingTasks() {
#if defined(ARDUINO_ARCH_ESP32)
    timerStop(timer);
#endif

    uint32_t now = 0;

    while (true) {
        if (numScheduledTasks == 0)
            return; // no pending tasks

        now = micros();
        if (!hasExpired(scheduledTasks[0].scheduledTime, now))
            break; // next task has not yet expired

        TaskFunction taskFunction = scheduledTasks[0].function;
        numScheduledTasks -= 1;

        // move remaining elements if needed
        for (int i = 0; i < numScheduledTasks; i += 1)
            scheduledTasks[i] = scheduledTasks[i + 1];

        // execute task
        taskFunction();
    }

    uint32_t delayToFirstTask = timeDifference(scheduledTasks[0].scheduledTime, micros());

    // restart timer
#if defined(ARDUINO_ARCH_STM32)
    if (delayToFirstTask > 0xffff)
        delayToFirstTask = 0xffff;

    TIMER->CNT = 0;
    TIMER->ARR = delayToFirstTask;
    TIMER->CR1 |= TIM_CR1_CEN;
#elif defined(ARDUINO_ARCH_ESP32)
    timerWrite(timer, 0);
    timerAlarmWrite(timer, delayToFirstTask, false);
    timerAlarmEnable(timer);
    timerStart(timer);
#endif
}

void TaskScheduler::pause() {
#if defined(ARDUINO_ARCH_STM32)
    TIMER->CR1 &= ~TIM_CR1_CEN_Msk;
#elif defined(ARDUINO_ARCH_ESP32)
    timerAlarmDisable(timer);
#endif
}

void TaskScheduler::onInterrupt() {
    Scheduler.checkPendingTasks();
}
