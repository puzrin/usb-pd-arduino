//
// USB Power Delivery for Arduino
// Copyright (c) 2023 Manuel Bleichenbacher
//
// Licensed under MIT License
// https://opensource.org/licenses/MIT
//

#pragma once

#include <Arduino.h>
#include <stdint.h>

#ifndef ARDUINO_ISR_ATTR
#define ARDUINO_ISR_ATTR
#endif


/**
 * @brief Scheduler to execute tasks in the future.
 * 
 * A task is represented by a function that will be called.
 * 
 * Time for scheduling is specified in µs. The maximum delay
 * into the future is slightly more than 1 hour. 
 */
struct TaskScheduler {
    TaskScheduler();

    /**
     * @brief Function type used for task.
     * 
     */
    typedef void (*TaskFunction)();

    /**
     * @brief Schedules a task to be executed after a delay.
     * 
     * @param task task to execute
     * @param delay delay, in µs
     */
    ARDUINO_ISR_ATTR void scheduleTaskAfter(TaskFunction task, uint32_t delay);

    /**
     * @brief Schedules a task to be executed at a time in the future.
     * 
     * @param task task to execute
     * @param time time, in µs, same base as `micros()`
     */
    ARDUINO_ISR_ATTR void scheduleTaskAt(TaskFunction task, uint32_t time);

    /**
     * @brief Cancels the execution of a previously scheduled task.
     * 
     * @param task task to cancel 
     */
    ARDUINO_ISR_ATTR void cancelTask(TaskFunction task);

    /**
     * @brief Cancels the execution of all scheduled tasks.
     */
    ARDUINO_ISR_ATTR void cancelAllTasks();

private:
    int numScheduledTasks;
    uint32_t scheduledTimes[10];
    TaskFunction scheduledFunctions[10];

    void start();
    ARDUINO_ISR_ATTR void pause();
    ARDUINO_ISR_ATTR void checkPendingTasks();
    static ARDUINO_ISR_ATTR void onInterrupt();
};

/**
 * @brief Task scheduler.
 * 
 * Global scheduler instance for executing tasks in the future
 */
extern TaskScheduler Scheduler;
