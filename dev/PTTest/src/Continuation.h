//
// USB Power Delivery for Arduino
// Copyright (c) 2024 Manuel Bleichenbacher
//
// Licensed under MIT License
// https://opensource.org/licenses/MIT
//

#pragma once

/**
 * @brief Continuation.
 * 
 * Light-weight concept for implementing long-running operations
 * yielding for all blocking operations.
 * 
 * The basic assumptions are:
 * 
 * - Blocking operations wait until a condition is met.
 * - The conditions depend on external events.
 * - The continuation must be notified that an external event has occurred,
 *   either by an interrupt handler or by the application (Andorid's loop() function).
 * - The non-blocking parts of the operation are quick.
 * 
 * Continuations are modelled after Protothreads.
 * 
 * To implement a long-running operation:
 * 
 * - override the class and implement `run()`.
 * - start `run()` with `CT_BEGIN` and end with `CT_END`.
 * - impelent block operations by waiting for a condition using `CT_WAIT_xxx`.
 * 
 * Note that `CT_WAIT_xxx` may only be called from `run()`, and that local variables
 * in `run()` are lost when `CT_WAIT_xxx` is called.
 */
class Continuation
{
public:
    /// Constructs a new instance.
    Continuation() : _ctLine(0) { }

    /// Virtual descructor.
    virtual ~Continuation() { }

    /// Start long-running operation from the start.
    void start();

    /// Notify continuation from the application that an event has occurred.
    void notifyFromApp();

    /// Notify continuation form an interrupt handler that an event has occurred.
    void notifyFromInterrupt();

protected:
    /**
     * @brief Override to implement the long runnung operation.
     * 
     * Enclose the code in `CT_BEGIN` and `CT_END` macros.
     */
    virtual void run() = 0;

    // Data type for storing the continuation line number.
    typedef unsigned short LineNumber;

    // Continuation line number (of last yield).
    LineNumber _ctLine;
};


/// Required at start of `run()` (without any parentheses, curly braces or semicolons).
#define CT_BEGIN switch (_ctLine) { case 0:

/// Required at end of `run()` (without any parentheses, curly braces or semicolons).
#define CT_END default: ; } _ctLine = 0; return;

/// Wait until given is condition is true (and yield)
#define CT_WAIT_UNTIL(condition) \
    do { _ctLine = __LINE__; case __LINE__: \
    if (!(condition)) return; } while (0)

/// Wait while given is condition is true (and yield)
#define CT_WAIT_WHILE(condition) CT_WAIT_UNTIL(!(condition))
