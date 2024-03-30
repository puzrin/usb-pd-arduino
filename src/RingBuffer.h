//
// USB Power Delivery for Arduino
// Copyright (c) 2024 Manuel Bleichenbacher
//
// Licensed under MIT License
// https://opensource.org/licenses/MIT
//

#include <stdlib.h>

/**
 * @brief Ring buffer.
 * 
 * A ring buffer is a fixed-size buffer with a head and a tail pointer.
 * Items can be put into the buffer and retrieved from the buffer.
 * If the buffer is full, the put operation will be ignored.
 * If the buffer is empty, the get operation will return false.
 * 
 * @tparam T type of items in the buffer
 * @tparam N size of the buffer
 */
template<typename T, size_t N> struct RingBuffer {

    /**
     * @brief Constructor.
     * 
     */
    RingBuffer() : head(0), tail(0) {}

    /**
     * @brief Resets the buffer.
     * 
     */
    void reset() {
        head = 0;
        tail = 0;
    }

    /**
     * @brief Puts an item into the buffer.
     * 
     * @param item item to put into the buffer
     */
    void put(const T& item) {
        unsigned int h = head;
        unsigned int diff = h - tail;
        if (diff >= N)
            return; // buffer full
        buffer[h % N] = item;
        head = h + 1;
    }

    /**
     * @brief Gets an item from the buffer.
     * 
     * @param item item to get from the buffer
     * @return true if an item was retrieved, false if the buffer is empty
     */
    bool get(T& item) {
        unsigned int t = tail;
        if (t == head)
            return false; // buffer empty
        item = buffer[t % N];
        tail = t + 1;
        return true;
    }

    /**
     * @brief Gets the remaining capacity of the buffer.
     * 
     * @return capacity of the buffer
     */
    int capacity() {
        return (tail - head) % N;
    }

    /**
     * @brief Gets the number of items in the buffer.
     * 
     * @return number of items in the buffer
     */
    int available() {
        return head - tail;
    }

private:
    volatile unsigned int head;
    volatile unsigned int tail;
    T buffer[N];
};
