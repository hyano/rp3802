/*
 * Copyright (c) 2025 Hirokuni Yano
 *
 * Released under the MIT license.
 * see https://opensource.org/licenses/MIT
 */
#ifndef FIFO_H__
#define FIFO_H__

#include <cstddef>
#include <cstdint>

template <typename T, size_t Depth>
class FIFO
{
    static_assert((Depth & (Depth - 1)) == 0, "Depth must be power of 2");

public:
    FIFO()
    {
        reset();
    }

    inline bool push(const T &item)
    {
        const size_t next_head = (head + 1) & mask;
        if (next_head == tail)
            return false; // full

        buffer[head] = item;
        head = next_head;
        return true;
    }

    inline bool pop(T &item)
    {
        if (tail == head)
            return false; // empty

        item = buffer[tail];
        tail = (tail + 1) & mask;
        return true;
    }

    inline bool peek(T &item)
    {
        if (tail == head)
            return false; // empty

        item = buffer[tail];
        return true;
    }

    inline bool is_empty() const
    {
        return head == tail;
    }

    inline bool is_full() const
    {
        return ((head + 1) & mask) == tail;
    }

    inline void reset()
    {
        head = tail = 0;
    }

private:
    static constexpr size_t mask = Depth - 1;
    T buffer[Depth];

    volatile size_t head;
    volatile size_t tail;
};

#endif // FIFO_H__
