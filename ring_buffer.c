#include "ring_buffer.h"

void RingBuffer_Init(RingBuffer_t *rb, uint8_t *buffer, uint16_t size)
{
    rb->buffer = buffer;
    rb->size = size;
    rb->head = 0U;
    rb->tail = 0U;
    rb->overflow_count = 0U;
}

bool RingBuffer_Push(RingBuffer_t *rb, uint8_t data)
{
    uint16_t next_head = (uint16_t)(rb->head + 1U);

    if (next_head >= rb->size)
    {
        next_head = 0U;
    }

    if (next_head == rb->tail)
    {
        rb->overflow_count++;
        return false;
    }

    rb->buffer[rb->head] = data;
    rb->head = next_head;

    return true;
}

uint16_t RingBuffer_Write(RingBuffer_t *rb, const uint8_t *data, uint16_t length)
{
    uint16_t written = 0U;

    while (written < length)
    {
        if (!RingBuffer_Push(rb, data[written]))
        {
            break;
        }

        written++;
    }

    return written;
}

bool RingBuffer_Pop(RingBuffer_t *rb, uint8_t *data)
{
    if (rb->head == rb->tail)
    {
        return false;
    }

    *data = rb->buffer[rb->tail];

    rb->tail++;

    if (rb->tail >= rb->size)
    {
        rb->tail = 0U;
    }

    return true;
}

uint16_t RingBuffer_Available(const RingBuffer_t *rb)
{
    if (rb->head >= rb->tail)
    {
        return (uint16_t)(rb->head - rb->tail);
    }

    return (uint16_t)(rb->size - rb->tail + rb->head);
}

uint16_t RingBuffer_Free(const RingBuffer_t *rb)
{
    return (uint16_t)(rb->size - RingBuffer_Available(rb) - 1U);
}

void RingBuffer_Clear(RingBuffer_t *rb)
{
    rb->head = 0U;
    rb->tail = 0U;
    rb->overflow_count = 0U;
}
