#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
    uint8_t *buffer;
    uint16_t size;
    volatile uint16_t head;
    volatile uint16_t tail;
    volatile uint32_t overflow_count;
} RingBuffer_t;

void RingBuffer_Init(RingBuffer_t *rb, uint8_t *buffer, uint16_t size);
bool RingBuffer_Push(RingBuffer_t *rb, uint8_t data);
uint16_t RingBuffer_Write(RingBuffer_t *rb, const uint8_t *data, uint16_t length);
bool RingBuffer_Pop(RingBuffer_t *rb, uint8_t *data);
uint16_t RingBuffer_Available(const RingBuffer_t *rb);
uint16_t RingBuffer_Free(const RingBuffer_t *rb);
void RingBuffer_Clear(RingBuffer_t *rb);

#endif
