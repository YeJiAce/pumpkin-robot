#ifndef PUMPKIN_SERIAL_DMA_H
#define PUMPKIN_SERIAL_DMA_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* All public functions are called exclusively by commTask. */
bool SerialDma_Init(void);
void SerialDma_Service(void);
bool SerialDma_ReadByte(uint8_t *byte);
bool SerialDma_TakeRxFault(void);
bool SerialDma_Write(const char *text);
/* Nonblocking binary send; owns a copy until DMA and UART TC complete. */
bool SerialDma_TryWrite(const uint8_t *bytes, size_t length);

#endif
