#pragma once
#include <stdint.h>
typedef struct { int unused; } UART_HandleTypeDef;
uint32_t HAL_GetTick(void);
int HAL_UART_Transmit(UART_HandleTypeDef *, uint8_t *, uint16_t, uint32_t);
