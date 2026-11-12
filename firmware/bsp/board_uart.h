#ifndef BOARD_UART_H
#define BOARD_UART_H

#include <stdint.h>

#include "stm32h7xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

extern UART_HandleTypeDef huart1;

void board_uart_init(uint32_t baudrate);
void board_uart_write(const uint8_t *data, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_UART_H */
