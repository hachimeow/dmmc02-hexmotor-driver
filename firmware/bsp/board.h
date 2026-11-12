#ifndef BOARD_H
#define BOARD_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32h7xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* DM-MC02 / DM-MC-Board02: STM32H723VGT6, 24 MHz HSE, 500 MHz SYSCLK. */
void board_init(void);
void board_power_init(void);

typedef enum {
    BOARD_MOTOR_POWER_NONE = 0,
    BOARD_MOTOR_POWER_OUT1 = 1,  /* PC14 / P7: CAN1 + VCC_OUT1 */
    BOARD_MOTOR_POWER_OUT2 = 2   /* PC13 / P8: CAN2 + VCC_OUT2 */
} board_motor_power_port_t;

/* Active-high PMOS switches, 5 A continuous each. */
void board_motor_power_set(board_motor_power_port_t port, bool on);
bool board_motor_power_get(board_motor_power_port_t port);

uint32_t board_millis(void);
void board_delay_ms(uint32_t ms);

/* USART1: PA9 TX, PA10 RX, 115200 8N1. */
void board_uart_init(uint32_t baudrate);
void board_uart_write(const uint8_t *data, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_H */
