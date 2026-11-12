#include "board.h"
#include "board_clock.h"
#include "board_led.h"
#include "board_uart.h"

/*
 * DM-MC02 power enables (active high):
 *   PC13 = Power_OUT2_EN (XT30 2+2 output 2)
 *   PC14 = Power_OUT1_EN (XT30 2+2 output 1)
 *   PC15 = Power_5V_EN   (external 5 V rail, also feeds the CAN transceivers)
 *
 * The CAN transceivers are powered from VCC_5V, so PC15 must be high before
 * any CAN traffic is attempted. The high-current outputs are left disabled.
 */
void board_power_init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13 | GPIO_PIN_14, GPIO_PIN_RESET);

    gpio.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &gpio);
}

void board_motor_power_set(board_motor_power_port_t port, bool on)
{
    uint16_t pin;

    if (port == BOARD_MOTOR_POWER_OUT1) {
        pin = GPIO_PIN_14;
    } else if (port == BOARD_MOTOR_POWER_OUT2) {
        pin = GPIO_PIN_13;
    } else {
        return;
    }

    HAL_GPIO_WritePin(GPIOC, pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

bool board_motor_power_get(board_motor_power_port_t port)
{
    uint16_t pin;

    if (port == BOARD_MOTOR_POWER_OUT1) {
        pin = GPIO_PIN_14;
    } else if (port == BOARD_MOTOR_POWER_OUT2) {
        pin = GPIO_PIN_13;
    } else {
        return false;
    }

    return HAL_GPIO_ReadPin(GPIOC, pin) == GPIO_PIN_SET;
}

void board_init(void)
{
    /* Enable the CPU caches before running from flash at 500 MHz. */
    SCB_EnableICache();
    SCB_EnableDCache();

    if (HAL_Init() != HAL_OK) {
        while (1) {
        }
    }

    board_clock_init();
    board_uart_init(115200U);
    board_power_init();
    board_led_init();

    __enable_irq();
}

uint32_t board_millis(void)
{
    return HAL_GetTick();
}

void board_delay_ms(uint32_t ms)
{
    HAL_Delay(ms);
}
