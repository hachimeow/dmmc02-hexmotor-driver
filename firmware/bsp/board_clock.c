#include "board.h"
#include "board_clock.h"

/*
 * DM-MC02 clock tree (same values as the vendor CubeMX examples):
 *
 *   HSE 24 MHz / PLLM 3 = 8 MHz
 *   VCO = 8 MHz * (62 + 4096/8192) = 500 MHz
 *   SYSCLK = VCO / PLLP(1)        = 500 MHz
 *   HCLK   = SYSCLK / 2           = 250 MHz
 *   PLL1Q  = VCO / 5              = 100 MHz  -> FDCAN kernel clock
 */
void board_clock_init(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    if (HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY) != HAL_OK) {
        while (1) {
        }
    }

    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);
    while (__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY) == 0U) {
    }

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState = RCC_HSE_ON;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLM = 3U;
    osc.PLL.PLLN = 62U;
    osc.PLL.PLLP = 1U;
    osc.PLL.PLLQ = 5U;
    osc.PLL.PLLR = 2U;
    osc.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
    osc.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
    osc.PLL.PLLFRACN = 4096U;

    if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
        while (1) {
        }
    }

    clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                    RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 |
                    RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.SYSCLKDivider = RCC_SYSCLK_DIV1;
    clk.AHBCLKDivider = RCC_HCLK_DIV2;
    clk.APB3CLKDivider = RCC_APB3_DIV2;
    clk.APB1CLKDivider = RCC_APB1_DIV2;
    clk.APB2CLKDivider = RCC_APB2_DIV2;
    clk.APB4CLKDivider = RCC_APB4_DIV2;

    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_3) != HAL_OK) {
        while (1) {
        }
    }

    SystemCoreClockUpdate();
}
