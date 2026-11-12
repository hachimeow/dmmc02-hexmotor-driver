#include "board_fdcan.h"
#include "stm32h7xx_hal.h"

void SysTick_Handler(void)
{
    HAL_IncTick();
}

void FDCAN1_IT0_IRQHandler(void)
{
    HAL_FDCAN_IRQHandler(&hfdcan1);
}

void FDCAN2_IT0_IRQHandler(void)
{
    HAL_FDCAN_IRQHandler(&hfdcan2);
}

void FDCAN3_IT0_IRQHandler(void)
{
    HAL_FDCAN_IRQHandler(&hfdcan3);
}

void HardFault_Handler(void)
{
    __disable_irq();
    while (1) {
    }
}

void MemManage_Handler(void)
{
    __disable_irq();
    while (1) {
    }
}

void BusFault_Handler(void)
{
    __disable_irq();
    while (1) {
    }
}

void UsageFault_Handler(void)
{
    __disable_irq();
    while (1) {
    }
}
