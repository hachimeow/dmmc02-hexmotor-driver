#include "board_fdcan.h"

/* 5 Mbit/s data phase transceiver delay compensation (data time quanta). */
#define BOARD_FDCAN_TDC_OFFSET 12U
#define BOARD_FDCAN_TDC_FILTER 10U

FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan2;
FDCAN_HandleTypeDef hfdcan3;

FDCAN_HandleTypeDef *board_fdcan_handle(board_fdcan_id_t id)
{
    switch (id) {
    case BOARD_FDCAN_1:
        return &hfdcan1;
    case BOARD_FDCAN_2:
        return &hfdcan2;
    case BOARD_FDCAN_3:
        return &hfdcan3;
    default:
        return 0;
    }
}

static void board_fdcan_gpio_init(board_fdcan_id_t id)
{
    GPIO_InitTypeDef gpio = {0};

    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;

    switch (id) {
    case BOARD_FDCAN_1:
        __HAL_RCC_GPIOD_CLK_ENABLE();
        gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1;
        gpio.Alternate = GPIO_AF9_FDCAN1;
        HAL_GPIO_Init(GPIOD, &gpio);
        break;

    case BOARD_FDCAN_2:
        __HAL_RCC_GPIOB_CLK_ENABLE();
        gpio.Pin = GPIO_PIN_5 | GPIO_PIN_6;
        gpio.Alternate = GPIO_AF9_FDCAN2;
        HAL_GPIO_Init(GPIOB, &gpio);
        break;

    case BOARD_FDCAN_3:
        __HAL_RCC_GPIOD_CLK_ENABLE();
        gpio.Pin = GPIO_PIN_12 | GPIO_PIN_13;
        gpio.Alternate = GPIO_AF5_FDCAN3;
        HAL_GPIO_Init(GPIOD, &gpio);
        break;

    default:
        break;
    }
}

static void board_fdcan_clock_init(void)
{
    RCC_PeriphCLKInitTypeDef clk = {0};

    /* FDCAN kernel clock = PLL1Q = 100 MHz. */
    clk.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    clk.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL;
    (void)HAL_RCCEx_PeriphCLKConfig(&clk);

    __HAL_RCC_FDCAN_CLK_ENABLE();
}

HAL_StatusTypeDef board_fdcan_init(board_fdcan_id_t id,
                                   uint32_t message_ram_offset,
                                   bool fd_with_brs)
{
    FDCAN_HandleTypeDef *hfdcan = board_fdcan_handle(id);
    FDCAN_FilterTypeDef filter = {0};

    if (hfdcan == 0) {
        return HAL_ERROR;
    }

    board_fdcan_clock_init();
    board_fdcan_gpio_init(id);

    if (id == BOARD_FDCAN_1) {
        hfdcan->Instance = FDCAN1;
    } else if (id == BOARD_FDCAN_2) {
        hfdcan->Instance = FDCAN2;
    } else {
        hfdcan->Instance = FDCAN3;
    }

    hfdcan->Init.FrameFormat = fd_with_brs ? FDCAN_FRAME_FD_BRS : FDCAN_FRAME_CLASSIC;
    hfdcan->Init.Mode = FDCAN_MODE_NORMAL;
    hfdcan->Init.AutoRetransmission = ENABLE;
    hfdcan->Init.TransmitPause = DISABLE;
    hfdcan->Init.ProtocolException = ENABLE;

    /*
     * Match the vendor Meow setting:
     *   nominal: 100 / (5 * (1 + 15 + 4)) = 1 Mbit/s, sample point 80 %, SJW 5
     *   data:    100 / (1 * (1 + 14 + 5)) = 5 Mbit/s, sample point 75 %, DSJW 3
     */
    hfdcan->Init.NominalPrescaler = 5U;
    hfdcan->Init.NominalSyncJumpWidth = 5U;
    hfdcan->Init.NominalTimeSeg1 = 15U;
    hfdcan->Init.NominalTimeSeg2 = 4U;

    hfdcan->Init.DataPrescaler = 1U;
    hfdcan->Init.DataSyncJumpWidth = 3U;
    hfdcan->Init.DataTimeSeg1 = 14U;
    hfdcan->Init.DataTimeSeg2 = 5U;

    hfdcan->Init.MessageRAMOffset = message_ram_offset;
    hfdcan->Init.StdFiltersNbr = 8U;
    hfdcan->Init.ExtFiltersNbr = 0U;
    hfdcan->Init.RxFifo0ElmtsNbr = 16U;
    hfdcan->Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_64;
    hfdcan->Init.RxFifo1ElmtsNbr = 0U;
    hfdcan->Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
    hfdcan->Init.RxBuffersNbr = 0U;
    hfdcan->Init.RxBufferSize = FDCAN_DATA_BYTES_8;
    hfdcan->Init.TxEventsNbr = 0U;
    hfdcan->Init.TxBuffersNbr = 0U;
    hfdcan->Init.TxFifoQueueElmtsNbr = 16U;
    hfdcan->Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
    hfdcan->Init.TxElmtSize = FDCAN_DATA_BYTES_64;

    if (HAL_FDCAN_Init(hfdcan) != HAL_OK) {
        return HAL_ERROR;
    }

#if defined(BOARD_FDCAN_TDC_OFFSET) && (BOARD_FDCAN_TDC_OFFSET > 0)
    (void)HAL_FDCAN_ConfigTxDelayCompensation(hfdcan,
                                              BOARD_FDCAN_TDC_OFFSET,
                                              BOARD_FDCAN_TDC_FILTER);
    (void)HAL_FDCAN_EnableTxDelayCompensation(hfdcan);
#endif

    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0U;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = 0x000U;
    filter.FilterID2 = 0x000U;
    if (HAL_FDCAN_ConfigFilter(hfdcan, &filter) != HAL_OK) {
        return HAL_ERROR;
    }

    if (HAL_FDCAN_ConfigGlobalFilter(hfdcan,
                                    FDCAN_ACCEPT_IN_RX_FIFO0,
                                    FDCAN_REJECT,
                                    FDCAN_REJECT_REMOTE,
                                    FDCAN_REJECT_REMOTE) != HAL_OK) {
        return HAL_ERROR;
    }

    if (HAL_FDCAN_ActivateNotification(hfdcan,
                                       FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
                                       0U) != HAL_OK) {
        return HAL_ERROR;
    }

    if (id == BOARD_FDCAN_1) {
        HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 5U, 0U);
        HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
    } else if (id == BOARD_FDCAN_2) {
        HAL_NVIC_SetPriority(FDCAN2_IT0_IRQn, 5U, 0U);
        HAL_NVIC_EnableIRQ(FDCAN2_IT0_IRQn);
    } else {
        HAL_NVIC_SetPriority(FDCAN3_IT0_IRQn, 5U, 0U);
        HAL_NVIC_EnableIRQ(FDCAN3_IT0_IRQn);
    }

    return HAL_OK;
}

HAL_StatusTypeDef board_fdcan_start(board_fdcan_id_t id)
{
    FDCAN_HandleTypeDef *hfdcan = board_fdcan_handle(id);
    if (hfdcan == 0) {
        return HAL_ERROR;
    }
    return HAL_FDCAN_Start(hfdcan);
}

HAL_StatusTypeDef board_fdcan_stop(board_fdcan_id_t id)
{
    FDCAN_HandleTypeDef *hfdcan = board_fdcan_handle(id);
    if (hfdcan == 0) {
        return HAL_ERROR;
    }
    return HAL_FDCAN_Stop(hfdcan);
}
