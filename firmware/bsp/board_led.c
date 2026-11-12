#include "board_led.h"

#include "board.h"

/* WS2812B-3528 data is on SPI6_MOSI (PA7). The SPI clock is generated
 * internally; the SCK pin is not routed to the LED. */
SPI_HandleTypeDef hspi6;

#define WS2812_LOW  0x60U
#define WS2812_HIGH 0x78U
#define WS2812_RESET_BYTES 100U

static uint32_t s_last_update_ms;
static uint32_t s_start_ms;
static float s_hue;

void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
    GPIO_InitTypeDef gpio = {0};
    RCC_PeriphCLKInitTypeDef clk = {0};

    if (hspi->Instance != SPI6) {
        return;
    }

    clk.PeriphClockSelection = RCC_PERIPHCLK_SPI6;
    clk.Spi6ClockSelection = RCC_SPI6CLKSOURCE_HSE; /* 24 MHz / 4 = 6 MHz */
    (void)HAL_RCCEx_PeriphCLKConfig(&clk);

    __HAL_RCC_SPI6_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    gpio.Pin = GPIO_PIN_7;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF8_SPI6;
    HAL_GPIO_Init(GPIOA, &gpio);
}

void board_led_set_rgb(uint8_t r, uint8_t g, uint8_t b)
{
    uint8_t txbuf[24];
    uint8_t reset[WS2812_RESET_BYTES] = {0};
    uint8_t i;

    for (i = 0U; i < 8U; i++) {
        txbuf[7U - i] = ((g & (1U << i)) != 0U) ? WS2812_HIGH : WS2812_LOW;
        txbuf[15U - i] = ((r & (1U << i)) != 0U) ? WS2812_HIGH : WS2812_LOW;
        txbuf[23U - i] = ((b & (1U << i)) != 0U) ? WS2812_HIGH : WS2812_LOW;
    }

    /* Keep the timeout short so an LED update cannot stall the motor loop. */
    (void)HAL_SPI_Transmit(&hspi6, txbuf, sizeof(txbuf), 2U);
    (void)HAL_SPI_Transmit(&hspi6, reset, sizeof(reset), 2U);
}

void board_led_init(void)
{
    hspi6.Instance = SPI6;
    hspi6.Init.Mode = SPI_MODE_MASTER;
    hspi6.Init.Direction = SPI_DIRECTION_2LINES_TXONLY;
    hspi6.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi6.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi6.Init.CLKPhase = SPI_PHASE_2EDGE;
    hspi6.Init.NSS = SPI_NSS_SOFT;
    hspi6.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
    hspi6.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi6.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi6.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi6.Init.CRCPolynomial = 7U;
    hspi6.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
    hspi6.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
    hspi6.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
    hspi6.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
    hspi6.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
    hspi6.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
    hspi6.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
    hspi6.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
    hspi6.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
    hspi6.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
    hspi6.Init.IOSwap = SPI_IO_SWAP_DISABLE;

    if (HAL_SPI_Init(&hspi6) != HAL_OK) {
        return;
    }

    s_start_ms = board_millis();
    s_last_update_ms = s_start_ms;
    s_hue = 0.0f;
    board_led_set_rgb(0U, 0U, 0U);
}

static void rainbow(uint8_t index, uint8_t *r, uint8_t *g, uint8_t *b)
{
    static const uint8_t table[7][3] = {
        {255U, 0U, 0U},
        {255U, 255U, 0U},
        {0U, 255U, 0U},
        {0U, 255U, 255U},
        {0U, 0U, 255U},
        {255U, 0U, 255U},
        {255U, 0U, 0U},
    };
    *r = table[index][0];
    *g = table[index][1];
    *b = table[index][2];
}

void board_led_effect_poll(void)
{
    uint32_t now;
    uint32_t elapsed;
    uint32_t phase_ms;
    float phase;
    float breath;
    float value;
    float seg;
    uint8_t idx;
    float frac;
    uint8_t r0;
    uint8_t g0;
    uint8_t b0;
    uint8_t r1;
    uint8_t g1;
    uint8_t b1;

    if (hspi6.Instance != SPI6) {
        return;
    }

    now = board_millis();
    if ((now - s_last_update_ms) < 100U) {
        return;
    }
    s_last_update_ms = now;

    /* Slow hue flow (~8 s per full wheel). */
    s_hue += 0.0025f;
    if (s_hue >= 1.0f) {
        s_hue -= 1.0f;
    }

    /* Breathing brightness with a 4 s period, smoothstep shaped. */
    elapsed = now - s_start_ms;
    phase_ms = elapsed % 4000U;
    phase = (float)phase_ms / 4000.0f;
    breath = (phase < 0.5f) ? (phase * 2.0f) : ((1.0f - phase) * 2.0f);
    breath = breath * breath * (3.0f - (2.0f * breath));
    value = 0.12f + (0.88f * breath);

    seg = s_hue * 6.0f;
    idx = (uint8_t)seg;
    if (idx > 5U) {
        idx = 5U;
    }
    frac = seg - (float)idx;
    rainbow(idx, &r0, &g0, &b0);
    rainbow((uint8_t)(idx + 1U), &r1, &g1, &b1);

    board_led_set_rgb(
        (uint8_t)(((float)r0 + ((float)r1 - (float)r0) * frac) * value),
        (uint8_t)(((float)g0 + ((float)g1 - (float)g0) * frac) * value),
        (uint8_t)(((float)b0 + ((float)b1 - (float)b0) * frac) * value));
}
