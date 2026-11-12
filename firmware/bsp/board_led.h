#ifndef BOARD_LED_H
#define BOARD_LED_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Board WS2812B on SPI6_MOSI (PA7). */
void board_led_init(void);
void board_led_set_rgb(uint8_t r, uint8_t g, uint8_t b);

/* Smooth rainbow hue flow + breathing brightness. Call from the main loop. */
void board_led_effect_poll(void);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_LED_H */
