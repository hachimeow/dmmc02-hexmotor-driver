#ifndef CANFD_H
#define CANFD_H

#include <stdbool.h>
#include <stdint.h>

#include "board_fdcan.h"
#include "stm32h7xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CANFD_MAX_DATA 64U
#define CANFD_RX_RING_SIZE 64U

typedef struct {
    uint32_t id;
    uint8_t len;
    bool fd;
    bool brs;
    uint8_t data[CANFD_MAX_DATA];
} canfd_frame_t;

typedef struct {
    uint8_t tx_error_count;
    uint8_t rx_error_count;
    uint8_t last_error_code;
    bool error_passive;
    bool bus_off;
} canfd_status_t;

typedef struct canfd_bus {
    FDCAN_HandleTypeDef *hfdcan;
    board_fdcan_id_t id;
    bool fd_capable;
    bool started;

    volatile uint16_t rx_head;
    volatile uint16_t rx_tail;
    canfd_frame_t rx_ring[CANFD_RX_RING_SIZE];

    volatile uint32_t rx_frames;
    volatile uint32_t tx_frames;
    volatile uint32_t rx_dropped;
    volatile uint32_t tx_dropped;
    volatile uint32_t rx_overrun;
} canfd_bus_t;

HAL_StatusTypeDef canfd_bus_init(canfd_bus_t *bus, board_fdcan_id_t id, bool fd_capable);
HAL_StatusTypeDef canfd_bus_start(canfd_bus_t *bus);
HAL_StatusTypeDef canfd_bus_stop(canfd_bus_t *bus);

HAL_StatusTypeDef canfd_send(canfd_bus_t *bus,
                             uint32_t id,
                             const uint8_t *data,
                             uint8_t len,
                             bool fd,
                             uint32_t timeout_ms);

HAL_StatusTypeDef canfd_send_ex(canfd_bus_t *bus,
                                uint32_t id,
                                const uint8_t *data,
                                uint8_t len,
                                bool fd,
                                bool brs,
                                uint32_t timeout_ms);

bool canfd_receive(canfd_bus_t *bus, canfd_frame_t *frame);
void canfd_get_status(canfd_bus_t *bus, canfd_status_t *status);
HAL_StatusTypeDef canfd_recover_bus_off(canfd_bus_t *bus);

/* Called by the HAL callback in the FDCAN interrupt path. */
void canfd_irq_rx(FDCAN_HandleTypeDef *hfdcan);

#ifdef __cplusplus
}
#endif

#endif /* CANFD_H */
