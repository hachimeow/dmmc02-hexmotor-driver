#ifndef BOARD_FDCAN_H
#define BOARD_FDCAN_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32h7xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    BOARD_FDCAN_1 = 0,
    BOARD_FDCAN_2 = 1,
    BOARD_FDCAN_3 = 2,
    BOARD_FDCAN_COUNT = 3
} board_fdcan_id_t;

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

/*
 * DM-MC02 FDCAN pins:
 *   FDCAN1: PD0 RX, PD1 TX, AF9
 *   FDCAN2: PB5 RX, PB6 TX, AF9
 *   FDCAN3: PD12 RX, PD13 TX, AF5
 */
FDCAN_HandleTypeDef *board_fdcan_handle(board_fdcan_id_t id);
HAL_StatusTypeDef board_fdcan_init(board_fdcan_id_t id,
                                   uint32_t message_ram_offset,
                                   bool fd_with_brs);
HAL_StatusTypeDef board_fdcan_start(board_fdcan_id_t id);
HAL_StatusTypeDef board_fdcan_stop(board_fdcan_id_t id);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_FDCAN_H */
