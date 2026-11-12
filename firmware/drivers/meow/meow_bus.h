#ifndef MEOW_BUS_H
#define MEOW_BUS_H

#include "canfd.h"
#include "canopen.h"
#include "meow_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct meow_bus {
    /* Owned by the application/BSP, not by the Meow driver. */
    canfd_bus_t *can;
    canopen_t co;

    uint8_t host_node_id;
    uint16_t rpdo1_cob_id;
    uint16_t rpdo2_cob_id;

    meow_motor_t *motors[MEOW_MAX_MOTORS];
    uint8_t motor_count;

    uint8_t discovered[MEOW_MAX_DISCOVERED];
    uint8_t discovered_count;

    bool sync_enabled;
    uint32_t last_mode_tx_ms;
    uint32_t last_bus_off_recovery_ms;
    uint32_t tx_frames;
    uint32_t tx_dropped;
    uint32_t bus_off_count;
} meow_bus_t;

meow_status_t meow_bus_init(meow_bus_t *bus,
                            canfd_bus_t *can,
                            uint8_t host_node_id);
meow_status_t meow_bus_start(meow_bus_t *bus);

void meow_bus_poll(meow_bus_t *bus);
meow_status_t meow_bus_discover(meow_bus_t *bus, uint32_t listen_ms);

int meow_bus_add_motor(meow_bus_t *bus, meow_motor_t *motor, uint8_t node_id);
meow_status_t meow_bus_configure(meow_bus_t *bus, const meow_mit_range_t *range);

void meow_bus_set_sync(meow_bus_t *bus, bool enabled);
void meow_bus_sync(meow_bus_t *bus);

meow_motor_t *meow_bus_find_motor(meow_bus_t *bus, uint8_t node_id);
meow_motor_t *meow_bus_find_slot(meow_bus_t *bus, uint8_t slot);

#ifdef __cplusplus
}
#endif

#endif /* MEOW_BUS_H */
