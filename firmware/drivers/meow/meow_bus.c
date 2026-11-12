#include "meow_bus.h"

#include <string.h>

#include "board.h"
#include "meow_codec.h"
#include "meow_motor.h"

#define BUS_OFF_RETRY_MS 1000U
#define MODE_REFRESH_MS 100U

static void record_discovered(meow_bus_t *bus, uint8_t node_id)
{
    uint8_t i;

    if ((node_id == 0U) || (node_id > 127U)) {
        return;
    }
    for (i = 0U; i < bus->discovered_count; i++) {
        if (bus->discovered[i] == node_id) {
            return;
        }
    }
    if (bus->discovered_count < MEOW_MAX_DISCOVERED) {
        bus->discovered[bus->discovered_count++] = node_id;
    }
}

static void meow_bus_rx_cb(void *ctx, uint32_t cob_id, const uint8_t *data, uint8_t len)
{
    meow_bus_t *bus = (meow_bus_t *)ctx;
    uint8_t node_id = (uint8_t)(cob_id & 0x7FU);
    meow_motor_t *motor;
    uint32_t now = board_millis();

    if ((bus == 0) || (node_id == 0U)) {
        return;
    }

    if ((cob_id & 0x780U) == 0x700U) {
        record_discovered(bus, node_id);
    }

    motor = meow_bus_find_motor(bus, node_id);
    if (motor != 0) {
        meow_motor_on_frame(motor, cob_id, data, len, now);
    }
}

meow_status_t meow_bus_init(meow_bus_t *bus,
                            board_fdcan_id_t can_id,
                            uint8_t host_node_id)
{
    HAL_StatusTypeDef hst;

    if (bus == 0) {
        return MEOW_ERR_INVALID_ARG;
    }

    memset(bus, 0, sizeof(*bus));
    bus->host_node_id = (uint8_t)(host_node_id & 0x7FU);
    bus->rpdo1_cob_id = (uint16_t)(0x180U + bus->host_node_id);
    bus->rpdo2_cob_id = (uint16_t)(0x280U + bus->host_node_id);

    hst = canfd_bus_init(&bus->can, can_id, true);
    if (hst != HAL_OK) {
        return MEOW_ERR_IO;
    }

    canopen_init(&bus->co, &bus->can, bus->host_node_id, meow_bus_rx_cb, bus);

    /*
     * Keep the host heartbeat off until every motor has finished SDO
     * configuration. The motor arms its consumer watchdog on the first
     * heartbeat, so early heartbeats can cause a false 0x8130 if a later
     * SDO sequence blocks the control loop.
     */
    canopen_set_heartbeat(&bus->co, false);
    return MEOW_OK;
}

meow_status_t meow_bus_start(meow_bus_t *bus)
{
    if (bus == 0) {
        return MEOW_ERR_INVALID_ARG;
    }
    return (canfd_bus_start(&bus->can) == HAL_OK) ? MEOW_OK : MEOW_ERR_IO;
}

meow_status_t meow_bus_discover(meow_bus_t *bus, uint32_t listen_ms)
{
    uint32_t start;

    if (bus == 0) {
        return MEOW_ERR_INVALID_ARG;
    }

    bus->discovered_count = 0U;
    (void)canopen_nmt(&bus->co, CO_NMT_CMD_PREOP, 0U);

    start = board_millis();
    while ((board_millis() - start) < listen_ms) {
        meow_bus_poll(bus);
        board_delay_ms(1U);
    }

    return MEOW_OK;
}

int meow_bus_add_motor(meow_bus_t *bus, meow_motor_t *motor, uint8_t node_id)
{
    uint8_t i;

    if ((bus == 0) || (motor == 0) || (node_id == 0U) || (node_id > 127U)) {
        return -1;
    }
    if (bus->motor_count >= MEOW_MAX_MOTORS) {
        return -1;
    }
    for (i = 0U; i < bus->motor_count; i++) {
        if (bus->motors[i]->node_id == node_id) {
            return -1;
        }
    }

    memset(motor, 0, sizeof(*motor));
    motor->node_id = node_id;
    motor->slot = bus->motor_count;
    motor->total_motors = 0U;
    bus->motors[bus->motor_count] = motor;
    bus->motor_count++;
    return (int)motor->slot;
}

meow_motor_t *meow_bus_find_motor(meow_bus_t *bus, uint8_t node_id)
{
    uint8_t i;
    if (bus == 0) {
        return 0;
    }
    for (i = 0U; i < bus->motor_count; i++) {
        if (bus->motors[i]->node_id == node_id) {
            return bus->motors[i];
        }
    }
    return 0;
}

meow_motor_t *meow_bus_find_slot(meow_bus_t *bus, uint8_t slot)
{
    if ((bus == 0) || (slot >= bus->motor_count)) {
        return 0;
    }
    return bus->motors[slot];
}

meow_status_t meow_bus_configure(meow_bus_t *bus, const meow_mit_range_t *range)
{
    meow_status_t first_error = MEOW_OK;
    uint8_t i;

    if ((bus == 0) || (bus->motor_count == 0U)) {
        return MEOW_ERR_NOT_CONFIGURED;
    }

    for (i = 0U; i < bus->motor_count; i++) {
        meow_motor_t *motor = bus->motors[i];
        meow_status_t st;

        motor->total_motors = bus->motor_count;

        if (!motor->identified) {
            st = meow_motor_probe(bus, motor);
            if (st != MEOW_OK) {
                if (first_error == MEOW_OK) {
                    first_error = st;
                }
                motor->ready = false;
                continue;
            }
        }

        st = meow_motor_init(bus, motor, range);
        if (st != MEOW_OK) {
            if (first_error == MEOW_OK) {
                first_error = st;
            }
            motor->ready = false;
        }
    }

    bus->sync_enabled = true;
    for (i = 0U; i < bus->motor_count; i++) {
        bus->motors[i]->mode_dirty = true;
    }
    return first_error;
}

void meow_bus_set_sync(meow_bus_t *bus, bool enabled)
{
    if (bus != 0) {
        bus->sync_enabled = enabled;
    }
}

void meow_bus_sync(meow_bus_t *bus)
{
    uint8_t rpdo1[MEOW_MAX_MOTORS * 8U];
    uint8_t rpdo2[MEOW_MAX_MOTORS];
    uint8_t total;
    uint8_t slot;
    bool send_modes = false;
    uint32_t now;

    if ((bus == 0) || !bus->sync_enabled || (bus->motor_count == 0U)) {
        return;
    }

    total = bus->motor_count;
    memset(rpdo1, 0, sizeof(rpdo1));
    memset(rpdo2, 0, sizeof(rpdo2));

    for (slot = 0U; slot < total; slot++) {
        meow_motor_t *motor = meow_bus_find_slot(bus, slot);

        if (motor != 0) {
            if (motor->ready && motor->enabled && !motor->fault && motor->cmd_valid) {
                meow_pack_mit(&motor->mit_range,
                              motor->cmd_position_rev,
                              motor->cmd_velocity_rev_s,
                              motor->cmd_kp_nm_rev,
                              motor->cmd_kd_nm_s_rev,
                              motor->cmd_torque_nm,
                              &rpdo1[slot * 8U]);
            }
            if (motor->enabled) {
                rpdo2[slot] = motor->mode;
            } else {
                rpdo2[slot] = MEOW_MODE_DISABLED;
            }
            if (motor->mode_dirty) {
                send_modes = true;
            }
        }
    }

    now = board_millis();
    if ((now - bus->last_mode_tx_ms) >= MODE_REFRESH_MS) {
        send_modes = true;
    }

    /* Always send the MIT lanes first, so the motor sees a safe target before
     * a mode transition (especially Disabled -> MIT). */
    {
        uint8_t len = (uint8_t)(total * 8U);
        bool fd = (len > 8U);
        if (canfd_send_ex(&bus->can, bus->rpdo1_cob_id, rpdo1, len, fd, true, 0U) == HAL_OK) {
            bus->tx_frames++;
        } else {
            bus->tx_dropped++;
        }
    }

    if (send_modes) {
        if (canfd_send(&bus->can, bus->rpdo2_cob_id, rpdo2, total, false, 0U) == HAL_OK) {
            bus->tx_frames++;
        } else {
            bus->tx_dropped++;
        }
        bus->last_mode_tx_ms = now;
        for (slot = 0U; slot < total; slot++) {
            if (bus->motors[slot] != 0) {
                bus->motors[slot]->mode_dirty = false;
            }
        }
    }
}

void meow_bus_poll(meow_bus_t *bus)
{
    canfd_status_t cs;
    uint32_t now;
    uint8_t i;

    if (bus == 0) {
        return;
    }

    canopen_poll(&bus->co);

    now = board_millis();
    canfd_get_status(&bus->can, &cs);
    if (cs.bus_off) {
        if ((now - bus->last_bus_off_recovery_ms) >= BUS_OFF_RETRY_MS) {
            bus->last_bus_off_recovery_ms = now;
            bus->bus_off_count++;
            (void)canfd_recover_bus_off(&bus->can);
            for (i = 0U; i < bus->motor_count; i++) {
                bus->motors[i]->ready = false;
                bus->motors[i]->enabled = false;
                bus->motors[i]->cmd_valid = false;
            }
        }
    }

    for (i = 0U; i < bus->motor_count; i++) {
        meow_motor_t *motor = bus->motors[i];
        uint32_t timeout_ms;

        if (motor->last_rx_ms == 0U) {
            continue;
        }

        timeout_ms = (uint32_t)motor->heartbeat_period_ms * 3U;
        if (timeout_ms < 500U) {
            timeout_ms = 500U;
        }
        if ((now - motor->last_rx_ms) > timeout_ms) {
            motor->state.online = false;
            motor->enabled = false;
            motor->ready = false;
            motor->cmd_valid = false;
        } else {
            motor->state.online = true;
        }

        if (motor->state.error_code != 0U) {
            motor->fault = true;
        }
    }
}
