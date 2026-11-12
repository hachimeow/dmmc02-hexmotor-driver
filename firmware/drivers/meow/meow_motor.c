#include "meow_motor.h"

#include <math.h>
#include <string.h>

#include "board.h"
#include "meow_bus.h"
#include "meow_codec.h"

/* Meow custom object dictionary. */
#define OD_ERROR_REGISTER        0x1001U
#define OD_HEARTBEAT_CONSUMER    0x1016U
#define OD_HEARTBEAT_PRODUCER    0x1017U
#define OD_IDENTITY              0x1018U
#define OD_MODE_CMD              0x4401U
#define OD_MODE_DISPLAY          0x4402U
#define OD_DETAILED_ERROR        0x453FU
#define OD_POSITION_ACTUAL       0x4564U
#define OD_TARGET_POSITION       0x457AU
#define OD_TARGET_VELOCITY       0x45FFU
#define OD_TARGET_TORQUE         0x4571U
#define OD_TORQUE_ACTUAL         0x4577U
#define OD_PEAK_TORQUE           0x4576U
#define OD_PROFILE_VELOCITY      0x4581U
#define OD_PROFILE_ACCELERATION  0x4583U
#define OD_PROFILE_DECELERATION  0x4584U
#define OD_MIT_UNCOMPRESSED      0x4102U
#define OD_MIT_COMPRESSED        0x4103U
#define OD_MIT_TEMPERATURES      0x4104U
#define OD_TIMESTAMP             0x4713U

/* Compressed MIT (0x4103) subindices. */
#define MITC_SELECTOR   0x01U
#define MITC_LOWER      0x02U
#define MITC_UPPER      0x03U
#define MITC_POS_MIN    0x04U
#define MITC_POS_MAX    0x05U
#define MITC_VEL_MIN    0x06U
#define MITC_VEL_MAX    0x07U
#define MITC_KP_MIN     0x08U
#define MITC_KP_MAX     0x09U
#define MITC_KD_MIN     0x0AU
#define MITC_KD_MAX     0x0BU
#define MITC_TOR_MIN    0x0CU
#define MITC_TOR_MAX    0x0DU
#define MITC_TOR_LIMIT  0x0EU

#define SDO_TIMEOUT_MS       100U
#define SDO_GAP_MS           2U
#define NMT_WAIT_MS          3000U
#define MODE_WAIT_MS         500U
#define HEARTBEAT_TIMEOUT_MS 250U

typedef struct {
    uint16_t index;
    uint8_t subindex;
    uint8_t bits;
} pdo_entry_t;

static meow_status_t map_co(co_status_t st)
{
    switch (st) {
    case CO_OK:
        return MEOW_OK;
    case CO_ERR_IO:
        return MEOW_ERR_IO;
    case CO_ERR_TIMEOUT:
        return MEOW_ERR_TIMEOUT;
    case CO_ERR_BUSY:
        return MEOW_ERR_BUSY;
    case CO_ERR_SDO_ABORT:
        return MEOW_ERR_SDO_ABORT;
    case CO_ERR_SDO_PROTOCOL:
        return MEOW_ERR_SDO_PROTOCOL;
    case CO_ERR_NOT_SUPPORTED:
        return MEOW_ERR_NOT_SUPPORTED;
    case CO_ERR_INVALID_ARG:
        return MEOW_ERR_INVALID_ARG;
    default:
        return MEOW_ERR_IO;
    }
}

static meow_status_t sdo_u8(meow_bus_t *bus, uint8_t nid, uint16_t index,
                            uint8_t sub, uint8_t value)
{
    return map_co(canopen_download_u8(&bus->co, nid, index, sub, value, SDO_TIMEOUT_MS));
}

static meow_status_t sdo_u16(meow_bus_t *bus, uint8_t nid, uint16_t index,
                             uint8_t sub, uint16_t value)
{
    return map_co(canopen_download_u16(&bus->co, nid, index, sub, value, SDO_TIMEOUT_MS));
}

static meow_status_t sdo_u32(meow_bus_t *bus, uint8_t nid, uint16_t index,
                             uint8_t sub, uint32_t value)
{
    return map_co(canopen_download_u32(&bus->co, nid, index, sub, value, SDO_TIMEOUT_MS));
}

static meow_status_t sdo_i16(meow_bus_t *bus, uint8_t nid, uint16_t index,
                             uint8_t sub, int16_t value)
{
    return map_co(canopen_download_i16(&bus->co, nid, index, sub, value, SDO_TIMEOUT_MS));
}

static meow_status_t sdo_i32(meow_bus_t *bus, uint8_t nid, uint16_t index,
                             uint8_t sub, int32_t value)
{
    return map_co(canopen_download_i32(&bus->co, nid, index, sub, value, SDO_TIMEOUT_MS));
}

static meow_status_t sdo_f32(meow_bus_t *bus, uint8_t nid, uint16_t index,
                             uint8_t sub, float value)
{
    return map_co(canopen_download_f32(&bus->co, nid, index, sub, value, SDO_TIMEOUT_MS));
}

static meow_status_t up_u8(meow_bus_t *bus, uint8_t nid, uint16_t index,
                           uint8_t sub, uint8_t *value)
{
    return map_co(canopen_upload_u8(&bus->co, nid, index, sub, value, SDO_TIMEOUT_MS));
}

static meow_status_t up_u16(meow_bus_t *bus, uint8_t nid, uint16_t index,
                            uint8_t sub, uint16_t *value)
{
    return map_co(canopen_upload_u16(&bus->co, nid, index, sub, value, SDO_TIMEOUT_MS));
}

static meow_status_t up_u32(meow_bus_t *bus, uint8_t nid, uint16_t index,
                            uint8_t sub, uint32_t *value)
{
    return map_co(canopen_upload_u32(&bus->co, nid, index, sub, value, SDO_TIMEOUT_MS));
}

static meow_status_t up_i32(meow_bus_t *bus, uint8_t nid, uint16_t index,
                            uint8_t sub, int32_t *value)
{
    return map_co(canopen_upload_i32(&bus->co, nid, index, sub, value, SDO_TIMEOUT_MS));
}

static meow_status_t up_f32(meow_bus_t *bus, uint8_t nid, uint16_t index,
                            uint8_t sub, float *value)
{
    return map_co(canopen_upload_f32(&bus->co, nid, index, sub, value, SDO_TIMEOUT_MS));
}

static void sdo_gap(void)
{
    board_delay_ms(SDO_GAP_MS);
}

static uint32_t pdo_pack(const pdo_entry_t *entry)
{
    return ((uint32_t)entry->index << 16) |
           ((uint32_t)entry->subindex << 8) |
           (uint32_t)entry->bits;
}

static meow_status_t write_mit_range_internal(meow_bus_t *bus,
                                              meow_motor_t *motor,
                                              const meow_mit_range_t *range)
{
    meow_status_t st;

    st = sdo_f32(bus, motor->node_id, OD_MIT_COMPRESSED, MITC_POS_MIN, range->position_min_rev);
    if (st != MEOW_OK) return st;
    sdo_gap();
    st = sdo_f32(bus, motor->node_id, OD_MIT_COMPRESSED, MITC_POS_MAX, range->position_max_rev);
    if (st != MEOW_OK) return st;
    sdo_gap();
    st = sdo_f32(bus, motor->node_id, OD_MIT_COMPRESSED, MITC_VEL_MIN, range->velocity_min_rev_s);
    if (st != MEOW_OK) return st;
    sdo_gap();
    st = sdo_f32(bus, motor->node_id, OD_MIT_COMPRESSED, MITC_VEL_MAX, range->velocity_max_rev_s);
    if (st != MEOW_OK) return st;
    sdo_gap();
    st = sdo_f32(bus, motor->node_id, OD_MIT_COMPRESSED, MITC_KP_MIN, range->kp_min_nm_rev);
    if (st != MEOW_OK) return st;
    sdo_gap();
    st = sdo_f32(bus, motor->node_id, OD_MIT_COMPRESSED, MITC_KP_MAX, range->kp_max_nm_rev);
    if (st != MEOW_OK) return st;
    sdo_gap();
    st = sdo_f32(bus, motor->node_id, OD_MIT_COMPRESSED, MITC_KD_MIN, range->kd_min_nm_s_rev);
    if (st != MEOW_OK) return st;
    sdo_gap();
    st = sdo_f32(bus, motor->node_id, OD_MIT_COMPRESSED, MITC_KD_MAX, range->kd_max_nm_s_rev);
    if (st != MEOW_OK) return st;
    sdo_gap();
    st = sdo_f32(bus, motor->node_id, OD_MIT_COMPRESSED, MITC_TOR_MIN, range->torque_min_nm);
    if (st != MEOW_OK) return st;
    sdo_gap();
    st = sdo_f32(bus, motor->node_id, OD_MIT_COMPRESSED, MITC_TOR_MAX, range->torque_max_nm);
    if (st != MEOW_OK) return st;
    sdo_gap();

    /* Safe zero target, then enable the compressed-MIT selector. */
    st = sdo_u32(bus, motor->node_id, OD_MIT_COMPRESSED, MITC_LOWER, 0U);
    if (st != MEOW_OK) return st;
    sdo_gap();
    st = sdo_u32(bus, motor->node_id, OD_MIT_COMPRESSED, MITC_UPPER, 0U);
    if (st != MEOW_OK) return st;
    sdo_gap();
    st = sdo_u8(bus, motor->node_id, OD_MIT_COMPRESSED, MITC_SELECTOR, 1U);
    if (st != MEOW_OK) return st;
    sdo_gap();

    motor->mit_range = *range;
    return MEOW_OK;
}

static meow_status_t configure_tpdo(meow_bus_t *bus, meow_motor_t *motor,
                                    uint8_t tpdo_index, uint16_t cob_id,
                                    const pdo_entry_t *entries, uint8_t count,
                                    uint16_t inhibit_100us, uint16_t event_timer_ms)
{
    uint16_t comm = (uint16_t)(0x1800U + tpdo_index);
    uint16_t map = (uint16_t)(0x1A00U + tpdo_index);
    meow_status_t st;
    uint8_t i;

    st = sdo_u32(bus, motor->node_id, comm, 1U, 0xC0000000UL | cob_id);
    if (st != MEOW_OK) return st;
    sdo_gap();
    st = sdo_u8(bus, motor->node_id, map, 0U, 0U);
    if (st != MEOW_OK) return st;
    sdo_gap();
    for (i = 0U; i < count; i++) {
        st = sdo_u32(bus, motor->node_id, map, (uint8_t)(i + 1U), pdo_pack(&entries[i]));
        if (st != MEOW_OK) return st;
        sdo_gap();
    }
    st = sdo_u8(bus, motor->node_id, map, 0U, count);
    if (st != MEOW_OK) return st;
    sdo_gap();
    st = sdo_u8(bus, motor->node_id, comm, 2U, 255U);
    if (st != MEOW_OK) return st;
    sdo_gap();
    st = sdo_u16(bus, motor->node_id, comm, 3U, inhibit_100us);
    if (st != MEOW_OK) return st;
    sdo_gap();
    st = sdo_u16(bus, motor->node_id, comm, 5U, event_timer_ms);
    if (st != MEOW_OK) return st;
    sdo_gap();
    return sdo_u32(bus, motor->node_id, comm, 1U, 0x40000000UL | cob_id);
}

static meow_status_t configure_rpdo(meow_bus_t *bus, meow_motor_t *motor,
                                    uint8_t rpdo_index, uint16_t cob_id,
                                    const pdo_entry_t *entries, uint8_t count)
{
    uint16_t comm = (uint16_t)(0x1400U + rpdo_index);
    uint16_t map = (uint16_t)(0x1600U + rpdo_index);
    meow_status_t st;
    uint8_t i;

    st = sdo_u32(bus, motor->node_id, comm, 1U, 0x80000000UL | cob_id);
    if (st != MEOW_OK) return st;
    sdo_gap();
    st = sdo_u8(bus, motor->node_id, comm, 2U, 255U);
    if (st != MEOW_OK) return st;
    sdo_gap();
    st = sdo_u8(bus, motor->node_id, map, 0U, 0U);
    if (st != MEOW_OK) return st;
    sdo_gap();
    for (i = 0U; i < count; i++) {
        st = sdo_u32(bus, motor->node_id, map, (uint8_t)(i + 1U), pdo_pack(&entries[i]));
        if (st != MEOW_OK) return st;
        sdo_gap();
    }
    st = sdo_u8(bus, motor->node_id, map, 0U, count);
    if (st != MEOW_OK) return st;
    sdo_gap();
    return sdo_u32(bus, motor->node_id, comm, 1U, cob_id);
}

static meow_status_t configure_default_tpdos(meow_bus_t *bus, meow_motor_t *motor)
{
    static const pdo_entry_t tpdo1[] = {
        {0x4564U, 0U, 32U}, /* actual position, Q8.24 */
        {0x4713U, 0U, 16U}, /* timestamp */
        {0x4577U, 0U, 16U}, /* actual torque, permille */
    };
    static const pdo_entry_t tpdo2[] = {
        {0x4402U, 0U, 8U},  /* mode display */
        {0x453FU, 0U, 16U}, /* detailed error */
        {0x4104U, 1U, 16U}, /* driver temperature */
        {0x4104U, 2U, 16U}, /* motor temperature */
    };
    meow_status_t st;

    st = configure_tpdo(bus, motor, 0U,
                        (uint16_t)(0x180U + motor->node_id),
                        tpdo1, 3U, 5U, 1U);
    if (st != MEOW_OK) return st;
    return configure_tpdo(bus, motor, 1U,
                          (uint16_t)(0x280U + motor->node_id),
                          tpdo2, 4U, 190U, 20U);
}

static meow_status_t configure_compressed_rpdos(meow_bus_t *bus, meow_motor_t *motor)
{
    pdo_entry_t rpdo1[MEOW_MAX_MOTORS * 2U];
    pdo_entry_t rpdo2[MEOW_MAX_MOTORS];
    uint8_t total = motor->total_motors;
    uint8_t slot;
    uint8_t n1 = 0U;
    uint8_t n2 = 0U;

    if ((total == 0U) || (total > MEOW_MAX_MOTORS) || (motor->slot >= total)) {
        return MEOW_ERR_NOT_CONFIGURED;
    }

    for (slot = 0U; slot < total; slot++) {
        if (slot == motor->slot) {
            rpdo1[n1].index = OD_MIT_COMPRESSED;
            rpdo1[n1].subindex = MITC_LOWER;
            rpdo1[n1].bits = 32U;
            n1++;
            rpdo1[n1].index = OD_MIT_COMPRESSED;
            rpdo1[n1].subindex = MITC_UPPER;
            rpdo1[n1].bits = 32U;
            n1++;

            rpdo2[n2].index = OD_MODE_CMD;
            rpdo2[n2].subindex = 0U;
            rpdo2[n2].bits = 8U;
            n2++;
        } else {
            rpdo1[n1].index = 0x3000U;
            rpdo1[n1].subindex = 3U;
            rpdo1[n1].bits = 32U;
            n1++;
            rpdo1[n1].index = 0x3000U;
            rpdo1[n1].subindex = 3U;
            rpdo1[n1].bits = 32U;
            n1++;

            rpdo2[n2].index = 0x3000U;
            rpdo2[n2].subindex = 1U;
            rpdo2[n2].bits = 8U;
            n2++;
        }
    }

    {
        meow_status_t st = configure_rpdo(bus, motor, 0U, bus->rpdo1_cob_id, rpdo1, n1);
        if (st != MEOW_OK) return st;
        return configure_rpdo(bus, motor, 1U, bus->rpdo2_cob_id, rpdo2, n2);
    }
}

static meow_status_t wait_operational(meow_bus_t *bus, meow_motor_t *motor)
{
    uint32_t start = board_millis();

    while ((board_millis() - start) < NMT_WAIT_MS) {
        if (bus->co.service_cb != 0) {
            bus->co.service_cb(bus->co.service_ctx);
        }
        meow_bus_poll(bus);
        if (motor->nmt_state == (uint8_t)CO_NMT_OPERATIONAL) {
            return MEOW_OK;
        }
        board_delay_ms(1U);
    }
    return MEOW_ERR_TIMEOUT;
}

meow_status_t meow_motor_probe(meow_bus_t *bus, meow_motor_t *motor)
{
    uint32_t vendor = 0U;
    uint32_t product = 0U;
    uint32_t revision = 0U;
    uint32_t serial = 0U;
    uint8_t mode_display = 0U;
    uint16_t error_code = 0U;
    uint16_t limit_permille = 0U;
    meow_status_t st;

    if ((bus == 0) || (motor == 0) || (motor->node_id == 0U)) {
        return MEOW_ERR_INVALID_ARG;
    }

    st = up_u32(bus, motor->node_id, OD_IDENTITY, 1U, &vendor);
    if (st != MEOW_OK) return st;
    st = up_u32(bus, motor->node_id, OD_IDENTITY, 2U, &product);
    if (st != MEOW_OK) return st;
    st = up_u32(bus, motor->node_id, OD_IDENTITY, 3U, &revision);
    if (st != MEOW_OK) return st;
    st = up_u32(bus, motor->node_id, OD_IDENTITY, 4U, &serial);
    if (st != MEOW_OK) return st;

    motor->vendor_id = vendor;
    motor->product_code = product;
    motor->revision = revision;
    motor->serial = serial;

    /*
     * Detect two devices answering on the same node ID. Their SDO responses
     * share COB-ID 0x580+nid, so a second identity read may return values from
     * the other device. Refuse to configure such a bus.
     */
    {
        uint32_t vendor2 = 0U;
        uint32_t product2 = 0U;
        uint32_t revision2 = 0U;
        uint32_t serial2 = 0U;
        if ((up_u32(bus, motor->node_id, OD_IDENTITY, 1U, &vendor2) != MEOW_OK) ||
            (up_u32(bus, motor->node_id, OD_IDENTITY, 2U, &product2) != MEOW_OK) ||
            (up_u32(bus, motor->node_id, OD_IDENTITY, 3U, &revision2) != MEOW_OK) ||
            (up_u32(bus, motor->node_id, OD_IDENTITY, 4U, &serial2) != MEOW_OK) ||
            (vendor2 != vendor) || (product2 != product) ||
            (revision2 != revision) || (serial2 != serial)) {
            motor->identified = false;
            motor->supported = false;
            return MEOW_ERR_IDENTITY_CONFLICT;
        }
    }

    motor->identified = true;

    if (vendor != MEOW_VENDOR_ID) {
        motor->supported = false;
        return MEOW_ERR_NOT_SUPPORTED;
    }

    if (product == MEOW_PRODUCT_4310) {
        motor->model = MEOW_MODEL_4310;
    } else if (product == MEOW_PRODUCT_4342) {
        motor->model = MEOW_MODEL_4342;
    } else if (product == MEOW_PRODUCT_3525) {
        motor->model = MEOW_MODEL_3525;
    } else {
        uint8_t md = 0U;
        uint16_t err = 0U;
        if ((up_u8(bus, motor->node_id, OD_MODE_DISPLAY, 0U, &md) == MEOW_OK) &&
            (up_u16(bus, motor->node_id, OD_DETAILED_ERROR, 0U, &err) == MEOW_OK)) {
            motor->model = MEOW_MODEL_GENERIC;
        } else {
            motor->supported = false;
            return MEOW_ERR_NOT_SUPPORTED;
        }
    }

    motor->supported = true;
    motor->heartbeat_period_ms = 1000U;

    (void)up_f32(bus, motor->node_id, OD_PEAK_TORQUE, 0U, &motor->peak_torque_nm);
    (void)up_f32(bus, motor->node_id, OD_MIT_UNCOMPRESSED, 7U, &motor->mit_factor);
    (void)up_f32(bus, motor->node_id, OD_MIT_COMPRESSED, MITC_TOR_LIMIT,
                 &motor->compressed_torque_limit_nm);
    (void)up_u16(bus, motor->node_id, OD_HEARTBEAT_PRODUCER, 0U,
                 &motor->heartbeat_period_ms);
    (void)up_u8(bus, motor->node_id, OD_ERROR_REGISTER, 0U,
                &motor->state.error_register);
    (void)up_u8(bus, motor->node_id, OD_MODE_DISPLAY, 0U, &mode_display);
    (void)up_u16(bus, motor->node_id, OD_DETAILED_ERROR, 0U, &error_code);
    (void)up_u16(bus, motor->node_id, OD_MIT_UNCOMPRESSED, 6U, &limit_permille);
    (void)up_f32(bus, motor->node_id, OD_PROFILE_VELOCITY, 0U,
                 &motor->profile_limits.velocity_limit_rev_s);
    (void)up_f32(bus, motor->node_id, OD_PROFILE_ACCELERATION, 0U,
                 &motor->profile_limits.acceleration_rev_s2);
    (void)up_f32(bus, motor->node_id, OD_PROFILE_DECELERATION, 0U,
                 &motor->profile_limits.deceleration_rev_s2);

    motor->state.mode_display = mode_display;
    motor->state.error_code = error_code;
    motor->state.fault = (error_code != 0U);
    motor->fault = motor->state.fault;
    motor->mit_torque_limit_nm = (motor->peak_torque_nm > 0.0f)
                                     ? ((float)limit_permille * motor->peak_torque_nm / 1000.0f)
                                     : 0.0f;

    {
        int32_t raw = 0;
        if (up_i32(bus, motor->node_id, OD_POSITION_ACTUAL, 0U, &raw) == MEOW_OK) {
            motor->last_position_raw = raw;
            motor->state.position_rev = meow_q24_to_float(raw);
        }
    }

    return MEOW_OK;
}

meow_status_t meow_motor_init(meow_bus_t *bus,
                              meow_motor_t *motor,
                              const meow_mit_range_t *range)
{
    meow_mit_range_t default_range;
    uint32_t consumer;
    meow_status_t st;

    if ((bus == 0) || (motor == 0) || (motor->node_id == 0U) ||
        (motor->total_motors == 0U) || (motor->slot >= motor->total_motors)) {
        return MEOW_ERR_INVALID_ARG;
    }

    if (range == 0) {
        meow_range_default(&default_range);
        range = &default_range;
    }
    if (!meow_range_validate(range)) {
        return MEOW_ERR_INVALID_ARG;
    }

    motor->ready = false;
    motor->enabled = false;
    motor->fault = false;
    motor->compressed_mit_enabled = false;
    motor->mode = MEOW_MODE_DISABLED;
    motor->cmd_position_rev = 0.0f;
    motor->cmd_velocity_rev_s = 0.0f;
    motor->cmd_kp_nm_rev = 0.0f;
    motor->cmd_kd_nm_s_rev = 0.0f;
    motor->cmd_torque_nm = 0.0f;
    motor->cmd_valid = false;

    st = map_co(canopen_nmt(&bus->co, CO_NMT_CMD_PREOP, motor->node_id));
    if (st != MEOW_OK) return st;
    board_delay_ms(20U);

    /* Clear a latched fault, then leave the motor disabled. */
    st = sdo_u8(bus, motor->node_id, OD_MODE_CMD, 0U, 0xFFU);
    if (st != MEOW_OK) return st;
    board_delay_ms(20U);
    st = sdo_u8(bus, motor->node_id, OD_MODE_CMD, 0U, 0U);
    if (st != MEOW_OK) return st;

    if (!motor->identified) {
        st = meow_motor_probe(bus, motor);
        if (st != MEOW_OK) return st;
    }

    st = write_mit_range_internal(bus, motor, range);
    if (st != MEOW_OK) return st;
    motor->compressed_mit_enabled = true;

    consumer = ((uint32_t)bus->host_node_id << 16) | HEARTBEAT_TIMEOUT_MS;
    st = sdo_u32(bus, motor->node_id, OD_HEARTBEAT_CONSUMER, 1U, consumer);
    if (st != MEOW_OK) return st;

    st = configure_default_tpdos(bus, motor);
    if (st != MEOW_OK) return st;

    st = configure_compressed_rpdos(bus, motor);
    if (st != MEOW_OK) return st;

    st = map_co(canopen_nmt(&bus->co, CO_NMT_CMD_START, motor->node_id));
    if (st != MEOW_OK) return st;

    st = wait_operational(bus, motor);
    if (st != MEOW_OK) return st;

    /* The motor may have entered Operational with the previous mode latched. */
    st = sdo_u8(bus, motor->node_id, OD_MODE_CMD, 0U, 0U);
    if (st != MEOW_OK) return st;

    st = meow_motor_read_status(bus, motor);
    if (st != MEOW_OK) return st;
    if (motor->state.error_code != 0U) {
        motor->ready = false;
        motor->fault = true;
        return MEOW_ERR_FAULT;
    }

    motor->ready = true;
    motor->enabled = false;
    motor->fault = false;
    motor->mode = MEOW_MODE_DISABLED;
    motor->mode_dirty = true;
    motor->state.enabled = false;
    motor->state.fault = false;
    motor->state.online = true;
    motor->state.nmt_state = (uint8_t)CO_NMT_OPERATIONAL;
    motor->nmt_state = (uint8_t)CO_NMT_OPERATIONAL;
    return MEOW_OK;
}

meow_status_t meow_motor_set_mit_range(meow_bus_t *bus,
                                       meow_motor_t *motor,
                                       const meow_mit_range_t *range)
{
    if ((bus == 0) || (motor == 0) || !motor->ready) {
        return MEOW_ERR_NOT_READY;
    }
    if (motor->enabled) {
        return MEOW_ERR_NOT_READY;
    }
    if (!meow_range_validate(range)) {
        return MEOW_ERR_INVALID_ARG;
    }
    return write_mit_range_internal(bus, motor, range);
}

meow_status_t meow_motor_get_mit_range(meow_bus_t *bus,
                                       meow_motor_t *motor,
                                       meow_mit_range_t *range)
{
    meow_status_t st;

    if ((bus == 0) || (motor == 0) || (range == 0)) {
        return MEOW_ERR_INVALID_ARG;
    }

    st = up_f32(bus, motor->node_id, OD_MIT_COMPRESSED, MITC_POS_MIN, &range->position_min_rev);
    if (st != MEOW_OK) return st;
    st = up_f32(bus, motor->node_id, OD_MIT_COMPRESSED, MITC_POS_MAX, &range->position_max_rev);
    if (st != MEOW_OK) return st;
    st = up_f32(bus, motor->node_id, OD_MIT_COMPRESSED, MITC_VEL_MIN, &range->velocity_min_rev_s);
    if (st != MEOW_OK) return st;
    st = up_f32(bus, motor->node_id, OD_MIT_COMPRESSED, MITC_VEL_MAX, &range->velocity_max_rev_s);
    if (st != MEOW_OK) return st;
    st = up_f32(bus, motor->node_id, OD_MIT_COMPRESSED, MITC_KP_MIN, &range->kp_min_nm_rev);
    if (st != MEOW_OK) return st;
    st = up_f32(bus, motor->node_id, OD_MIT_COMPRESSED, MITC_KP_MAX, &range->kp_max_nm_rev);
    if (st != MEOW_OK) return st;
    st = up_f32(bus, motor->node_id, OD_MIT_COMPRESSED, MITC_KD_MIN, &range->kd_min_nm_s_rev);
    if (st != MEOW_OK) return st;
    st = up_f32(bus, motor->node_id, OD_MIT_COMPRESSED, MITC_KD_MAX, &range->kd_max_nm_s_rev);
    if (st != MEOW_OK) return st;
    st = up_f32(bus, motor->node_id, OD_MIT_COMPRESSED, MITC_TOR_MIN, &range->torque_min_nm);
    if (st != MEOW_OK) return st;
    st = up_f32(bus, motor->node_id, OD_MIT_COMPRESSED, MITC_TOR_MAX, &range->torque_max_nm);
    if (st != MEOW_OK) return st;

    motor->mit_range = *range;
    return MEOW_OK;
}

meow_status_t meow_motor_set_torque_limit(meow_bus_t *bus,
                                          meow_motor_t *motor,
                                          float torque_limit_nm)
{
    float limit = fabsf(torque_limit_nm);
    meow_status_t st;

    if ((bus == 0) || (motor == 0) || !motor->ready) {
        return MEOW_ERR_NOT_READY;
    }
    if (motor->enabled || (limit <= 0.0f) || !isfinite(limit)) {
        return MEOW_ERR_INVALID_ARG;
    }
    if ((motor->peak_torque_nm > 0.0f) && (limit > motor->peak_torque_nm)) {
        limit = motor->peak_torque_nm;
    }

    st = sdo_f32(bus, motor->node_id, OD_MIT_COMPRESSED, MITC_TOR_MIN, -limit);
    if (st != MEOW_OK) return st;
    sdo_gap();
    st = sdo_f32(bus, motor->node_id, OD_MIT_COMPRESSED, MITC_TOR_MAX, limit);
    if (st != MEOW_OK) return st;
    sdo_gap();

    motor->mit_range.torque_min_nm = -limit;
    motor->mit_range.torque_max_nm = limit;
    motor->mit_torque_limit_nm = limit;

    /* The compressed path is clamped by 0x4103:0C/0D. 0x4102:06 belongs to
     * the uncompressed-MIT path and is intentionally not written here. */
    return MEOW_OK;
}

meow_status_t meow_motor_get_torque_limit(meow_bus_t *bus,
                                          meow_motor_t *motor,
                                          float *torque_limit_nm)
{
    float tor_min = 0.0f;
    float tor_max = 0.0f;
    uint16_t permille = 0U;

    if ((bus == 0) || (motor == 0) || (torque_limit_nm == 0)) {
        return MEOW_ERR_INVALID_ARG;
    }

    if (up_f32(bus, motor->node_id, OD_MIT_COMPRESSED, MITC_TOR_MIN, &tor_min) == MEOW_OK &&
        up_f32(bus, motor->node_id, OD_MIT_COMPRESSED, MITC_TOR_MAX, &tor_max) == MEOW_OK) {
        motor->mit_range.torque_min_nm = tor_min;
        motor->mit_range.torque_max_nm = tor_max;
        *torque_limit_nm = fmaxf(fabsf(tor_min), fabsf(tor_max));
    } else if (motor->peak_torque_nm > 0.0f &&
               up_u16(bus, motor->node_id, OD_MIT_UNCOMPRESSED, 6U, &permille) == MEOW_OK) {
        *torque_limit_nm = (float)permille * motor->peak_torque_nm / 1000.0f;
    } else {
        *torque_limit_nm = motor->mit_torque_limit_nm;
    }

    motor->mit_torque_limit_nm = *torque_limit_nm;
    return MEOW_OK;
}

meow_status_t meow_motor_set_profile_limits(meow_bus_t *bus,
                                            meow_motor_t *motor,
                                            const meow_profile_limits_t *limits)
{
    meow_status_t st;

    if ((bus == 0) || (motor == 0) || (limits == 0) || !motor->ready) {
        return MEOW_ERR_NOT_READY;
    }
    if (!isfinite(limits->velocity_limit_rev_s) ||
        !isfinite(limits->acceleration_rev_s2) ||
        !isfinite(limits->deceleration_rev_s2) ||
        (limits->velocity_limit_rev_s <= 0.0f) ||
        (limits->acceleration_rev_s2 <= 0.0f) ||
        (limits->deceleration_rev_s2 <= 0.0f)) {
        return MEOW_ERR_INVALID_ARG;
    }

    st = sdo_f32(bus, motor->node_id, OD_PROFILE_VELOCITY, 0U, limits->velocity_limit_rev_s);
    if (st != MEOW_OK) return st;
    sdo_gap();
    st = sdo_f32(bus, motor->node_id, OD_PROFILE_ACCELERATION, 0U, limits->acceleration_rev_s2);
    if (st != MEOW_OK) return st;
    sdo_gap();
    st = sdo_f32(bus, motor->node_id, OD_PROFILE_DECELERATION, 0U, limits->deceleration_rev_s2);
    if (st != MEOW_OK) return st;
    sdo_gap();

    /* This hybrid firmware also exposes the CiA402 profile objects. Write
     * them best-effort so PP uses the same limits. */
    (void)sdo_f32(bus, motor->node_id, 0x6081U, 0U, limits->velocity_limit_rev_s);
    sdo_gap();
    (void)sdo_f32(bus, motor->node_id, 0x6083U, 0U, limits->acceleration_rev_s2);
    sdo_gap();
    (void)sdo_f32(bus, motor->node_id, 0x6084U, 0U, limits->deceleration_rev_s2);
    sdo_gap();

    motor->profile_limits = *limits;
    return MEOW_OK;
}

meow_status_t meow_motor_get_profile_limits(meow_bus_t *bus,
                                            meow_motor_t *motor,
                                            meow_profile_limits_t *limits)
{
    meow_status_t st;

    if ((bus == 0) || (motor == 0) || (limits == 0)) {
        return MEOW_ERR_INVALID_ARG;
    }

    st = up_f32(bus, motor->node_id, OD_PROFILE_VELOCITY, 0U, &limits->velocity_limit_rev_s);
    if (st != MEOW_OK) return st;
    st = up_f32(bus, motor->node_id, OD_PROFILE_ACCELERATION, 0U, &limits->acceleration_rev_s2);
    if (st != MEOW_OK) return st;
    st = up_f32(bus, motor->node_id, OD_PROFILE_DECELERATION, 0U, &limits->deceleration_rev_s2);
    if (st != MEOW_OK) return st;

    motor->profile_limits = *limits;
    return MEOW_OK;
}

static void stop_mit(meow_bus_t *bus, meow_motor_t *motor)
{
    motor->cmd_position_rev = 0.0f;
    motor->cmd_velocity_rev_s = 0.0f;
    motor->cmd_kp_nm_rev = 0.0f;
    motor->cmd_kd_nm_s_rev = 0.0f;
    motor->cmd_torque_nm = 0.0f;
    motor->cmd_valid = true;
    motor->enabled = false;
    motor->mode = MEOW_MODE_DISABLED;
    motor->mode_dirty = true;
    motor->state.enabled = false;
    meow_bus_sync(bus);
    (void)sdo_u8(bus, motor->node_id, OD_MODE_CMD, 0U, 0U);
}

meow_status_t meow_motor_enable_mit(meow_bus_t *bus, meow_motor_t *motor)
{
    uint32_t start;
    uint32_t last_mode_read = 0U;

    if ((bus == 0) || (motor == 0) || !motor->ready || !motor->compressed_mit_enabled) {
        return MEOW_ERR_NOT_READY;
    }
    if (motor->fault || (motor->state.error_code != 0U)) {
        return MEOW_ERR_FAULT;
    }
    if (motor->enabled && (motor->state.mode_display == MEOW_MODE_MIT)) {
        return MEOW_OK;
    }

    motor->cmd_position_rev = 0.0f;
    motor->cmd_velocity_rev_s = 0.0f;
    motor->cmd_kp_nm_rev = 0.0f;
    motor->cmd_kd_nm_s_rev = 0.0f;
    motor->cmd_torque_nm = 0.0f;
    motor->cmd_valid = true;
    motor->enabled = true;
    motor->mode = MEOW_MODE_MIT;
    motor->mode_dirty = true;

    meow_bus_set_sync(bus, true);
    meow_bus_sync(bus);

    start = board_millis();
    while ((board_millis() - start) < MODE_WAIT_MS) {
        meow_bus_poll(bus);
        meow_bus_sync(bus);

        if (motor->state.error_code != 0U) {
            motor->fault = true;
            stop_mit(bus, motor);
            return MEOW_ERR_FAULT;
        }
        if (motor->state.mode_display == MEOW_MODE_MIT) {
            motor->state.enabled = true;
            return MEOW_OK;
        }

        if ((board_millis() - last_mode_read) >= 20U) {
            uint8_t md = 0U;
            last_mode_read = board_millis();
            if (up_u8(bus, motor->node_id, OD_MODE_DISPLAY, 0U, &md) == MEOW_OK) {
                motor->state.mode_display = md;
                if (md == MEOW_MODE_MIT) {
                    motor->state.enabled = true;
                    return MEOW_OK;
                }
            }
        }

        board_delay_ms(1U);
    }

    stop_mit(bus, motor);
    return MEOW_ERR_TIMEOUT;
}

meow_status_t meow_motor_disable(meow_bus_t *bus, meow_motor_t *motor)
{
    if ((bus == 0) || (motor == 0)) {
        return MEOW_ERR_INVALID_ARG;
    }
    stop_mit(bus, motor);
    return MEOW_OK;
}

meow_status_t meow_motor_clear_error(meow_bus_t *bus, meow_motor_t *motor)
{
    meow_status_t st;

    if ((bus == 0) || (motor == 0)) {
        return MEOW_ERR_INVALID_ARG;
    }

    if (motor->enabled && (motor->mode == MEOW_MODE_MIT)) {
        motor->cmd_position_rev = 0.0f;
        motor->cmd_velocity_rev_s = 0.0f;
        motor->cmd_kp_nm_rev = 0.0f;
        motor->cmd_kd_nm_s_rev = 0.0f;
        motor->cmd_torque_nm = 0.0f;
        motor->cmd_valid = true;
        meow_bus_sync(bus);
    }

    st = sdo_u8(bus, motor->node_id, OD_MODE_CMD, 0U, 0xFFU);
    if (st != MEOW_OK) return st;
    board_delay_ms(20U);
    st = sdo_u8(bus, motor->node_id, OD_MODE_CMD, 0U, 0U);
    if (st != MEOW_OK) return st;

    motor->fault = false;
    motor->error_count = 0U;
    motor->state.error_code = 0U;
    motor->state.fault = false;
    motor->state.enabled = false;
    motor->enabled = false;
    motor->mode = MEOW_MODE_DISABLED;
    motor->mode_dirty = true;
    meow_bus_sync(bus);
    return MEOW_OK;
}

static meow_status_t cia402_enable(meow_bus_t *bus, meow_motor_t *motor)
{
    uint32_t start = board_millis();
    uint32_t last_read = 0U;
    uint16_t status = 0U;

    (void)sdo_u16(bus, motor->node_id, 0x6040U, 0U, 0x0006U);
    board_delay_ms(10U);
    (void)sdo_u16(bus, motor->node_id, 0x6040U, 0U, 0x0007U);
    board_delay_ms(10U);
    (void)sdo_u16(bus, motor->node_id, 0x6040U, 0U, 0x000FU);
    board_delay_ms(10U);

    while ((board_millis() - start) < 300U) {
        meow_bus_poll(bus);
        meow_bus_sync(bus);
        if ((board_millis() - last_read) >= 10U) {
            last_read = board_millis();
            if (canopen_upload_u16(&bus->co, motor->node_id, 0x6041U, 0U,
                                   &status, SDO_TIMEOUT_MS) == CO_OK) {
                if ((status & 0x006FU) == 0x0027U) {
                    return MEOW_OK;
                }
            }
        }
        board_delay_ms(1U);
    }
    return MEOW_ERR_TIMEOUT;
}

static meow_status_t wait_for_mode(meow_bus_t *bus, meow_motor_t *motor, uint8_t mode)
{
    uint32_t start = board_millis();
    uint32_t last_read = 0U;

    while ((board_millis() - start) < 300U) {
        meow_bus_poll(bus);
        meow_bus_sync(bus);

        if (motor->state.error_code != 0U) {
            return MEOW_ERR_FAULT;
        }
        if (motor->state.mode_display == mode) {
            return MEOW_OK;
        }
        if ((board_millis() - last_read) >= 20U) {
            uint8_t md = 0U;
            last_read = board_millis();
            if (up_u8(bus, motor->node_id, OD_MODE_DISPLAY, 0U, &md) == MEOW_OK) {
                motor->state.mode_display = md;
                if (md == mode) {
                    return MEOW_OK;
                }
            }
        }
        board_delay_ms(1U);
    }
    return MEOW_ERR_TIMEOUT;
}

static meow_status_t set_mode_sdo(meow_bus_t *bus, meow_motor_t *motor, uint8_t mode)
{
    meow_status_t st;

    switch (mode) {
    case MEOW_MODE_PROFILE_POSITION:
        st = sdo_i32(bus, motor->node_id, OD_TARGET_POSITION, 0U, 0);
        break;
    case MEOW_MODE_PROFILE_VELOCITY:
        st = sdo_f32(bus, motor->node_id, OD_TARGET_VELOCITY, 0U, 0.0f);
        break;
    case MEOW_MODE_TORQUE:
        st = sdo_i16(bus, motor->node_id, OD_TARGET_TORQUE, 0U, 0);
        break;
    default:
        return MEOW_ERR_INVALID_ARG;
    }
    if (st != MEOW_OK) return st;
    sdo_gap();

    (void)sdo_u16(bus, motor->node_id, 0x6040U, 0U, 0x0006U);
    sdo_gap();
    st = sdo_u8(bus, motor->node_id, OD_MODE_CMD, 0U, mode);
    if (st != MEOW_OK) return st;
    sdo_gap();

    motor->mode = mode;
    motor->enabled = true;
    motor->mode_dirty = true;
    motor->cmd_valid = false;
    return wait_for_mode(bus, motor, mode);
}

meow_status_t meow_motor_set_velocity(meow_bus_t *bus,
                                      meow_motor_t *motor,
                                      float rev_per_s)
{
    meow_status_t st;

    if ((bus == 0) || (motor == 0) || !motor->ready || motor->fault) {
        return MEOW_ERR_NOT_READY;
    }
    if (motor->mode != MEOW_MODE_PROFILE_VELOCITY) {
        st = set_mode_sdo(bus, motor, MEOW_MODE_PROFILE_VELOCITY);
        if (st != MEOW_OK) return st;
    }
    return sdo_f32(bus, motor->node_id, OD_TARGET_VELOCITY, 0U, rev_per_s);
}

meow_status_t meow_motor_set_position(meow_bus_t *bus,
                                      meow_motor_t *motor,
                                      float rev)
{
    meow_status_t st;

    if ((bus == 0) || (motor == 0) || !motor->ready || motor->fault) {
        return MEOW_ERR_NOT_READY;
    }
    /*
     * This Meow firmware implements Profile Position as a hybrid:
     *   0x4401 = 1 (Meow mode), 0x6060 = 1 (CiA402 mode),
     *   CiA402 controlword enable ramp, then both 0x457A (Q8.24) and
     *   0x607A (f32) targets, followed by the new-setpoint edge 0x2F -> 0x3F.
     */
    if ((motor->mode != MEOW_MODE_PROFILE_POSITION) || !motor->enabled) {
        st = sdo_u8(bus, motor->node_id, OD_MODE_CMD, 0U,
                    MEOW_MODE_PROFILE_POSITION);
        if (st != MEOW_OK) return st;
        sdo_gap();
        st = sdo_u8(bus, motor->node_id, 0x6060U, 0U, 1U);
        if (st != MEOW_OK) return st;
        sdo_gap();
        motor->mode = MEOW_MODE_PROFILE_POSITION;
        motor->enabled = true;
        motor->mode_dirty = true;
        motor->cmd_valid = false;
        st = cia402_enable(bus, motor);
        if (st != MEOW_OK) return st;
    }

    st = sdo_i32(bus, motor->node_id, OD_TARGET_POSITION, 0U,
                 meow_float_to_q24(rev));
    if (st != MEOW_OK) return st;
    sdo_gap();
    st = sdo_f32(bus, motor->node_id, 0x607AU, 0U, rev);
    if (st != MEOW_OK) return st;
    sdo_gap();
    st = sdo_u16(bus, motor->node_id, 0x6040U, 0U, 0x002FU);
    if (st != MEOW_OK) return st;
    sdo_gap();
    return sdo_u16(bus, motor->node_id, 0x6040U, 0U, 0x003FU);
}

meow_status_t meow_motor_set_torque(meow_bus_t *bus,
                                    meow_motor_t *motor,
                                    float torque_nm)
{
    float permille_f;
    int16_t permille;

    if ((bus == 0) || (motor == 0) || !motor->ready || motor->fault) {
        return MEOW_ERR_NOT_READY;
    }
    if (motor->peak_torque_nm <= 0.0f) {
        return MEOW_ERR_NOT_SUPPORTED;
    }
    if (motor->mode != MEOW_MODE_TORQUE) {
        meow_status_t st = set_mode_sdo(bus, motor, MEOW_MODE_TORQUE);
        if (st != MEOW_OK) return st;
    }

    permille_f = torque_nm / motor->peak_torque_nm * 1000.0f;
    if (permille_f > 1000.0f) permille_f = 1000.0f;
    if (permille_f < -1000.0f) permille_f = -1000.0f;
    permille = (int16_t)((permille_f >= 0.0f) ? (permille_f + 0.5f) : (permille_f - 0.5f));
    return sdo_i16(bus, motor->node_id, OD_TARGET_TORQUE, 0U, permille);
}

meow_status_t meow_motor_mit(meow_motor_t *motor,
                             float position_rev,
                             float velocity_rev_s,
                             float kp_nm_rev,
                             float kd_nm_s_rev,
                             float torque_nm)
{
    if ((motor == 0) || !motor->ready || !motor->enabled ||
        (motor->mode != MEOW_MODE_MIT)) {
        return MEOW_ERR_NOT_READY;
    }
    if (motor->fault || (motor->state.error_code != 0U)) {
        return MEOW_ERR_FAULT;
    }

    motor->cmd_position_rev = position_rev;
    motor->cmd_velocity_rev_s = velocity_rev_s;
    motor->cmd_kp_nm_rev = kp_nm_rev;
    motor->cmd_kd_nm_s_rev = kd_nm_s_rev;
    motor->cmd_torque_nm = torque_nm;
    motor->cmd_valid = true;
    return MEOW_OK;
}

meow_status_t meow_motor_read_status(meow_bus_t *bus, meow_motor_t *motor)
{
    meow_status_t first_error = MEOW_OK;
    meow_status_t st;
    uint8_t mode_display = 0U;
    uint8_t error_register = 0U;
    uint16_t error_code = 0U;
    int32_t position_raw = 0;
    int16_t torque_permille = 0;

    if ((bus == 0) || (motor == 0)) {
        return MEOW_ERR_INVALID_ARG;
    }

    st = up_u8(bus, motor->node_id, OD_MODE_DISPLAY, 0U, &mode_display);
    if (st == MEOW_OK) {
        motor->state.mode_display = mode_display;
        motor->state.enabled = (mode_display >= MEOW_MODE_PROFILE_POSITION) &&
                               (mode_display <= MEOW_MODE_MIT);
    } else if (first_error == MEOW_OK) {
        first_error = st;
    }

    st = up_u16(bus, motor->node_id, OD_DETAILED_ERROR, 0U, &error_code);
    if (st == MEOW_OK) {
        motor->state.error_code = error_code;
        motor->state.fault = (error_code != 0U);
        if (motor->state.fault) {
            motor->fault = true;
            motor->error_count++;
        }
    } else if (first_error == MEOW_OK) {
        first_error = st;
    }

    st = up_u8(bus, motor->node_id, OD_ERROR_REGISTER, 0U, &error_register);
    if (st == MEOW_OK) {
        motor->state.error_register = error_register;
    }

    st = up_i32(bus, motor->node_id, OD_POSITION_ACTUAL, 0U, &position_raw);
    if (st == MEOW_OK) {
        motor->state.position_rev = meow_q24_to_float(position_raw);
    }

    if (map_co(canopen_upload_i16(&bus->co, motor->node_id, OD_TORQUE_ACTUAL, 0U,
                                   &torque_permille, SDO_TIMEOUT_MS)) == MEOW_OK) {
        motor->state.torque_nm = (motor->peak_torque_nm > 0.0f)
                                     ? ((float)torque_permille * motor->peak_torque_nm / 1000.0f)
                                     : 0.0f;
    }

    if (first_error == MEOW_OK) {
        motor->state.last_update_ms = board_millis();
        motor->last_rx_ms = board_millis();
        motor->state.online = true;
    }
    return first_error;
}

meow_status_t meow_motor_get_state(const meow_motor_t *motor,
                                   meow_motor_state_t *state)
{
    if ((motor == 0) || (state == 0)) {
        return MEOW_ERR_INVALID_ARG;
    }
    *state = motor->state;
    return MEOW_OK;
}

void meow_motor_on_frame(meow_motor_t *motor,
                         uint32_t cob_id,
                         const uint8_t *data,
                         uint8_t len,
                         uint32_t now_ms)
{
    uint32_t function = cob_id & 0x780U;

    if ((motor == 0) || (data == 0)) {
        return;
    }

    motor->last_rx_ms = now_ms;

    if (function == 0x700U) {
        if (len >= 1U) {
            motor->nmt_state = data[0];
            motor->state.nmt_state = data[0];
            motor->last_heartbeat_ms = now_ms;
            motor->state.online = true;
            if (data[0] != (uint8_t)CO_NMT_OPERATIONAL) {
                motor->ready = false;
                motor->enabled = false;
                motor->mode = MEOW_MODE_DISABLED;
                motor->cmd_valid = false;
            }
        }
    } else if (function == 0x180U) {
        meow_decode_tpdo1(motor, data, len, now_ms);
    } else if (function == 0x280U) {
        meow_decode_tpdo2(motor, data, len, now_ms);
    } else if (function == 0x080U) {
        if (len >= 2U) {
            motor->state.error_code = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
            motor->state.fault = (motor->state.error_code != 0U);
            motor->fault = motor->state.fault;
            motor->error_count++;
        }
    }
}
