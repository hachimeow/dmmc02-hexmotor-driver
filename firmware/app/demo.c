#include "demo.h"

#include <stdio.h>
#include <string.h>

#include "board.h"
#include "meow_codec.h"

#ifndef MEOW_DEMO_MIT
#define MEOW_DEMO_MIT 0
#endif

#ifndef MEOW_DEMO_PVPP
#define MEOW_DEMO_PVPP 0
#endif

#ifndef MEOW_DEBUG_OD
#define MEOW_DEBUG_OD 0
#endif

#ifndef MEOW_DEMO_CIA_PP
#define MEOW_DEMO_CIA_PP 0
#endif

#ifndef MEOW_TEST_NODE
#define MEOW_TEST_NODE 2
#endif

#if MEOW_DEBUG_OD

#include "canopen.h"

typedef struct {
    uint16_t index;
    uint8_t sub;
    uint8_t status;
    uint32_t value;
    uint32_t abort_code;
} od_probe_entry_t;

#define OD_PROBE_MAX 64U
od_probe_entry_t g_od_probe[OD_PROBE_MAX];
uint32_t g_od_probe_count;

static void probe_one(meow_bus_t *bus, uint8_t nid, uint16_t index, uint8_t sub)
{
    od_probe_entry_t *e;
    uint8_t buf[4] = {0};
    uint8_t len = 0U;
    co_status_t st;

    if (g_od_probe_count >= OD_PROBE_MAX) {
        return;
    }
    e = &g_od_probe[g_od_probe_count++];
    e->index = index;
    e->sub = sub;
    st = canopen_sdo_upload(&bus->co, nid, index, sub, buf, sizeof(buf), &len, 50U);
    e->status = (uint8_t)st;
    e->value = (uint32_t)buf[0] | ((uint32_t)buf[1] << 8) |
               ((uint32_t)buf[2] << 16) | ((uint32_t)buf[3] << 24);
    e->abort_code = (st == CO_ERR_SDO_ABORT) ? bus->co.sdo.abort_code : 0U;
}

void demo_init(meow_bus_t *bus, meow_motor_t *motor)
{
    static const uint16_t od_list[] = {
        0x4570, 0x4571, 0x4572, 0x4573, 0x4574, 0x4575, 0x4576, 0x4577,
        0x4578, 0x4579, 0x457A, 0x457B, 0x457C, 0x457D, 0x457E, 0x457F,
        0x4580, 0x4581, 0x4582, 0x4583, 0x4584, 0x4585, 0x4586,
        0x4402, 0x4403, 0x4404,
        0x6040, 0x6041, 0x6060, 0x6061, 0x6064, 0x606C,
        0x6071, 0x6072, 0x6076, 0x607A, 0x6081, 0x6083, 0x6084,
        0x6067, 0x6068,
    };
    uint32_t i;

    if (motor->node_id != MEOW_TEST_NODE) {
        return;
    }

    g_od_probe_count = 0U;
    for (i = 0U; i < (sizeof(od_list) / sizeof(od_list[0])); i++) {
        probe_one(bus, motor->node_id, od_list[i], 0U);
    }
    probe_one(bus, motor->node_id, 0x4103U, 0x01U);
    probe_one(bus, motor->node_id, 0x4103U, 0x0EU);
    probe_one(bus, motor->node_id, 0x4104U, 0x01U);
    probe_one(bus, motor->node_id, 0x4104U, 0x02U);
}

void demo_poll(meow_bus_t *bus, meow_motor_t *motor)
{
    (void)bus;
    (void)motor;
}

#elif MEOW_DEMO_CIA_PP

typedef struct {
    uint32_t stage;
    uint32_t error;
    uint32_t sw_enabled;
    uint32_t sw_final;
    uint32_t mode_display;
    uint32_t pp_ok;
    uint32_t elapsed_ms;
    float start_rev;
    float target_rev;
    float end_rev;
} cia_pp_result_t;

cia_pp_result_t g_cia_pp_result;

static float absf_local2(float v)
{
    return (v < 0.0f) ? -v : v;
}

static void cia_pp_test(meow_bus_t *bus, meow_motor_t *motor)
{
    uint8_t nid = motor->node_id;
    uint16_t sw = 0U;
    uint8_t md = 0U;
    uint32_t start_ms;
    meow_status_t st;

    memset(&g_cia_pp_result, 0, sizeof(g_cia_pp_result));
    g_cia_pp_result.stage = 1U;

    meow_bus_set_sync(bus, false);

    st = (meow_status_t)canopen_download_u8(&bus->co, nid, 0x4401U, 0U, 0xFFU, 100U);
    board_delay_ms(20U);
    st = (meow_status_t)canopen_download_u8(&bus->co, nid, 0x4401U, 0U, 0U, 100U);
    if (st != MEOW_OK) { g_cia_pp_result.error = 1U; goto done; }

    /* Meow mode = Profile Position, then CiA402 enable ramp. */
    st = (meow_status_t)canopen_download_u8(&bus->co, nid, 0x4401U, 0U, 1U, 100U);
    if (st != MEOW_OK) { g_cia_pp_result.error = 2U; goto done; }
    board_delay_ms(20U);
    (void)canopen_download_i8(&bus->co, nid, 0x6060U, 0U, 1, 100U);
    board_delay_ms(20U);

    (void)canopen_download_u16(&bus->co, nid, 0x6040U, 0U, 0x0006U, 100U);
    board_delay_ms(20U);
    (void)canopen_download_u16(&bus->co, nid, 0x6040U, 0U, 0x0007U, 100U);
    board_delay_ms(20U);
    (void)canopen_download_u16(&bus->co, nid, 0x6040U, 0U, 0x000FU, 100U);
    board_delay_ms(50U);
    (void)canopen_upload_u16(&bus->co, nid, 0x6041U, 0U, &sw, 100U);
    g_cia_pp_result.sw_enabled = sw;
    if ((sw & 0x006FU) != 0x0027U) {
        g_cia_pp_result.error = 3U;
        goto done;
    }

    st = (meow_status_t)canopen_upload_f32(&bus->co, nid, 0x6064U, 0U,
                                           &g_cia_pp_result.start_rev, 100U);
    if (st != MEOW_OK) { g_cia_pp_result.error = 4U; goto done; }

    g_cia_pp_result.target_rev = (g_cia_pp_result.start_rev >= 0.0f) ? -0.25f : 0.25f;
    /* Write both object variants so the hybrid firmware picks its PP target. */
    st = (meow_status_t)canopen_download_i32(&bus->co, nid, 0x457AU, 0U,
                                             meow_float_to_q24(g_cia_pp_result.target_rev), 100U);
    if (st != MEOW_OK) { g_cia_pp_result.error = 5U; goto done; }
    board_delay_ms(20U);
    st = (meow_status_t)canopen_download_f32(&bus->co, nid, 0x607AU, 0U,
                                             g_cia_pp_result.target_rev, 100U);
    if (st != MEOW_OK) { g_cia_pp_result.error = 6U; goto done; }
    board_delay_ms(20U);
    (void)canopen_download_u16(&bus->co, nid, 0x6040U, 0U, 0x002FU, 100U);
    board_delay_ms(20U);
    (void)canopen_download_u16(&bus->co, nid, 0x6040U, 0U, 0x003FU, 100U);

    start_ms = board_millis();
    while ((board_millis() - start_ms) < 5000U) {
        meow_bus_poll(bus);
        (void)canopen_upload_u16(&bus->co, nid, 0x6041U, 0U, &sw, 100U);
        (void)canopen_upload_f32(&bus->co, nid, 0x6064U, 0U,
                                 &g_cia_pp_result.end_rev, 100U);
        if (absf_local2(g_cia_pp_result.end_rev - g_cia_pp_result.target_rev) < 0.01f) {
            g_cia_pp_result.pp_ok = 1U;
            break;
        }
        board_delay_ms(20U);
    }
    g_cia_pp_result.sw_final = sw;
    (void)canopen_upload_u8(&bus->co, nid, 0x6061U, 0U, &md, 100U);
    g_cia_pp_result.mode_display = md;
    g_cia_pp_result.elapsed_ms = board_millis() - start_ms;

done:
    (void)canopen_download_u16(&bus->co, nid, 0x6040U, 0U, 0x0006U, 100U);
    (void)canopen_download_u8(&bus->co, nid, 0x4401U, 0U, 0U, 100U);
    meow_bus_set_sync(bus, true);
    g_cia_pp_result.stage = 2U;
}

void demo_init(meow_bus_t *bus, meow_motor_t *motor)
{
    if (motor->node_id != MEOW_TEST_NODE) {
        return;
    }
    cia_pp_test(bus, motor);
}

void demo_poll(meow_bus_t *bus, meow_motor_t *motor)
{
    (void)bus;
    (void)motor;
}

#elif MEOW_DEMO_PVPP



typedef struct {
    uint32_t stage;
    uint32_t error;
    uint32_t pv_mode;
    uint32_t pp_mode;
    uint32_t pv_ok;
    uint32_t pp_ok;
    uint32_t elapsed_ms;
    float pv_start_rev;
    float pv_end_rev;
    float pv_velocity_rev_s;
    float pp_start_rev;
    float pp_target_rev;
    float pp_end_rev;
} pvpp_result_t;

/* Global so the result can be read over J-Link without a serial console. */
pvpp_result_t g_pvpp_result;

typedef struct {
    meow_bus_t *bus;
    meow_motor_t *motor;
    uint32_t phase_ms;
    uint32_t pp_retry_ms;
    uint8_t phase;
    uint8_t pp_retries;
    bool active;
} demo_state_t;

static demo_state_t g_demo;

static float absf_local(float v)
{
    return (v < 0.0f) ? -v : v;
}

void demo_init(meow_bus_t *bus, meow_motor_t *motor)
{
    /* Only exercise one motor; keep the other one disabled. */
    if (motor->node_id != MEOW_TEST_NODE) {
        return;
    }

    memset(&g_pvpp_result, 0, sizeof(g_pvpp_result));
    memset(&g_demo, 0, sizeof(g_demo));

    g_demo.bus = bus;
    g_demo.motor = motor;
    g_demo.phase_ms = board_millis();
    g_demo.phase = 0U;
    g_demo.active = true;
    g_pvpp_result.stage = 1U;
}

void demo_poll(meow_bus_t *bus, meow_motor_t *motor)
{
    meow_status_t st;
    uint32_t now;
    float diff;

    if (!g_demo.active || (g_demo.motor != motor)) {
        return;
    }
    if (!motor->ready) {
        g_pvpp_result.error = 1U;
        g_demo.active = false;
        return;
    }

    now = board_millis();
    g_pvpp_result.elapsed_ms = now - g_demo.phase_ms;

    switch (g_demo.phase) {
    case 0U:
        if (g_pvpp_result.elapsed_ms >= 1000U) {
            meow_profile_limits_t limits = {
                .velocity_limit_rev_s = 0.50f,
                .acceleration_rev_s2 = 2.00f,
                .deceleration_rev_s2 = 2.00f,
            };
            st = meow_motor_set_profile_limits(bus, motor, &limits);
            if (st != MEOW_OK) {
                g_pvpp_result.error = (uint32_t)st + 1U;
                g_demo.active = false;
                return;
            }
            (void)meow_motor_read_status(bus, motor);
            g_pvpp_result.pv_start_rev = motor->state.position_rev;
            /* Move away from the +0.5 single-turn boundary. */
            g_pvpp_result.pv_velocity_rev_s =
                (g_pvpp_result.pv_start_rev > 0.25f) ? -0.05f : 0.05f;
            st = meow_motor_set_velocity(bus, motor,
                                         g_pvpp_result.pv_velocity_rev_s);
            if (st != MEOW_OK) {
                g_pvpp_result.error = (uint32_t)st + 1U;
                g_demo.active = false;
                return;
            }
            g_demo.phase = 1U;
            g_demo.phase_ms = now;
            g_pvpp_result.stage = 2U;
        }
        break;

    case 1U:
        if (g_pvpp_result.elapsed_ms >= 1000U) {
            (void)meow_motor_read_status(bus, motor);
            g_pvpp_result.pv_end_rev = motor->state.position_rev;
            g_pvpp_result.pv_mode = motor->state.mode_display;
            g_pvpp_result.pv_ok =
                (motor->state.mode_display == MEOW_MODE_PROFILE_VELOCITY) &&
                (absf_local(g_pvpp_result.pv_end_rev - g_pvpp_result.pv_start_rev) >= 0.01f);
            (void)meow_motor_set_velocity(bus, motor, 0.0f);
            g_demo.phase = 2U;
            g_demo.phase_ms = now;
            g_pvpp_result.stage = 3U;
        }
        break;

    case 2U:
        if (g_pvpp_result.elapsed_ms >= 500U) {
            (void)meow_motor_disable(bus, motor);
            g_demo.phase = 3U;
            g_demo.phase_ms = now;
            g_pvpp_result.stage = 4U;
        }
        break;

    case 3U:
        if (g_pvpp_result.elapsed_ms >= 500U) {
            (void)meow_motor_read_status(bus, motor);
            g_pvpp_result.pp_start_rev = motor->state.position_rev;
            /* ~0.5 rev PP move, staying inside [-0.5, 0.5). */
            if (g_pvpp_result.pp_start_rev >= 0.0f) {
                g_pvpp_result.pp_target_rev = g_pvpp_result.pp_start_rev - 0.5f;
            } else {
                g_pvpp_result.pp_target_rev = g_pvpp_result.pp_start_rev + 0.5f;
            }
            st = meow_motor_set_position(bus, motor, g_pvpp_result.pp_target_rev);
            if (st != MEOW_OK) {
                g_pvpp_result.error = (uint32_t)st + 1U;
                g_demo.active = false;
                return;
            }
            g_demo.phase = 4U;
            g_demo.phase_ms = now;
            g_demo.pp_retry_ms = now;
            g_demo.pp_retries = 0U;
            g_pvpp_result.stage = 5U;
        }
        break;

    case 4U:
        if (((now - g_demo.pp_retry_ms) >= 200U) && (g_demo.pp_retries < 5U)) {
            g_demo.pp_retry_ms = now;
            g_demo.pp_retries++;
            (void)meow_motor_set_position(bus, motor, g_pvpp_result.pp_target_rev);
        }
        g_pvpp_result.pp_end_rev = motor->state.position_rev;
        g_pvpp_result.pp_mode = motor->state.mode_display;
        diff = g_pvpp_result.pp_end_rev - g_pvpp_result.pp_target_rev;
        if (diff < 0.0f) {
            diff = -diff;
        }
        if (motor->state.error_code != 0U) {
            g_pvpp_result.error = 0x10000U + motor->state.error_code;
            (void)meow_motor_disable(bus, motor);
            g_demo.phase = 5U;
            g_pvpp_result.stage = 6U;
        } else if (diff < 0.005f) {
            g_pvpp_result.pp_ok =
                (motor->state.mode_display == MEOW_MODE_PROFILE_POSITION) &&
                (motor->state.position_rev != g_pvpp_result.pp_start_rev);
            (void)meow_motor_disable(bus, motor);
            g_demo.phase = 5U;
            g_pvpp_result.stage = 6U;
        } else if (g_pvpp_result.elapsed_ms >= 5000U) {
            (void)meow_motor_disable(bus, motor);
            g_demo.phase = 5U;
            g_pvpp_result.stage = 6U;
        }
        break;

    default:
        g_demo.active = false;
        break;
    }
}

#elif MEOW_DEMO_MIT

typedef struct {
    meow_bus_t *bus;
    meow_motor_t *motor;
    uint32_t start_ms;
    uint8_t phase;
    bool active;
} demo_state_t;

static demo_state_t g_demo[MEOW_MAX_MOTORS];

void demo_init(meow_bus_t *bus, meow_motor_t *motor)
{
    uint8_t i;
    for (i = 0U; i < MEOW_MAX_MOTORS; i++) {
        if (!g_demo[i].active) {
            g_demo[i].bus = bus;
            g_demo[i].motor = motor;
            g_demo[i].start_ms = board_millis();
            g_demo[i].phase = 0U;
            g_demo[i].active = true;
            return;
        }
    }
}

void demo_poll(meow_bus_t *bus, meow_motor_t *motor)
{
    uint8_t i;

    for (i = 0U; i < MEOW_MAX_MOTORS; i++) {
        demo_state_t *d = &g_demo[i];
        uint32_t elapsed;

        if (!d->active || (d->motor != motor)) {
            continue;
        }
        if (!motor->ready) {
            d->active = false;
            continue;
        }

        elapsed = board_millis() - d->start_ms;

        if ((d->phase == 0U) && (elapsed >= 1000U)) {
            meow_status_t st = meow_motor_enable_mit(bus, motor);
            if (st != MEOW_OK) {
                printf("[demo] node 0x%02X enable MIT failed: %s\r\n",
                       motor->node_id, meow_status_str(st));
                d->active = false;
                continue;
            }
            (void)meow_motor_mit(motor, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            printf("[demo] node 0x%02X MIT enabled, zero torque\r\n", motor->node_id);
            d->phase = 1U;
        } else if ((d->phase == 1U) && (elapsed >= 4000U)) {
            (void)meow_motor_disable(bus, motor);
            printf("[demo] node 0x%02X MIT disabled\r\n", motor->node_id);
            d->active = false;
        } else if (d->phase == 1U) {
            (void)meow_motor_mit(motor, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        }
    }
}

#else

void demo_init(meow_bus_t *bus, meow_motor_t *motor)
{
    (void)bus;
    (void)motor;
}

void demo_poll(meow_bus_t *bus, meow_motor_t *motor)
{
    (void)bus;
    (void)motor;
}

#endif
