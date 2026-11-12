#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "board.h"
#include "board_led.h"
#include "demo.h"
#include "meow_bus.h"
#include "meow_motor.h"

#define APP_MAX_MOTORS MEOW_MAX_MOTORS
#define DISCOVERY_LISTEN_MS 1500U
#define APP_BUS_COUNT 3U

typedef struct {
    meow_bus_t bus;
    meow_motor_t motors[APP_MAX_MOTORS];
    uint8_t motor_count;
    board_fdcan_id_t can_id;
    const char *name;
} app_bus_t;

typedef struct {
    board_fdcan_id_t can_id;
    const char *name;
} app_bus_config_t;

/* Three buses run independently; each bus uses the same host node ID 0x10. */
static const app_bus_config_t g_bus_config[APP_BUS_COUNT] = {
    {BOARD_FDCAN_1, "CAN1 (PD0/PD1)"},
    {BOARD_FDCAN_2, "CAN2 (PB5/PB6)"},
    {BOARD_FDCAN_3, "CAN3 (PD12/PD13)"},
};

/* Runtime state stays in .bss; only the small config table above is in flash. */
static app_bus_t g_buses[APP_BUS_COUNT];

/* Keep the other buses alive while one bus is blocked in an SDO transfer. */
static void service_other_buses(void *ctx)
{
    app_bus_t *active = (app_bus_t *)ctx;
    uint8_t i;

    for (i = 0U; i < APP_BUS_COUNT; i++) {
        if (&g_buses[i] != active) {
            meow_bus_poll(&g_buses[i].bus);
        }
    }
}

static const char *model_name(const meow_motor_t *motor)
{
    return meow_model_name(motor->model);
}

static bool add_motor(app_bus_t *b, uint8_t node_id)
{
    if (meow_bus_find_motor(&b->bus, node_id) != 0) {
        return true;
    }
    if (b->motor_count >= APP_MAX_MOTORS) {
        return false;
    }
    if (meow_bus_add_motor(&b->bus, &b->motors[b->motor_count], node_id) < 0) {
        return false;
    }
    b->motor_count++;
    return true;
}

static void print_identity(const app_bus_t *b, const meow_motor_t *motor)
{
    printf("[%s] node 0x%02X: %s vendor=0x%08lX product=0x%08lX rev=0x%08lX\r\n",
           b->name,
           motor->node_id,
           model_name(motor),
           (unsigned long)motor->vendor_id,
           (unsigned long)motor->product_code,
           (unsigned long)motor->revision);
    printf("           peak=%0.3f Nm mit_factor=%0.6f heartbeats=%u ms\r\n",
           motor->peak_torque_nm,
           motor->mit_factor,
           motor->heartbeat_period_ms);
}

static void print_state(const app_bus_t *b, const meow_motor_t *motor)
{
    meow_motor_state_t st;

    if (meow_motor_get_state(motor, &st) != MEOW_OK) {
        return;
    }

    printf("  [%s] 0x%02X %-14s %-8s online=%u ready=%u enabled=%u mode=%u "
           "pos=%0.4f rev vel=%0.3f rev/s trq=%0.3f Nm "
           "temp=%0.1f/%0.1f C err=0x%04X %s\r\n",
           b->name,
           motor->node_id,
           model_name(motor),
           meow_mode_name(st.mode_display),
           st.online ? 1U : 0U,
           motor->ready ? 1U : 0U,
           st.enabled ? 1U : 0U,
           st.mode_display,
           st.position_rev,
           st.velocity_rev_s,
           st.torque_nm,
           st.driver_temp_c,
           st.motor_temp_c,
           st.error_code,
           meow_error_name(st.error_code));
}

static void configure_bus(app_bus_t *b)
{
    meow_status_t st;
    uint8_t i;

    if (b->motor_count == 0U) {
        return;
    }

    printf("[%s] configuring %u motor(s)...\r\n", b->name, b->motor_count);
    st = meow_bus_configure(&b->bus, NULL);
    if (st != MEOW_OK) {
        printf("[%s] configure returned %s\r\n", b->name, meow_status_str(st));
    }

    for (i = 0U; i < b->motor_count; i++) {
        print_identity(b, &b->motors[i]);
        if (b->motors[i].ready) {
            demo_init(&b->bus, &b->motors[i]);
        }
    }
}

static bool init_bus(app_bus_t *b)
{
    if (meow_bus_init(&b->bus, b->can_id, MEOW_HOST_NODE_ID) != MEOW_OK) {
        printf("[%s] meow_bus_init failed\r\n", b->name);
        return false;
    }

    b->bus.co.service_cb = service_other_buses;
    b->bus.co.service_ctx = b;

    if (meow_bus_start(&b->bus) != MEOW_OK) {
        printf("[%s] meow_bus_start failed\r\n", b->name);
        return false;
    }

    /*
     * Keep host heartbeats off until every motor is configured. The motor
     * arms its 0x1016 watchdog on the first heartbeat, so sending one during
     * a later blocking SDO sequence would trip a false 0x8130.
     */
    canopen_set_heartbeat(&b->bus.co, false);
    b->bus.discovered_count = 0U;
    (void)canopen_nmt(&b->bus.co, CO_NMT_CMD_PREOP, 0U);
    return true;
}

/* Discover all buses together so no motor misses host heartbeats. */
static void discover_all(void)
{
    uint32_t start = board_millis();
    uint8_t i;

    while ((board_millis() - start) < DISCOVERY_LISTEN_MS) {
        for (i = 0U; i < APP_BUS_COUNT; i++) {
            meow_bus_poll(&g_buses[i].bus);
        }
        board_delay_ms(1U);
    }

    for (i = 0U; i < APP_BUS_COUNT; i++) {
        app_bus_t *b = &g_buses[i];
        uint8_t j;

        printf("[%s] discovery: %u heartbeat(s)\r\n",
               b->name, b->bus.discovered_count);
        for (j = 0U; j < b->bus.discovered_count; j++) {
            (void)add_motor(b, b->bus.discovered[j]);
        }
    }
}

static void scan_bus(app_bus_t *b)
{
    uint16_t nid;

    printf("[%s] scanning node IDs %u..%u...\r\n", b->name, 1U, 16U);
    for (nid = 1U; nid <= 16U; nid++) {
        meow_motor_t temp;

        /* Let the bus recover if a previous probe hit bus-off. */
        meow_bus_poll(&b->bus);

        memset(&temp, 0, sizeof(temp));
        temp.node_id = (uint8_t)nid;
        if (meow_motor_probe(&b->bus, &temp) == MEOW_OK) {
            (void)add_motor(b, (uint8_t)nid);
        }
    }
}

static void initialize_motors(void)
{
    uint8_t i;

    for (i = 0U; i < APP_BUS_COUNT; i++) {
        app_bus_t *b = &g_buses[i];

        if (b->motor_count == 0U) {
            scan_bus(b);
        }
        if (b->motor_count > 0U) {
            configure_bus(b);
        } else {
            /* Keep an empty bus quiet. */
            canopen_set_heartbeat(&b->bus.co, false);
        }
    }
}

static void start_bus_heartbeats(app_bus_t *b)
{
    uint8_t m;

    if (b->motor_count == 0U) {
        return;
    }

    /* Clear any fault latched before the host heartbeat was running. */
    for (m = 0U; m < b->motor_count; m++) {
        if (b->motors[m].ready) {
            (void)meow_motor_clear_error(&b->bus, &b->motors[m]);
        }
    }

    canopen_set_heartbeat(&b->bus.co, true);
}

static void start_heartbeats(void)
{
    uint8_t i;

    for (i = 0U; i < APP_BUS_COUNT; i++) {
        start_bus_heartbeats(&g_buses[i]);
    }
}

/* A motor that boots after the discovery window is picked up here. */
static void add_late_motors(app_bus_t *b)
{
    uint8_t before = b->motor_count;
    uint8_t i;

    if (b->motor_count != 0U) {
        return;
    }

    for (i = 0U; i < b->bus.discovered_count; i++) {
        (void)add_motor(b, b->bus.discovered[i]);
    }

    if (b->motor_count > before) {
        configure_bus(b);
        start_bus_heartbeats(b);
    }
}

int main(void)
{
    uint32_t last_print_ms = 0U;
    uint8_t i;

    for (i = 0U; i < APP_BUS_COUNT; i++) {
        g_buses[i].can_id = g_bus_config[i].can_id;
        g_buses[i].name = g_bus_config[i].name;
    }

    board_init();

    /* Default power state: both controllable motor ports on, no checks. */
    board_motor_power_set(BOARD_MOTOR_POWER_OUT1, true);
    board_motor_power_set(BOARD_MOTOR_POWER_OUT2, true);

    printf("\r\n");
    printf("==============================================\r\n");
    printf(" DM-MC02 / STM32H723 3-bus Meow driver example\r\n");
    printf(" CAN1 + CAN2 + CAN3, host node 0x10 on each bus\r\n");
    printf(" OUT1/OUT2 motor power: ON (CAN3 uses external 24V)\r\n");
    printf(" SYSCLK = %lu Hz\r\n", (unsigned long)SystemCoreClock);
    printf("==============================================\r\n");

    for (i = 0U; i < APP_BUS_COUNT; i++) {
        (void)init_bus(&g_buses[i]);
    }

    discover_all();
    initialize_motors();
    start_heartbeats();

    while (1) {
        uint32_t now;

        board_led_effect_poll();

        for (i = 0U; i < APP_BUS_COUNT; i++) {
            uint8_t m;
            meow_bus_poll(&g_buses[i].bus);
            meow_bus_sync(&g_buses[i].bus);
            for (m = 0U; m < g_buses[i].motor_count; m++) {
                demo_poll(&g_buses[i].bus, &g_buses[i].motors[m]);
            }
        }

        now = board_millis();
        if ((now - last_print_ms) >= 1000U) {
            last_print_ms = now;
            printf("--- status @ %lu ms ---\r\n", (unsigned long)now);
            for (i = 0U; i < APP_BUS_COUNT; i++) {
                app_bus_t *b = &g_buses[i];
                uint8_t m;

                add_late_motors(b);

                printf("[%s] motors=%u rx=%lu tx=%lu drop=%lu busoff=%lu\r\n",
                       b->name,
                       b->motor_count,
                       (unsigned long)b->bus.can.rx_frames,
                       (unsigned long)b->bus.can.tx_frames,
                       (unsigned long)b->bus.can.rx_dropped,
                       (unsigned long)b->bus.bus_off_count);

                for (m = 0U; m < b->motor_count; m++) {
                    print_state(b, &b->motors[m]);
                }
            }
        }

        board_delay_ms(1U);
    }
}
