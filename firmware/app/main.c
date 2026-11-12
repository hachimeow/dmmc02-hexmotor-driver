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
#define RESCAN_PERIOD_MS 10000U

#if MEOW_CAN_PORT == 1
#define APP_CAN_PORT BOARD_FDCAN_1
#define APP_CAN_NAME "CAN1 (PD0/PD1)"
#elif MEOW_CAN_PORT == 2
#define APP_CAN_PORT BOARD_FDCAN_2
#define APP_CAN_NAME "CAN2 (PB5/PB6)"
#elif MEOW_CAN_PORT == 3
#define APP_CAN_PORT BOARD_FDCAN_3
#define APP_CAN_NAME "CAN3 (PD12/PD13)"
#else
#error "MEOW_CAN_PORT must be 1, 2 or 3"
#endif

static meow_bus_t g_bus;
static meow_motor_t g_motors[APP_MAX_MOTORS];
static uint8_t g_motor_count;

static const char *model_name(const meow_motor_t *motor)
{
    return meow_model_name(motor->model);
}

static bool add_motor(uint8_t node_id)
{
    if (meow_bus_find_motor(&g_bus, node_id) != 0) {
        return true;
    }
    if (g_motor_count >= APP_MAX_MOTORS) {
        return false;
    }
    if (meow_bus_add_motor(&g_bus, &g_motors[g_motor_count], node_id) < 0) {
        return false;
    }
    g_motor_count++;
    return true;
}

static void discover_and_add(void)
{
    uint8_t i;

    (void)meow_bus_discover(&g_bus, DISCOVERY_LISTEN_MS);
    printf("discovery: %u heartbeat(s)\r\n", g_bus.discovered_count);

    for (i = 0U; i < g_bus.discovered_count; i++) {
        (void)add_motor(g_bus.discovered[i]);
    }

    if (g_motor_count == 0U) {
        printf("no heartbeats, scanning node IDs 1..127...\r\n");
        for (uint16_t nid = 1U; nid <= 127U; nid++) {
            meow_motor_t temp;
            memset(&temp, 0, sizeof(temp));
            temp.node_id = (uint8_t)nid;
            if (meow_motor_probe(&g_bus, &temp) == MEOW_OK) {
                (void)add_motor((uint8_t)nid);
            }
        }
    }
}

static void print_identity(const meow_motor_t *motor)
{
    printf("  node 0x%02X: %s vendor=0x%08lX product=0x%08lX rev=0x%08lX\r\n",
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

static void print_state(const meow_motor_t *motor)
{
    meow_motor_state_t st;

    if (meow_motor_get_state(motor, &st) != MEOW_OK) {
        return;
    }

    printf("  0x%02X %-14s %-8s online=%u ready=%u enabled=%u mode=%u "
           "pos=%0.4f rev vel=%0.3f rev/s trq=%0.3f Nm "
           "temp=%0.1f/%0.1f C err=0x%04X %s\r\n",
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

int main(void)
{
    uint32_t last_print_ms = 0U;
    uint32_t last_scan_ms = 0U;
    uint8_t i;

    board_init();

    printf("\r\n");
    printf("==============================================\r\n");
    printf(" DM-MC02 / STM32H723 Meow compressed-MIT driver\r\n");
    printf(" SYSCLK = %lu Hz\r\n", (unsigned long)SystemCoreClock);
    printf(" %s, 1 Mbit/s nominal, 5 Mbit/s data\r\n", APP_CAN_NAME);
    printf(" host node = 0x%02X, shared RPDO1 = 0x%03X, RPDO2 = 0x%03X\r\n",
           MEOW_HOST_NODE_ID,
           0x180U + MEOW_HOST_NODE_ID,
           0x280U + MEOW_HOST_NODE_ID);
    printf("==============================================\r\n");

    if (meow_bus_init(&g_bus, APP_CAN_PORT, MEOW_HOST_NODE_ID) != MEOW_OK) {
        printf("meow_bus_init failed on %s\r\n", APP_CAN_NAME);
        while (1) {
        }
    }
    if (meow_bus_start(&g_bus) != MEOW_OK) {
        printf("meow_bus_start failed on %s\r\n", APP_CAN_NAME);
        while (1) {
        }
    }

#if MEOW_MOTOR_POWER_PORT != 0
    board_motor_power_set((board_motor_power_port_t)MEOW_MOTOR_POWER_PORT, true);
    printf("motor power OUT%d enabled, waiting for the drive to boot...\r\n",
           MEOW_MOTOR_POWER_PORT);
    board_delay_ms(500U);
#else
    printf("motor power output not enabled (MEOW_MOTOR_POWER_PORT=0)\r\n");
#endif

    discover_and_add();
    last_scan_ms = board_millis();

    if (g_motor_count > 0U) {
        printf("configuring %u motor(s)...\r\n", g_motor_count);
        {
            meow_status_t st = meow_bus_configure(&g_bus, NULL);
            if (st != MEOW_OK) {
                printf("meow_bus_configure returned %s\r\n", meow_status_str(st));
            }
        }
        for (i = 0U; i < g_motor_count; i++) {
            print_identity(&g_motors[i]);
            if (g_motors[i].ready) {
                demo_init(&g_bus, &g_motors[i]);
            }
        }
    } else {
        printf("no supported Meow motor found; rescan every %u ms\r\n",
               RESCAN_PERIOD_MS);
    }

    while (1) {
        uint32_t now;

        board_led_effect_poll();
        meow_bus_poll(&g_bus);
        meow_bus_sync(&g_bus);

        for (i = 0U; i < g_motor_count; i++) {
            demo_poll(&g_bus, &g_motors[i]);
        }

        now = board_millis();
        if ((now - last_print_ms) >= 1000U) {
            last_print_ms = now;
            printf("--- status @ %lu ms ---\r\n", (unsigned long)now);
            for (i = 0U; i < g_motor_count; i++) {
                print_state(&g_motors[i]);
            }
            printf("    CAN rx=%lu tx=%lu sync=%lu drop=%lu busoff=%lu\r\n",
                   (unsigned long)g_bus.can.rx_frames,
                   (unsigned long)g_bus.can.tx_frames,
                   (unsigned long)g_bus.tx_frames,
                   (unsigned long)g_bus.tx_dropped,
                   (unsigned long)g_bus.bus_off_count);
        }

        if ((g_motor_count == 0U) && ((now - last_scan_ms) >= RESCAN_PERIOD_MS)) {
            last_scan_ms = now;
            discover_and_add();
            if (g_motor_count > 0U) {
                (void)meow_bus_configure(&g_bus, NULL);
                for (i = 0U; i < g_motor_count; i++) {
                    print_identity(&g_motors[i]);
                    if (g_motors[i].ready) {
                        demo_init(&g_bus, &g_motors[i]);
                    }
                }
            }
        }

        board_delay_ms(1U);
    }
}
