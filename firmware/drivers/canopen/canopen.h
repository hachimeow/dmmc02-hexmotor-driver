#ifndef CANOPEN_H
#define CANOPEN_H

#include <stdbool.h>
#include <stdint.h>

#include "canfd.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CO_COB_NMT             0x000U
#define CO_COB_SYNC            0x080U
#define CO_COB_EMCY(nid)       (0x080U + (uint32_t)(nid))
#define CO_COB_TPDO1(nid)      (0x180U + (uint32_t)(nid))
#define CO_COB_TPDO2(nid)      (0x280U + (uint32_t)(nid))
#define CO_COB_SDO_TX(nid)     (0x600U + (uint32_t)(nid))
#define CO_COB_SDO_RX(nid)     (0x580U + (uint32_t)(nid))
#define CO_COB_HEARTBEAT(nid)  (0x700U + (uint32_t)(nid))

typedef enum {
    CO_NMT_BOOTUP = 0x00,
    CO_NMT_STOPPED = 0x04,
    CO_NMT_OPERATIONAL = 0x05,
    CO_NMT_PREOP = 0x7F,
    CO_NMT_UNKNOWN = 0xFF
} co_nmt_state_t;

typedef enum {
    CO_NMT_CMD_START = 0x01,
    CO_NMT_CMD_STOP = 0x02,
    CO_NMT_CMD_PREOP = 0x80,
    CO_NMT_CMD_RESET_NODE = 0x81,
    CO_NMT_CMD_RESET_COMM = 0x82
} co_nmt_command_t;

typedef enum {
    CO_OK = 0,
    CO_ERR_IO,
    CO_ERR_TIMEOUT,
    CO_ERR_BUSY,
    CO_ERR_SDO_ABORT,
    CO_ERR_SDO_PROTOCOL,
    CO_ERR_NOT_SUPPORTED,
    CO_ERR_INVALID_ARG
} co_status_t;

typedef struct {
    bool pending;
    uint8_t node_id;
    uint16_t index;
    uint8_t subindex;
    bool upload;
    bool done;
    bool aborted;
    uint32_t abort_code;
    uint8_t data[4];
    uint8_t len;
} co_sdo_t;

typedef void (*co_rx_cb_t)(void *ctx, uint32_t cob_id,
                           const uint8_t *data, uint8_t len);

typedef struct {
    canfd_bus_t *bus;
    co_sdo_t sdo;
    uint8_t host_node_id;
    uint16_t heartbeat_period_ms;
    uint32_t last_heartbeat_tx_ms;
    co_nmt_state_t host_state;
    bool heartbeat_enabled;
    co_rx_cb_t rx_cb;
    void *rx_ctx;
} canopen_t;

void canopen_init(canopen_t *co, canfd_bus_t *bus, uint8_t host_node_id,
                  co_rx_cb_t rx_cb, void *rx_ctx);
void canopen_poll(canopen_t *co);

void canopen_set_heartbeat(canopen_t *co, bool enabled);
co_status_t canopen_send_heartbeat(canopen_t *co, co_nmt_state_t state);
co_status_t canopen_nmt(canopen_t *co, co_nmt_command_t command, uint8_t node_id);

co_status_t canopen_sdo_upload(canopen_t *co, uint8_t node_id,
                               uint16_t index, uint8_t subindex,
                               uint8_t *out, uint8_t out_cap, uint8_t *out_len,
                               uint32_t timeout_ms);
co_status_t canopen_sdo_download(canopen_t *co, uint8_t node_id,
                                 uint16_t index, uint8_t subindex,
                                 const uint8_t *data, uint8_t len,
                                 uint32_t timeout_ms);

co_status_t canopen_upload_u8(canopen_t *co, uint8_t nid, uint16_t index,
                              uint8_t sub, uint8_t *value, uint32_t timeout_ms);
co_status_t canopen_upload_u16(canopen_t *co, uint8_t nid, uint16_t index,
                               uint8_t sub, uint16_t *value, uint32_t timeout_ms);
co_status_t canopen_upload_u32(canopen_t *co, uint8_t nid, uint16_t index,
                               uint8_t sub, uint32_t *value, uint32_t timeout_ms);
co_status_t canopen_upload_i8(canopen_t *co, uint8_t nid, uint16_t index,
                              uint8_t sub, int8_t *value, uint32_t timeout_ms);
co_status_t canopen_upload_i16(canopen_t *co, uint8_t nid, uint16_t index,
                               uint8_t sub, int16_t *value, uint32_t timeout_ms);
co_status_t canopen_upload_i32(canopen_t *co, uint8_t nid, uint16_t index,
                               uint8_t sub, int32_t *value, uint32_t timeout_ms);
co_status_t canopen_upload_f32(canopen_t *co, uint8_t nid, uint16_t index,
                               uint8_t sub, float *value, uint32_t timeout_ms);

co_status_t canopen_download_u8(canopen_t *co, uint8_t nid, uint16_t index,
                                uint8_t sub, uint8_t value, uint32_t timeout_ms);
co_status_t canopen_download_u16(canopen_t *co, uint8_t nid, uint16_t index,
                                 uint8_t sub, uint16_t value, uint32_t timeout_ms);
co_status_t canopen_download_u32(canopen_t *co, uint8_t nid, uint16_t index,
                                 uint8_t sub, uint32_t value, uint32_t timeout_ms);
co_status_t canopen_download_i8(canopen_t *co, uint8_t nid, uint16_t index,
                                uint8_t sub, int8_t value, uint32_t timeout_ms);
co_status_t canopen_download_i16(canopen_t *co, uint8_t nid, uint16_t index,
                                 uint8_t sub, int16_t value, uint32_t timeout_ms);
co_status_t canopen_download_i32(canopen_t *co, uint8_t nid, uint16_t index,
                                 uint8_t sub, int32_t value, uint32_t timeout_ms);
co_status_t canopen_download_f32(canopen_t *co, uint8_t nid, uint16_t index,
                                 uint8_t sub, float value, uint32_t timeout_ms);

const char *canopen_status_str(co_status_t status);
const char *canopen_nmt_state_str(co_nmt_state_t state);

#ifdef __cplusplus
}
#endif

#endif /* CANOPEN_H */
