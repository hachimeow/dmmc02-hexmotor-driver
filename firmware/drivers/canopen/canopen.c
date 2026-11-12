#include "canopen.h"

#include <string.h>

#include "board.h"

static uint16_t rd_u16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static uint32_t rd_u32(const uint8_t *p)
{
    return (uint32_t)p[0] |
           ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}

static void wr_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFU);
    p[1] = (uint8_t)((v >> 8) & 0xFFU);
}

static void wr_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFFU);
    p[1] = (uint8_t)((v >> 8) & 0xFFU);
    p[2] = (uint8_t)((v >> 16) & 0xFFU);
    p[3] = (uint8_t)((v >> 24) & 0xFFU);
}

void canopen_init(canopen_t *co, canfd_bus_t *bus, uint8_t host_node_id,
                  co_rx_cb_t rx_cb, void *rx_ctx)
{
    memset(co, 0, sizeof(*co));
    co->bus = bus;
    co->host_node_id = host_node_id & 0x7FU;
    co->heartbeat_period_ms = 50U;
    co->host_state = CO_NMT_OPERATIONAL;
    co->heartbeat_enabled = false;
    co->rx_cb = rx_cb;
    co->rx_ctx = rx_ctx;
    co->last_heartbeat_tx_ms = board_millis();
}

static co_status_t canopen_send_classic(canopen_t *co, uint32_t cob_id,
                                        const uint8_t *data, uint8_t len)
{
    if ((co == 0) || (co->bus == 0)) {
        return CO_ERR_IO;
    }
    if (canfd_send(co->bus, cob_id, data, len, false, 10U) != HAL_OK) {
        return CO_ERR_IO;
    }
    return CO_OK;
}

co_status_t canopen_send_heartbeat(canopen_t *co, co_nmt_state_t state)
{
    uint8_t data[1];
    if ((co == 0) || (co->host_node_id == 0U)) {
        return CO_ERR_INVALID_ARG;
    }
    data[0] = (uint8_t)state;
    return canopen_send_classic(co, CO_COB_HEARTBEAT(co->host_node_id), data, 1U);
}

void canopen_set_heartbeat(canopen_t *co, bool enabled)
{
    if (co != 0) {
        co->heartbeat_enabled = enabled;
        co->last_heartbeat_tx_ms = board_millis();
    }
}

co_status_t canopen_nmt(canopen_t *co, co_nmt_command_t command, uint8_t node_id)
{
    uint8_t data[2];
    data[0] = (uint8_t)command;
    data[1] = node_id & 0x7FU;
    return canopen_send_classic(co, CO_COB_NMT, data, 2U);
}

static void canopen_sdo_parse_response(canopen_t *co, const uint8_t *data, uint8_t len)
{
    co_sdo_t *sdo = &co->sdo;

    if (len < 8U) {
        return;
    }

    if ((rd_u16(&data[1]) != sdo->index) || (data[3] != sdo->subindex)) {
        return;
    }

    if (data[0] == 0x80U) {
        sdo->aborted = true;
        sdo->abort_code = rd_u32(&data[4]);
        sdo->done = true;
        return;
    }

    if (sdo->upload) {
        uint8_t n;
        uint8_t payload_len;

        if ((data[0] & 0xE0U) != 0x40U) {
            sdo->aborted = true;
            sdo->abort_code = 0x05040000UL; /* SDO protocol timed out / invalid command */
            sdo->done = true;
            return;
        }

        if ((data[0] & 0x02U) == 0U) {
            /* Segmented upload is not implemented in the bring-up driver. */
            sdo->aborted = true;
            sdo->abort_code = 0x05040000UL;
            sdo->done = true;
            return;
        }

        n = (uint8_t)((data[0] >> 2) & 0x03U);
        payload_len = (uint8_t)(4U - n);
        memcpy(sdo->data, &data[4], payload_len);
        sdo->len = payload_len;
        sdo->done = true;
    } else {
        if (data[0] == 0x60U) {
            sdo->done = true;
        } else {
            sdo->aborted = true;
            sdo->abort_code = 0x05040000UL;
            sdo->done = true;
        }
    }
}

static void canopen_service(canopen_t *co)
{
    if ((co != 0) && (co->service_cb != 0)) {
        co->service_cb(co->service_ctx);
    }
}

void canopen_poll(canopen_t *co)
{
    canfd_frame_t frame;

    if ((co == 0) || (co->bus == 0)) {
        return;
    }

    while (canfd_receive(co->bus, &frame)) {
        if (co->sdo.pending &&
            (frame.id == CO_COB_SDO_RX(co->sdo.node_id)) &&
            (frame.len >= 8U)) {
            canopen_sdo_parse_response(co, frame.data, frame.len);
            continue;
        }

        if (co->rx_cb != 0) {
            co->rx_cb(co->rx_ctx, frame.id, frame.data, frame.len);
        }
    }

    if (co->heartbeat_enabled) {
        uint32_t now = board_millis();
        if ((now - co->last_heartbeat_tx_ms) >= co->heartbeat_period_ms) {
            co->last_heartbeat_tx_ms = now;
            (void)canopen_send_heartbeat(co, co->host_state);
        }
    }
}

co_status_t canopen_sdo_upload(canopen_t *co, uint8_t node_id,
                               uint16_t index, uint8_t subindex,
                               uint8_t *out, uint8_t out_cap, uint8_t *out_len,
                               uint32_t timeout_ms)
{
    uint8_t request[8] = {0};
    uint32_t start_ms;

    if ((co == 0) || (out == 0) || (out_len == 0) || (node_id == 0U) || (node_id > 127U)) {
        return CO_ERR_INVALID_ARG;
    }
    if (co->sdo.pending) {
        return CO_ERR_BUSY;
    }

    request[0] = 0x40U;
    wr_u16(&request[1], index);
    request[3] = subindex;

    co->sdo.pending = true;
    co->sdo.node_id = node_id;
    co->sdo.index = index;
    co->sdo.subindex = subindex;
    co->sdo.upload = true;
    co->sdo.done = false;
    co->sdo.aborted = false;
    co->sdo.abort_code = 0U;
    co->sdo.len = 0U;

    if (canopen_send_classic(co, CO_COB_SDO_TX(node_id), request, 8U) != CO_OK) {
        co->sdo.pending = false;
        return CO_ERR_IO;
    }

    start_ms = board_millis();
    while (!co->sdo.done) {
        canopen_service(co);
        canopen_poll(co);
        if ((board_millis() - start_ms) >= timeout_ms) {
            co->sdo.pending = false;
            return CO_ERR_TIMEOUT;
        }
    }

    co->sdo.pending = false;

    if (co->sdo.aborted) {
        return CO_ERR_SDO_ABORT;
    }
    if (co->sdo.len == 0U) {
        return CO_ERR_SDO_PROTOCOL;
    }

    if (co->sdo.len > out_cap) {
        return CO_ERR_SDO_PROTOCOL;
    }

    memcpy(out, co->sdo.data, co->sdo.len);
    *out_len = co->sdo.len;
    return CO_OK;
}

co_status_t canopen_sdo_download(canopen_t *co, uint8_t node_id,
                                 uint16_t index, uint8_t subindex,
                                 const uint8_t *data, uint8_t len,
                                 uint32_t timeout_ms)
{
    uint8_t request[8] = {0};
    uint32_t start_ms;
    uint8_t n;

    if ((co == 0) || (data == 0) || (node_id == 0U) || (node_id > 127U)) {
        return CO_ERR_INVALID_ARG;
    }
    if ((len == 0U) || (len > 4U)) {
        return CO_ERR_NOT_SUPPORTED;
    }
    if (co->sdo.pending) {
        return CO_ERR_BUSY;
    }

    n = (uint8_t)(4U - len);
    request[0] = (uint8_t)(0x20U | (n << 2) | 0x03U);
    wr_u16(&request[1], index);
    request[3] = subindex;
    memcpy(&request[4], data, len);

    co->sdo.pending = true;
    co->sdo.node_id = node_id;
    co->sdo.index = index;
    co->sdo.subindex = subindex;
    co->sdo.upload = false;
    co->sdo.done = false;
    co->sdo.aborted = false;
    co->sdo.abort_code = 0U;
    co->sdo.len = 0U;

    if (canopen_send_classic(co, CO_COB_SDO_TX(node_id), request, 8U) != CO_OK) {
        co->sdo.pending = false;
        return CO_ERR_IO;
    }

    start_ms = board_millis();
    while (!co->sdo.done) {
        canopen_service(co);
        canopen_poll(co);
        if ((board_millis() - start_ms) >= timeout_ms) {
            co->sdo.pending = false;
            return CO_ERR_TIMEOUT;
        }
    }

    co->sdo.pending = false;

    if (co->sdo.aborted) {
        return CO_ERR_SDO_ABORT;
    }
    return CO_OK;
}

co_status_t canopen_upload_u8(canopen_t *co, uint8_t nid, uint16_t index,
                              uint8_t sub, uint8_t *value, uint32_t timeout_ms)
{
    uint8_t buf[4];
    uint8_t len = 0U;
    co_status_t st = canopen_sdo_upload(co, nid, index, sub, buf, sizeof(buf), &len, timeout_ms);
    if ((st == CO_OK) && (len >= 1U)) {
        *value = buf[0];
    } else if (st == CO_OK) {
        st = CO_ERR_SDO_PROTOCOL;
    }
    return st;
}

co_status_t canopen_upload_i8(canopen_t *co, uint8_t nid, uint16_t index,
                              uint8_t sub, int8_t *value, uint32_t timeout_ms)
{
    uint8_t raw = 0U;
    co_status_t st = canopen_upload_u8(co, nid, index, sub, &raw, timeout_ms);
    if (st == CO_OK) {
        *value = (int8_t)raw;
    }
    return st;
}

co_status_t canopen_upload_u16(canopen_t *co, uint8_t nid, uint16_t index,
                               uint8_t sub, uint16_t *value, uint32_t timeout_ms)
{
    uint8_t buf[4];
    uint8_t len = 0U;
    co_status_t st = canopen_sdo_upload(co, nid, index, sub, buf, sizeof(buf), &len, timeout_ms);
    if ((st == CO_OK) && (len >= 2U)) {
        *value = rd_u16(buf);
    } else if (st == CO_OK) {
        st = CO_ERR_SDO_PROTOCOL;
    }
    return st;
}

co_status_t canopen_upload_i16(canopen_t *co, uint8_t nid, uint16_t index,
                               uint8_t sub, int16_t *value, uint32_t timeout_ms)
{
    uint16_t raw = 0U;
    co_status_t st = canopen_upload_u16(co, nid, index, sub, &raw, timeout_ms);
    if (st == CO_OK) {
        *value = (int16_t)raw;
    }
    return st;
}

co_status_t canopen_upload_u32(canopen_t *co, uint8_t nid, uint16_t index,
                               uint8_t sub, uint32_t *value, uint32_t timeout_ms)
{
    uint8_t buf[4];
    uint8_t len = 0U;
    co_status_t st = canopen_sdo_upload(co, nid, index, sub, buf, sizeof(buf), &len, timeout_ms);
    if ((st == CO_OK) && (len >= 4U)) {
        *value = rd_u32(buf);
    } else if (st == CO_OK) {
        st = CO_ERR_SDO_PROTOCOL;
    }
    return st;
}

co_status_t canopen_upload_f32(canopen_t *co, uint8_t nid, uint16_t index,
                               uint8_t sub, float *value, uint32_t timeout_ms)
{
    uint32_t raw = 0U;
    co_status_t st = canopen_upload_u32(co, nid, index, sub, &raw, timeout_ms);
    if (st == CO_OK) {
        memcpy(value, &raw, sizeof(raw));
    }
    return st;
}

co_status_t canopen_upload_i32(canopen_t *co, uint8_t nid, uint16_t index,
                               uint8_t sub, int32_t *value, uint32_t timeout_ms)
{
    uint32_t raw = 0U;
    co_status_t st = canopen_upload_u32(co, nid, index, sub, &raw, timeout_ms);
    if (st == CO_OK) {
        *value = (int32_t)raw;
    }
    return st;
}

co_status_t canopen_download_u8(canopen_t *co, uint8_t nid, uint16_t index,
                                uint8_t sub, uint8_t value, uint32_t timeout_ms)
{
    return canopen_sdo_download(co, nid, index, sub, &value, 1U, timeout_ms);
}

co_status_t canopen_download_i8(canopen_t *co, uint8_t nid, uint16_t index,
                                uint8_t sub, int8_t value, uint32_t timeout_ms)
{
    return canopen_sdo_download(co, nid, index, sub, (const uint8_t *)&value, 1U, timeout_ms);
}

co_status_t canopen_download_u16(canopen_t *co, uint8_t nid, uint16_t index,
                                 uint8_t sub, uint16_t value, uint32_t timeout_ms)
{
    uint8_t buf[2];
    wr_u16(buf, value);
    return canopen_sdo_download(co, nid, index, sub, buf, 2U, timeout_ms);
}

co_status_t canopen_download_i16(canopen_t *co, uint8_t nid, uint16_t index,
                                 uint8_t sub, int16_t value, uint32_t timeout_ms)
{
    uint8_t buf[2];
    wr_u16(buf, (uint16_t)value);
    return canopen_sdo_download(co, nid, index, sub, buf, 2U, timeout_ms);
}

co_status_t canopen_download_u32(canopen_t *co, uint8_t nid, uint16_t index,
                                 uint8_t sub, uint32_t value, uint32_t timeout_ms)
{
    uint8_t buf[4];
    wr_u32(buf, value);
    return canopen_sdo_download(co, nid, index, sub, buf, 4U, timeout_ms);
}

co_status_t canopen_download_f32(canopen_t *co, uint8_t nid, uint16_t index,
                                 uint8_t sub, float value, uint32_t timeout_ms)
{
    uint32_t raw;
    uint8_t buf[4];
    memcpy(&raw, &value, sizeof(raw));
    wr_u32(buf, raw);
    return canopen_sdo_download(co, nid, index, sub, buf, 4U, timeout_ms);
}

co_status_t canopen_download_i32(canopen_t *co, uint8_t nid, uint16_t index,
                                 uint8_t sub, int32_t value, uint32_t timeout_ms)
{
    uint8_t buf[4];
    wr_u32(buf, (uint32_t)value);
    return canopen_sdo_download(co, nid, index, sub, buf, 4U, timeout_ms);
}

const char *canopen_status_str(co_status_t status)
{
    switch (status) {
    case CO_OK:
        return "OK";
    case CO_ERR_IO:
        return "CAN I/O error";
    case CO_ERR_TIMEOUT:
        return "SDO timeout";
    case CO_ERR_BUSY:
        return "SDO busy";
    case CO_ERR_SDO_ABORT:
        return "SDO abort";
    case CO_ERR_SDO_PROTOCOL:
        return "SDO protocol error";
    case CO_ERR_NOT_SUPPORTED:
        return "not supported";
    case CO_ERR_INVALID_ARG:
        return "invalid argument";
    default:
        return "unknown error";
    }
}

const char *canopen_nmt_state_str(co_nmt_state_t state)
{
    switch (state) {
    case CO_NMT_BOOTUP:
        return "Boot-up";
    case CO_NMT_STOPPED:
        return "Stopped";
    case CO_NMT_OPERATIONAL:
        return "Operational";
    case CO_NMT_PREOP:
        return "Pre-Operational";
    default:
        return "Unknown";
    }
}
