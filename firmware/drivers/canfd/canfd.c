#include "canfd.h"
#include "board.h"

#include <string.h>

static canfd_bus_t *g_bus_by_id[BOARD_FDCAN_COUNT];

static uint32_t canfd_len_to_dlc(uint8_t len)
{
    if (len <= 8U) {
        return (uint32_t)len;
    }
    if (len <= 12U) {
        return FDCAN_DLC_BYTES_12;
    }
    if (len <= 16U) {
        return FDCAN_DLC_BYTES_16;
    }
    if (len <= 20U) {
        return FDCAN_DLC_BYTES_20;
    }
    if (len <= 24U) {
        return FDCAN_DLC_BYTES_24;
    }
    if (len <= 32U) {
        return FDCAN_DLC_BYTES_32;
    }
    if (len <= 48U) {
        return FDCAN_DLC_BYTES_48;
    }
    return FDCAN_DLC_BYTES_64;
}

static uint8_t canfd_dlc_to_len(uint32_t dlc)
{
    switch (dlc) {
    case FDCAN_DLC_BYTES_12:
        return 12U;
    case FDCAN_DLC_BYTES_16:
        return 16U;
    case FDCAN_DLC_BYTES_20:
        return 20U;
    case FDCAN_DLC_BYTES_24:
        return 24U;
    case FDCAN_DLC_BYTES_32:
        return 32U;
    case FDCAN_DLC_BYTES_48:
        return 48U;
    case FDCAN_DLC_BYTES_64:
        return 64U;
    default:
        return (uint8_t)dlc;
    }
}

static board_fdcan_id_t canfd_id_from_handle(FDCAN_HandleTypeDef *hfdcan)
{
    if (hfdcan->Instance == FDCAN1) {
        return BOARD_FDCAN_1;
    }
    if (hfdcan->Instance == FDCAN2) {
        return BOARD_FDCAN_2;
    }
    return BOARD_FDCAN_3;
}

HAL_StatusTypeDef canfd_bus_init(canfd_bus_t *bus, board_fdcan_id_t id, bool fd_capable)
{
    uint32_t message_ram_offset;

    if (bus == 0) {
        return HAL_ERROR;
    }

    bus->hfdcan = board_fdcan_handle(id);
    if (bus->hfdcan == 0) {
        return HAL_ERROR;
    }

    bus->id = id;
    bus->fd_capable = fd_capable;
    bus->started = false;
    bus->rx_head = 0U;
    bus->rx_tail = 0U;
    bus->rx_frames = 0U;
    bus->tx_frames = 0U;
    bus->rx_dropped = 0U;
    bus->tx_dropped = 0U;
    bus->rx_overrun = 0U;

    switch (id) {
    case BOARD_FDCAN_1:
        message_ram_offset = 0x000U;
        break;
    case BOARD_FDCAN_2:
        message_ram_offset = 0x300U;
        break;
    default:
        message_ram_offset = 0x600U;
        break;
    }

    g_bus_by_id[id] = bus;

    return board_fdcan_init(id, message_ram_offset, fd_capable);
}

HAL_StatusTypeDef canfd_bus_start(canfd_bus_t *bus)
{
    HAL_StatusTypeDef status;
    if ((bus == 0) || (bus->hfdcan == 0)) {
        return HAL_ERROR;
    }
    status = board_fdcan_start(bus->id);
    if (status == HAL_OK) {
        bus->started = true;
    }
    return status;
}

HAL_StatusTypeDef canfd_bus_stop(canfd_bus_t *bus)
{
    if ((bus == 0) || (bus->hfdcan == 0)) {
        return HAL_ERROR;
    }
    bus->started = false;
    return board_fdcan_stop(bus->id);
}

HAL_StatusTypeDef canfd_send(canfd_bus_t *bus,
                             uint32_t id,
                             const uint8_t *data,
                             uint8_t len,
                             bool fd,
                             uint32_t timeout_ms)
{
    return canfd_send_ex(bus, id, data, len, fd, fd, timeout_ms);
}

HAL_StatusTypeDef canfd_send_ex(canfd_bus_t *bus,
                                uint32_t id,
                                const uint8_t *data,
                                uint8_t len,
                                bool fd,
                                bool brs,
                                uint32_t timeout_ms)
{
    FDCAN_TxHeaderTypeDef header = {0};
    uint32_t start_ms;
    uint8_t tx_buf[CANFD_MAX_DATA] = {0};

    if ((bus == 0) || (bus->hfdcan == 0) || (data == 0) || (len > CANFD_MAX_DATA)) {
        return HAL_ERROR;
    }
    if (fd && !bus->fd_capable) {
        return HAL_ERROR;
    }

    memcpy(tx_buf, data, len);

    start_ms = board_millis();
    while (HAL_FDCAN_GetTxFifoFreeLevel(bus->hfdcan) == 0U) {
        if ((board_millis() - start_ms) >= timeout_ms) {
            bus->tx_dropped++;
            return HAL_TIMEOUT;
        }
    }

    header.Identifier = id;
    header.IdType = FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = canfd_len_to_dlc(len);
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = brs ? FDCAN_BRS_ON : FDCAN_BRS_OFF;
    header.FDFormat = fd ? FDCAN_FD_CAN : FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    header.MessageMarker = 0U;

    if (HAL_FDCAN_AddMessageToTxFifoQ(bus->hfdcan, &header, tx_buf) != HAL_OK) {
        bus->tx_dropped++;
        return HAL_ERROR;
    }

    bus->tx_frames++;
    return HAL_OK;
}

void canfd_get_status(canfd_bus_t *bus, canfd_status_t *status)
{
    FDCAN_ErrorCountersTypeDef counters;
    FDCAN_ProtocolStatusTypeDef protocol;

    if ((bus == 0) || (status == 0) || (bus->hfdcan == 0)) {
        return;
    }

    (void)HAL_FDCAN_GetErrorCounters(bus->hfdcan, &counters);
    (void)HAL_FDCAN_GetProtocolStatus(bus->hfdcan, &protocol);

    status->tx_error_count = (uint8_t)counters.TxErrorCnt;
    status->rx_error_count = (uint8_t)counters.RxErrorCnt;
    status->last_error_code = (uint8_t)protocol.LastErrorCode;
    status->error_passive = (protocol.ErrorPassive != 0U);
    status->bus_off = (protocol.BusOff != 0U);
}

HAL_StatusTypeDef canfd_recover_bus_off(canfd_bus_t *bus)
{
    if ((bus == 0) || (bus->hfdcan == 0)) {
        return HAL_ERROR;
    }

    (void)HAL_FDCAN_Stop(bus->hfdcan);
    bus->rx_head = 0U;
    bus->rx_tail = 0U;
    return HAL_FDCAN_Start(bus->hfdcan);
}

bool canfd_receive(canfd_bus_t *bus, canfd_frame_t *frame)
{
    uint16_t tail;

    if ((bus == 0) || (frame == 0)) {
        return false;
    }

    tail = bus->rx_tail;
    if (tail == bus->rx_head) {
        return false;
    }

    *frame = bus->rx_ring[tail];
    __DMB();
    bus->rx_tail = (uint16_t)((tail + 1U) % CANFD_RX_RING_SIZE);
    return true;
}

void canfd_irq_rx(FDCAN_HandleTypeDef *hfdcan)
{
    board_fdcan_id_t id = canfd_id_from_handle(hfdcan);
    canfd_bus_t *bus = g_bus_by_id[id];
    FDCAN_RxHeaderTypeDef header;
    canfd_frame_t frame;

    if (bus == 0) {
        return;
    }

    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U) {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0,
                                   &header, frame.data) != HAL_OK) {
            break;
        }

        frame.id = header.Identifier;
        frame.len = canfd_dlc_to_len(header.DataLength);
        frame.fd = (header.FDFormat == FDCAN_FD_CAN);
        frame.brs = (header.BitRateSwitch == FDCAN_BRS_ON);

        if (frame.len > CANFD_MAX_DATA) {
            frame.len = CANFD_MAX_DATA;
        }

        {
            uint16_t head = bus->rx_head;
            uint16_t next = (uint16_t)((head + 1U) % CANFD_RX_RING_SIZE);

            if (next == bus->rx_tail) {
                bus->rx_dropped++;
                bus->rx_overrun++;
            } else {
                bus->rx_ring[head] = frame;
                __DMB();
                bus->rx_head = next;
                bus->rx_frames++;
            }
        }
    }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0U) {
        canfd_irq_rx(hfdcan);
    }
}
