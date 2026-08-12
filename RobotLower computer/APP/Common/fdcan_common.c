#include "fdcan_common.h"

void FDCANStandardInit(
    FDCAN_HandleTypeDef *fdcan_handle,
    uint16_t start_id,
    uint16_t end_id)
{
    FDCAN_FilterTypeDef filter = {0};

    if ((fdcan_handle == 0) || (start_id > end_id))
    {
        return;
    }

    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0U;
    filter.FilterType = FDCAN_FILTER_RANGE;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = start_id;
    filter.FilterID2 = end_id;

    if (HAL_FDCAN_ConfigFilter(fdcan_handle, &filter) != HAL_OK)
    {
        return;
    }
    if (HAL_FDCAN_ActivateNotification(
            fdcan_handle,
            FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
            0U) != HAL_OK)
    {
        return;
    }
    (void)HAL_FDCAN_Start(fdcan_handle);
}

void FDCANSendStandard(
    FDCAN_HandleTypeDef *fdcan_handle,
    uint16_t std_id,
    const uint8_t *data,
    uint8_t length)
{
    FDCAN_TxHeaderTypeDef tx_header = {0};

    if ((fdcan_handle == 0) || (data == 0) || (length > 8U))
    {
        return;
    }

    tx_header.Identifier = std_id;
    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    switch (length)
    {
        case 0U:
            tx_header.DataLength = FDCAN_DLC_BYTES_0;
            break;
        case 1U:
            tx_header.DataLength = FDCAN_DLC_BYTES_1;
            break;
        case 2U:
            tx_header.DataLength = FDCAN_DLC_BYTES_2;
            break;
        case 3U:
            tx_header.DataLength = FDCAN_DLC_BYTES_3;
            break;
        case 4U:
            tx_header.DataLength = FDCAN_DLC_BYTES_4;
            break;
        case 5U:
            tx_header.DataLength = FDCAN_DLC_BYTES_5;
            break;
        case 6U:
            tx_header.DataLength = FDCAN_DLC_BYTES_6;
            break;
        case 7U:
            tx_header.DataLength = FDCAN_DLC_BYTES_7;
            break;
        default:
            tx_header.DataLength = FDCAN_DLC_BYTES_8;
            break;
    }
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0U;

    (void)HAL_FDCAN_AddMessageToTxFifoQ(fdcan_handle, &tx_header, (uint8_t *)data);
}
