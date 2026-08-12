#include "bsp_callback.h"

#define BSP_CALLBACK_FDCAN_RX_HANDLER_MAX  (4U)

static BSPCallback_FDCANRxHandler_t
    fdcan_rx_handlers[BSP_CALLBACK_FDCAN_RX_HANDLER_MAX];

uint8_t BSPCallback_RegisterFDCANRxHandler(
    BSPCallback_FDCANRxHandler_t handler)
{
    uint32_t index;
    uint32_t primask;

    if (handler == 0)
    {
        return 0U;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    for (index = 0U; index < BSP_CALLBACK_FDCAN_RX_HANDLER_MAX; ++index)
    {
        if (fdcan_rx_handlers[index] == handler)
        {
            __set_PRIMASK(primask);
            return 1U;
        }
    }
    for (index = 0U; index < BSP_CALLBACK_FDCAN_RX_HANDLER_MAX; ++index)
    {
        if (fdcan_rx_handlers[index] == 0)
        {
            fdcan_rx_handlers[index] = handler;
            __set_PRIMASK(primask);
            return 1U;
        }
    }
    __set_PRIMASK(primask);
    return 0U;
}

void HAL_FDCAN_RxFifo0Callback(
    FDCAN_HandleTypeDef *hfdcan,
    uint32_t rx_fifo0_its)
{
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    uint32_t index;

    if ((rx_fifo0_its & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U)
    {
        return;
    }

    /*
     * Keep this ISR bounded and allocation-free: read one frame, then hand it
     * to registered modules. Control loops and other heavy work stay in tasks.
     */
    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U)
    {
        if (HAL_FDCAN_GetRxMessage(
                hfdcan,
                FDCAN_RX_FIFO0,
                &rx_header,
                rx_data) != HAL_OK)
        {
            break;
        }

        if (rx_header.IdType != FDCAN_STANDARD_ID)
        {
            continue;
        }

        for (index = 0U; index < BSP_CALLBACK_FDCAN_RX_HANDLER_MAX; ++index)
        {
            if (fdcan_rx_handlers[index] != 0)
            {
                fdcan_rx_handlers[index](hfdcan, rx_header.Identifier, rx_data);
            }
        }
    }
}
