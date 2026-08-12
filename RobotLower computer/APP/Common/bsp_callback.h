#ifndef BSP_CALLBACK_H
#define BSP_CALLBACK_H

#include <stdint.h>

#include "fdcan.h"

typedef void (*BSPCallback_FDCANRxHandler_t)(
    FDCAN_HandleTypeDef *fdcan_handle,
    uint32_t std_id,
    const uint8_t data[8]);

/*
 * Register a consumer of standard-ID frames received by FDCAN FIFO0.
 * Registration is intended for module initialization, before the scheduler
 * starts. HAL_FDCAN_RxFifo0Callback is defined only in bsp_callback.c.
 */
uint8_t BSPCallback_RegisterFDCANRxHandler(
    BSPCallback_FDCANRxHandler_t handler);

#endif
