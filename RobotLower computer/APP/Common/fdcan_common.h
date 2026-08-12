#ifndef FDCAN_COMMON_H
#define FDCAN_COMMON_H

#include <stdint.h>

#include "fdcan.h"

void FDCANSendStandard(
    FDCAN_HandleTypeDef *fdcan_handle,
    uint16_t std_id,
    const uint8_t *data,
    uint8_t length);

void FDCANStandardInit(
    FDCAN_HandleTypeDef *fdcan_handle,
    uint16_t start_id,
    uint16_t end_id);

#endif
