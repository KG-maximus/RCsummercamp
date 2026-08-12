#ifndef BSP_CONFIG_H
#define BSP_CONFIG_H

#include "fdcan.h"
#include "usart.h"

/* The chassis CAN transceiver is wired to FDCAN2: PB12 RX, PB13 TX. */
#define CANBUS1_HANDLE       (&hfdcan2)

#define DEBUG_UART_HANDLE    (&huart1)
#define DEBUG_UART_INSTANCE  USART1

#endif
