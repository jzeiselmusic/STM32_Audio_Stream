#ifndef BLE_MOD_DRIVER_H
#define BLE_MOD_DRIVER_H

#include "main.h"

typedef enum 
{
    COMM_LINE_READY,
    COMM_LINE_NOT_READY,
} TypeDef_Command_Line;

/* driver header for the PMOD BLE peripheral module */
/* the module communicates with the host through UART commands */

HAL_StatusTypeDef enter_command_mode(void);

HAL_StatusTypeDef start_advertising(void);

#endif