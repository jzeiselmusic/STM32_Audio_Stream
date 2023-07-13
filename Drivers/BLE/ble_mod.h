#ifndef BLE_MOD_DRIVER_H
#define BLE_MOD_DRIVER_H

#include "main.h"

/* driver header for the PMOD BLE peripheral module */
/* the module communicates with the host through UART commands */

HAL_StatusTypeDef enter_command_mode(void);

#endif