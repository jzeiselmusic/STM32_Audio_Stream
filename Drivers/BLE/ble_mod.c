#include "ble_mod.h"
#include "uart_comm.h"
#include "cmsis_os.h"

#define MAX_RESPONSE   128

HAL_StatusTypeDef enter_command_mode(void)
{
    /* send enter command mode */
    send_uart_message((uint8_t*)"$", 1);
    osDelay(100);
    send_uart_message((uint8_t*)"$$\r", 3);

    /* look for command prompt */
    uint8_t response[MAX_RESPONSE];
    uint16_t num_read;
    read_uart_line(response, &num_read, (uint16_t)MAX_RESPONSE);

    if (num_read == 5)
    {
        if (response[0] == 'C' && response[1] == 'M' && response[2] == 'D' && response[3] == '>')
        {
            return HAL_OK;
        }
        else 
        {
            return HAL_ERROR;
        }
    }
    else 
    {
        return HAL_ERROR;
    }

}