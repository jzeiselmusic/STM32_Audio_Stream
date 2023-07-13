#include "ble_mod.h"
#include "uart_comm.h"
#include "cmsis_os.h"

#define MAX_RESPONSE   128

static TypeDef_Command_Line is_comm_line_ready = COMM_LINE_NOT_READY;

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
            is_comm_line_ready = COMM_LINE_READY;
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

HAL_StatusTypeDef start_advertising(void)
{
    if (is_comm_line_ready == COMM_LINE_READY)
    {
        /* start advertising for 60 seconds */
        send_uart_message((uint8_t*)"A,0050,005E\r", 12);

        /* check to see status response from BLE module */
        uint8_t response[MAX_RESPONSE];
        uint16_t num_read;
        read_uart_line(response, &num_read, (uint16_t)MAX_RESPONSE);

        if (num_read == 4)
        {
            if (response[0] == 'A' && response[1] == 'O' && response[2] == 'K') 
            {
                return HAL_OK;
            }
            else 
            {
                return HAL_OK;
            }
        }
        else 
        {
            return HAL_OK;
        }
    }
    else 
    {
        return HAL_OK;
    }
}
