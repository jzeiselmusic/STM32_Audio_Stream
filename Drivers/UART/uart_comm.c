#include "uart_comm.h"
#include "main.h"

static UART_HandleTypeDef* perpetual_uart_handle;
/* set set flag if uart handle has been set */
static int set_flag = 0;

UART_HandleTypeDef* get_uart_handle(void)
{
    if (set_flag) 
    {
        return perpetual_uart_handle;
    }
    else 
    {
        return (UART_HandleTypeDef*)(NULL);
    }
}

void set_uart_handle(UART_HandleTypeDef* input) 
{
    perpetual_uart_handle = input;
    set_flag = 1;
}

void send_uart_message(const uint8_t* data, uint16_t size, uint32_t timeout) 
{
    HAL_UART_Transmit(perpetual_uart_handle, data, size, timeout);
}