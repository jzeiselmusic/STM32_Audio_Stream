#include "uart_comm.h"
#include "main.h"

static UART_HandleTypeDef* perpetual_uart_handle;
/* set set flag if uart has been initialized */
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

/* blocking UART Send arbitrary amount of data */
void send_uart_message(const uint8_t* data, uint16_t size) 
{
    if (set_flag) {
        HAL_UART_Transmit(perpetual_uart_handle, data, size, 1000);
    }
}

/* blocking UART receive a single character */
uint8_t read_uart_char(void)
{
    if (set_flag)
    {
        uint8_t buffer;
        HAL_UART_Receive(perpetual_uart_handle, &buffer, 1, 1000);
        return buffer;
    }
    else 
    {
        return (uint8_t)(NULL);
    }
}

/* blocking UART receive until receive a carriage return */
void read_uart_line(uint8_t* buffer, uint16_t size)
{
    if (set_flag)
    {
        int n = 0;
        uint8_t val;
        do
        {
            val = read_uart_char();
            buffer[n] = val;
            n += 1;
        } while (n <= size && val != '\r');
    }
}