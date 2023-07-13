#ifndef UART_COMM_H
#define UART_COMM_H

#include "main.h"

UART_HandleTypeDef* get_uart_handle(void);
void set_uart_handle(UART_HandleTypeDef*);
void send_uart_message(const uint8_t*, uint16_t, uint32_t);

#endif