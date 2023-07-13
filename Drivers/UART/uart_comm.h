#ifndef UART_COMM_H
#define UART_COMM_H

#include "main.h"

UART_HandleTypeDef* get_uart_handle(void);

void set_uart_handle(UART_HandleTypeDef* handlename);

void send_uart_message(const uint8_t*data, uint16_t size);

uint8_t read_uart_char(void);

#endif