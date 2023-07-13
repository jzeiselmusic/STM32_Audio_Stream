#ifndef SPI_DRIVER_H
#define SPI_DRIVER_H

#include "arm_math_types.h"
#include "main.h"

#define SPI_TIMEOUT 100000

void send_SPI_message(uint8_t *, uint16_t);
void set_spi_handle(SPI_HandleTypeDef*);
SPI_HandleTypeDef* get_spi_handle(void);

#endif