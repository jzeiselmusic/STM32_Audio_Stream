#ifndef SPI_DRIVER_H
#define SPI_DRIVER_H

#include "arm_math_types.h"
#include "main.h"

#define SPI_TIMEOUT 100000

void send_SPI_message(uint8_t *data, uint16_t size);

void set_spi_handle(SPI_HandleTypeDef*handlename);

SPI_HandleTypeDef* get_spi_handle(void);

#endif