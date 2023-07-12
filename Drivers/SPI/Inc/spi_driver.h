#ifndef SPI_DRIVER_H
#define SPI_DRIVER_H

#include "arm_math_types.h"
#include "main.h"

#define SPI_TIMEOUT 100000

void send_SPI_message(uint8_t *, uint16_t, SPI_HandleTypeDef*);

#endif