#ifndef APP_INIT_H
#define APP_INIT_H

#include "main.h"

#define SPI_TIMEOUT 100000

void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_DMA_Init(void);
void MX_I2S2_Init(I2S_HandleTypeDef*);
void MX_UART4_Init(UART_HandleTypeDef*);
void MX_SPI1_Init(SPI_HandleTypeDef*);

void send_SPI_message(uint8_t *, uint16_t, SPI_HandleTypeDef*);

#endif