#ifndef APP_INIT_H
#define APP_INIT_H

#include "main.h"

void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_DMA_Init(void);
void MX_I2S2_Init(I2S_HandleTypeDef*);
void MX_UART4_Init(UART_HandleTypeDef*);
void MX_SPI1_Init(SPI_HandleTypeDef*);

#endif