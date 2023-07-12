#include "spi_driver.h"
#include "main.h"

void send_SPI_message(uint8_t *pData, uint16_t Size, SPI_HandleTypeDef* hspi1) {
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET); // set NSS to low
	HAL_SPI_Transmit(hspi1, pData, Size, SPI_TIMEOUT);

	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);
}