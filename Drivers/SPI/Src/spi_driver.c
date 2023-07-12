#include "spi_driver.h"
#include "main.h"

static SPI_HandleTypeDef* perpetual_spi_handle;

void set_spi_handle(SPI_HandleTypeDef* spihandle)
{
    perpetual_spi_handle = spihandle;
}

SPI_HandleTypeDef* get_spi_handle(void)
{
    return perpetual_spi_handle;
}

void send_SPI_message(uint8_t *pData, uint16_t Size, SPI_HandleTypeDef* hspi1) {
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET); // set NSS to low
	HAL_SPI_Transmit(hspi1, pData, Size, SPI_TIMEOUT);

	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);
}