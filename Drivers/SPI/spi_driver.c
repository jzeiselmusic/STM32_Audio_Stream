#include "spi_driver.h"
#include "main.h"

static SPI_HandleTypeDef* perpetual_spi_handle;
/* set set flag if spi has been initialized */
static int set_flag = 0;

void set_spi_handle(SPI_HandleTypeDef* spihandle)
{
    perpetual_spi_handle = spihandle;
	set_flag = 1;
}

SPI_HandleTypeDef* get_spi_handle(void)
{
	if (set_flag)
	{
    	return perpetual_spi_handle;
	}
	else 
	{
		return (SPI_HandleTypeDef*)(NULL);
	}
}

void send_SPI_message(uint8_t *pData, uint16_t Size) {
	if (set_flag)
	{
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET); // set NSS to low
		HAL_SPI_Transmit(perpetual_spi_handle, pData, Size, SPI_TIMEOUT);

		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);
	}
}