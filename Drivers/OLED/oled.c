#include "oled.h"
#include "main.h"
#include "spi_driver.h"
#include "cmsis_os.h"

static uint8_t color_red[3]   = {0xff, 0x00, 0x00};
static uint8_t color_green[3] = {0x00, 0xff, 0x00};
static uint8_t color_blue[3]  = {0x00, 0x00, 0xff};

static uint8_t* oled_color = color_red;

void set_color(TypeDef_OLED_Color choice) 
{
  switch(choice) 
  {
    case COLOR_RED:   oled_color = color_red;
    case COLOR_GREEN: oled_color = color_green;
    case COLOR_BLUE:  oled_color = color_blue;
  }

}

void start_LED_screen(void) {
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET); // set D/C to low always
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET); // bring VCCen low
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET); // set NSS to high

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET); // set RESET to high
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET); // set PMOD_EN to high
  osDelay(20); // wait 20 ms
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET); // set RESET to low
  osDelay(1); // wait 1 ms
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET); // set RESET back to high
  osDelay(1);

  uint8_t pData[2] = {0xFD, 0x12};
  send_SPI_message(pData, 2);
  uint8_t pData1[1] = {0xAE};
  send_SPI_message(pData1, 1);
  uint8_t pData2[2] = {0xA0, 0x72};
  send_SPI_message(pData2, 2);
  uint8_t pData3[2] = {0xA1, 0x00};
  send_SPI_message(pData3, 2);
  uint8_t pData4[2] = {0xA2, 0x00};
  send_SPI_message(pData4, 2);
  uint8_t pData5[1] = {0xA5}; 	// turn the screen white instead of black
  send_SPI_message(pData5, 1);
  uint8_t pData6[2] = {0xA8, 0x3F};
  send_SPI_message(pData6, 2);
  uint8_t pData7[2] = {0xAD, 0x8E};
  send_SPI_message(pData7, 2);
  uint8_t pData8[2] = {0xB0, 0x0B};
  send_SPI_message(pData8, 2);
  uint8_t pData9[2] = {0xB1, 0x31};
  send_SPI_message(pData9, 2);
  uint8_t pData10[2] = {0xB3, 0xF0};
  send_SPI_message(pData10, 2);
  uint8_t pData11[2] = {0x8A, 0x64};
  send_SPI_message(pData11, 2);
  uint8_t pData12[2] = {0x8B, 0x78};
  send_SPI_message(pData12, 2);
  uint8_t pData13[2] = {0x8C, 0x64};
  send_SPI_message(pData13, 2);
  uint8_t pData14[2] = {0xBB, 0x3A};
  send_SPI_message(pData14, 2);
  uint8_t pData15[2] = {0xBE, 0x3E};
  send_SPI_message(pData15, 2);
  uint8_t pData16[2] = {0x87, 0x06};
  send_SPI_message(pData16, 2);
  uint8_t pData17[2] = {0x81, 0x91};
  send_SPI_message(pData17, 2);
  uint8_t pData18[2] = {0x82, 0x50};
  send_SPI_message(pData18, 2);
  uint8_t pData19[2] = {0x83, 0x7D};
  send_SPI_message(pData19, 2);
  uint8_t pData20[1] = {0x2E};
  send_SPI_message(pData20, 1);
  uint8_t pData21[5] = {0x25, 0x00, 0x00, 0x5F, 0x3F};
  send_SPI_message(pData21, 5);

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET); // set VCCen back to high
  osDelay(25);

  uint8_t pData22[1] = {0xAF};
  send_SPI_message(pData22, 1);
  osDelay(1000);

  uint8_t turn_black[1] = {0xA4};
  send_SPI_message(turn_black, 1);

}


void turnoff_LED_screen(void) {
	uint8_t pData23[1] = {0xAE};
	send_SPI_message(pData23, 1);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
	osDelay(400);
}

void draw_Rectangle(uint8_t start_column, uint8_t start_row,
					uint8_t end_column, uint8_t end_row)
{
	// draw a rectangle command
	uint8_t pDataDrawRect[1] = {0x22};
	send_SPI_message(pDataDrawRect, 1);
	uint8_t pData24[1] = {start_column};
	send_SPI_message(pData24, 1);
	uint8_t pData25[1] = {start_row};
	send_SPI_message(pData25, 1);
	uint8_t pData26[1] = {end_column};
	send_SPI_message(pData26, 1);
	uint8_t pData27[1] = {end_row};
	send_SPI_message(pData27, 1);
	uint8_t color1[1] = {oled_color[0]};
	send_SPI_message(color1, 1);
	uint8_t color2[1] = {oled_color[1]};
	send_SPI_message(color2, 1);
	uint8_t color3[1] = {oled_color[2]};
	send_SPI_message(color3, 1);
	uint8_t color4[1] = {0x00};
	send_SPI_message(color4, 1);
	uint8_t color5[1] = {0x00};
	send_SPI_message(color5, 1);
	uint8_t color6[1] = {0x00};
	send_SPI_message(color6, 1);
}

void clear_LED_screen(void) {
	// clear screen send 0x25, 0x00, 0x00, 0x5f, 0x3f
	uint8_t clear_screen[1] = {0x25};
	uint8_t data1[1] = {0x00};
	uint8_t data2[1] = {0x5f};
	uint8_t data3[1] = {0x3f};
	send_SPI_message(clear_screen, 1);
	send_SPI_message(data1, 1);
	send_SPI_message(data1, 1);
	send_SPI_message(data2, 1);
	send_SPI_message(data3, 1);
}