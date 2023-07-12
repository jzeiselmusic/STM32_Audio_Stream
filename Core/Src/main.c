


/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* user implemented complex number library */
#include "complex_numbers.h"

/* freertos includes */
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "cmsis_os.h"

/* int types include */
#include "arm_math_types.h"

/* average volume and fourier transform includes */
#include "fourier.h"
#include "return_vals.h"

/* linear filtering includes */
#include "lpf.h"

/* initializing peripheral drivers includes */
#include "app_init.h"

/* Private defines ----------------------------------------------------------*/

/* size of audio i/o ring buffer */
#include "BUFF_SIZE.h"

/* function prototypes -----------------------------------------------------*/
void start_LED_screen(void);
void turnoff_LED_screen(void);
void clear_LED_screen(void);
void draw_Rectangle(uint8_t, uint8_t, uint8_t, uint8_t);
void LEDScreenTask(void const *);
void TickDelayTask(void const *);
void Process_Data(char *);

/* Private variables ---------------------------------------------------------*/
I2S_HandleTypeDef hi2s2;
SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart4;

osThreadId LEDScreenTaskHandle;

uint16_t rx_buf[16];
uint16_t tx_buf[16];

/* total average volume value. 
   volatile because being affected by multiple functions 
   static because not leaving this compilation unit */
static volatile uint32_t avg = 0;

int main(void)
{

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2S2_Init(&hi2s2);
  MX_UART4_Init(&huart4);
  MX_SPI1_Init(&hspi1);

  HAL_I2SEx_TransmitReceive_DMA(&hi2s2, tx_buf, rx_buf, 8);

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(LEDTask, LEDScreenTask, osPriorityNormal, 0, 128*3);
  LEDScreenTaskHandle = osThreadCreate(osThread(LEDTask), NULL);

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  }
  /* USER CODE END 3 */
}

void HAL_I2SEx_TxRxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {
	char side = 0x00;
	Process_Data(&side);
}
void HAL_I2SEx_TxRxCpltCallback(I2S_HandleTypeDef *hi2s) {
	char side = 0x01;
	Process_Data(&side);
}

void Process_Data(char *side) {
	int start = 0;

	if (*side == 0x01) {
		start += 8;
	}

	int left_in_1 = (((int)rx_buf[start]<<16)|rx_buf[start+1])>>8;
	int right_in_1 = (((int)rx_buf[start+2]<<16)|rx_buf[start+3])>>8;

	int left_in_2 = (((int)rx_buf[start+4]<<16)|rx_buf[start+5])>>8;
	int right_in_2 = (((int)rx_buf[start+6]<<16)|rx_buf[start+7])>>8;

	// left in and right in are both 16 bit integers converted to 24 bit ints
	// we can do processing on them here, as long as it is
	// done in time for the buffer to be passed on by the DMA unit

	avg = update_and_calculate_average((uint32_t)my_abs(left_in_1)); // calculate average of last LIST_LEN samples

	int left_out_1 = left_in_1;
	int right_out_1 = right_in_1;
	int left_out_2 = left_in_2;
	int right_out_2 = right_in_2;

	tx_buf[start] = (left_out_1>>8) & 0xFFFF;
	tx_buf[start+1] = left_out_1 & 0xFFFF;
	tx_buf[start+2] = (right_out_1>>8) & 0xFFFF;
	tx_buf[start+3] = right_out_1 & 0xFFFF;
	tx_buf[start+4] = (left_out_2>>8) & 0xFFFF;
	tx_buf[start+5] = left_out_2 & 0xFFFF;
	tx_buf[start+6] = (right_out_2>>8) & 0xFFFF;
	tx_buf[start+7] = right_out_2 & 0xFFFF;
}

void LEDScreenTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  start_LED_screen();
  uint8_t start_column = 0x08;
  uint8_t start_row = 0x01;
  uint8_t end_column = 0x0E;
  uint8_t end_row = 0x06;

  //uint8_t row_ender;
  uint8_t row_enders[8];

  for (uint8_t m = 0; (start_column + m) < 95; m += 8) {
	  for (uint8_t n = 0; (start_row + n) < 63; n += 8) {
		  draw_Rectangle(start_column + m, start_row + n, end_column + m, end_row + n);

	  }
  }

  osDelay(1000);
  //clear_LED_screen();
  /* Infinite loop */
  GPIO_PinState val, val2;
  for(;;)
  {
	osDelay(50);

	clear_LED_screen();
	osDelay(2);
	avg = low_pass_filter((float32_t)avg);
	calculate_FFT();
	for (int i = 0; i < 8; i++) {
		if (return_vals[i] < 1000000) {
			row_enders[i] = 7;
		} else if (return_vals[i] < 3000000) {
			row_enders[i] = 15;
		} else if (return_vals[i] < 4000000) {
			row_enders[i] = 23;
		} else if (return_vals[i] < 5000000) {
			row_enders[i] = 31;
		} else if (return_vals[i] < 6000000) {
			row_enders[i] = 39;
		} else if (return_vals[i] < 7000000) {
			row_enders[i] = 47;
		} else {
			row_enders[i] = 55;
		}
	}

	int iterator = 7;
	for (uint8_t m = 16; (start_column + m) < 87; m += 8) {
		  for (uint8_t n = 0; (start_row + n) < row_enders[iterator]; n += 8) {
			  draw_Rectangle(start_column + m, start_row + n, end_column + m, end_row + n);

		  }
		  iterator--;
	 }


    val = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4);
    val2 = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3);
    if (val == GPIO_PIN_RESET) {
    	clear_LED_screen();
    	vTaskDelete(NULL);
    } else {
    	if (val2 == GPIO_PIN_RESET) {
    		end_row++;
    		end_column++;
    		clear_LED_screen();
    		osDelay(100);
    		draw_Rectangle(start_row, start_column, end_row, end_column);
    		osDelay(100);
    	}
    }
  }
  /* USER CODE END 5 */
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
  send_SPI_message(pData, 2, &hspi1);
  uint8_t pData1[1] = {0xAE};
  send_SPI_message(pData1, 1, &hspi1);
  uint8_t pData2[2] = {0xA0, 0x72};
  send_SPI_message(pData2, 2, &hspi1);
  uint8_t pData3[2] = {0xA1, 0x00};
  send_SPI_message(pData3, 2, &hspi1);
  uint8_t pData4[2] = {0xA2, 0x00};
  send_SPI_message(pData4, 2, &hspi1);
  uint8_t pData5[1] = {0xA5}; 	// turn the screen white instead of black
  send_SPI_message(pData5, 1, &hspi1);
  uint8_t pData6[2] = {0xA8, 0x3F};
  send_SPI_message(pData6, 2, &hspi1);
  uint8_t pData7[2] = {0xAD, 0x8E};
  send_SPI_message(pData7, 2, &hspi1);
  uint8_t pData8[2] = {0xB0, 0x0B};
  send_SPI_message(pData8, 2, &hspi1);
  uint8_t pData9[2] = {0xB1, 0x31};
  send_SPI_message(pData9, 2, &hspi1);
  uint8_t pData10[2] = {0xB3, 0xF0};
  send_SPI_message(pData10, 2, &hspi1);
  uint8_t pData11[2] = {0x8A, 0x64};
  send_SPI_message(pData11, 2, &hspi1);
  uint8_t pData12[2] = {0x8B, 0x78};
  send_SPI_message(pData12, 2, &hspi1);
  uint8_t pData13[2] = {0x8C, 0x64};
  send_SPI_message(pData13, 2, &hspi1);
  uint8_t pData14[2] = {0xBB, 0x3A};
  send_SPI_message(pData14, 2, &hspi1);
  uint8_t pData15[2] = {0xBE, 0x3E};
  send_SPI_message(pData15, 2, &hspi1);
  uint8_t pData16[2] = {0x87, 0x06};
  send_SPI_message(pData16, 2, &hspi1);
  uint8_t pData17[2] = {0x81, 0x91};
  send_SPI_message(pData17, 2, &hspi1);
  uint8_t pData18[2] = {0x82, 0x50};
  send_SPI_message(pData18, 2, &hspi1);
  uint8_t pData19[2] = {0x83, 0x7D};
  send_SPI_message(pData19, 2, &hspi1);
  uint8_t pData20[1] = {0x2E};
  send_SPI_message(pData20, 1, &hspi1);
  uint8_t pData21[5] = {0x25, 0x00, 0x00, 0x5F, 0x3F};
  send_SPI_message(pData21, 5, &hspi1);

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET); // set VCCen back to high
  osDelay(25);

  uint8_t pData22[1] = {0xAF};
  send_SPI_message(pData22, 1, &hspi1);
  osDelay(1000);

  uint8_t turn_black[1] = {0xA4};
  send_SPI_message(turn_black, 1, &hspi1);

}

void turnoff_LED_screen(void) {
	uint8_t pData23[1] = {0xAE};
	send_SPI_message(pData23, 1, &hspi1);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
	osDelay(400);
}

void draw_Rectangle(uint8_t start_column, uint8_t start_row,
					uint8_t end_column, uint8_t end_row) {
	// draw a rectangle command
	uint8_t pDataDrawRect[1] = {0x22};
	send_SPI_message(pDataDrawRect, 1, &hspi1);
	uint8_t pData24[1] = {start_column};
	send_SPI_message(pData24, 1, &hspi1);
	uint8_t pData25[1] = {start_row};
	send_SPI_message(pData25, 1, &hspi1);
	uint8_t pData26[1] = {end_column};
	send_SPI_message(pData26, 1, &hspi1);
	uint8_t pData27[1] = {end_row};
	send_SPI_message(pData27, 1, &hspi1);
	uint8_t color1[1] = {0x28};
	send_SPI_message(color1, 1, &hspi1);
	uint8_t color2[1] = {0x10};
	send_SPI_message(color2, 1, &hspi1);
	uint8_t color3[1] = {0x00};
	send_SPI_message(color3, 1, &hspi1);
	uint8_t color4[1] = {0x28};
	send_SPI_message(color4, 1, &hspi1);
	uint8_t color5[1] = {0x00};
	send_SPI_message(color5, 1, &hspi1);
	uint8_t color6[1] = {0x00};
	send_SPI_message(color6, 1, &hspi1);
}

void clear_LED_screen(void) {
	// clear screen send 0x25, 0x00, 0x00, 0x5f, 0x3f
	uint8_t clear_screen[1] = {0x25};
	uint8_t data1[1] = {0x00};
	uint8_t data2[1] = {0x5f};
	uint8_t data3[1] = {0x3f};
	send_SPI_message(clear_screen, 1, &hspi1);
	send_SPI_message(data1, 1, &hspi1);
	send_SPI_message(data1, 1, &hspi1);
	send_SPI_message(data2, 1, &hspi1);
	send_SPI_message(data3, 1, &hspi1);
}





void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

