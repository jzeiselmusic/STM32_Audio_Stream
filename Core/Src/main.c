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
#include "spi_driver.h"
#include "oled.h"

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

/* handles for peripherals */
I2S_HandleTypeDef hi2s2;
SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart4;
DMA_HandleTypeDef hdma_i2s2_ext_rx;
DMA_HandleTypeDef hdma_spi2_tx;
osThreadId LEDScreenTaskHandle;

/* buffers to be used by DMA for rx and tx from I2S */
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
  /* gets called when first half of DMA buffer is filled */
	char side = 0x00;
  /* process first half of DMA buffer */
	Process_Data(&side);
}
void HAL_I2SEx_TxRxCpltCallback(I2S_HandleTypeDef *hi2s) {
  /* gets called when second half of DMA buffer is filled */
	char side = 0x01;
  /* process second half of DMA buffer */
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

  /* use the current most recent value to update the rolling audio average */
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

