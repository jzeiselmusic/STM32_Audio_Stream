


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

/* Private defines ----------------------------------------------------------*/

#define SPI_TIMEOUT 100000
/* size of audio i/o ring buffer */
#include "BUFF_SIZE.h"

/* function prototypes -----------------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2S2_Init(void);
static void MX_UART4_Init(void);
static void MX_SPI1_Init(void);
void send_SPI_message(uint8_t *, uint16_t);
void start_LED_screen(void);
void turnoff_LED_screen(void);
void clear_LED_screen(void);
void draw_Rectangle(uint8_t, uint8_t, uint8_t, uint8_t);
void LEDScreenTask(void const *);
void TickDelayTask(void const *);
void Process_Data(char *);

/* Private variables ---------------------------------------------------------*/
I2S_HandleTypeDef hi2s2;
DMA_HandleTypeDef hdma_i2s2_ext_rx;
DMA_HandleTypeDef hdma_spi2_tx;

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
  MX_I2S2_Init();
  MX_UART4_Init();
  MX_SPI1_Init();

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

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV16;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_I2S2_Init(void)
{

  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_24B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_48K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_ENABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }

}

static void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }

  uint8_t data[1] = {0x00};
  // send initial empty message to initialize values
  send_SPI_message(data, 1);

}

static void MX_UART4_Init(void)
{

  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }

}

static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);

}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);


  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);


  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);


  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // this is the output for NSS
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  // this is the clear screen button K0
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  // this is the grow box button K1
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

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

void send_SPI_message(uint8_t *pData, uint16_t Size) {
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET); // set NSS to low
	HAL_SPI_Transmit(&hspi1, pData, Size, SPI_TIMEOUT);
	//osDelay(1);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);
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
					uint8_t end_column, uint8_t end_row) {
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
	uint8_t color1[1] = {0x28};
	send_SPI_message(color1, 1);
	uint8_t color2[1] = {0x10};
	send_SPI_message(color2, 1);
	uint8_t color3[1] = {0x00};
	send_SPI_message(color3, 1);
	uint8_t color4[1] = {0x28};
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

