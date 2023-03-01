

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <arm_math_types.h>
#include <basic_math_functions.h>
#include <support_functions.h>

/* Private includes ----------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/
I2S_HandleTypeDef hi2s2;
DMA_HandleTypeDef hdma_i2s2_ext_rx;
DMA_HandleTypeDef hdma_spi2_tx;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart4;

osThreadId LEDScreenTaskHandle;
osThreadId TickDelayTaskHandle;
/* USER CODE BEGIN PV */
uint16_t rx_buf[16];
uint16_t tx_buf[16];


float32_t input_list[4];
float32_t output_list[4];

int count = 0;
uint8_t turn_off_flag = 0;

float32_t b[4] = {0.5887, 1.7660, 1.7660, 0.5887};
float32_t a[3] = {1.9630, 1.4000, 0.3464};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2S2_Init(void);
static void MX_UART4_Init(void);
static void MX_SPI1_Init(void);
void LEDScreenTask(void const * argument);
void TickDelayTask(void const * argument);

/* USER CODE BEGIN PFP */
void Process_Data(char *);
float32_t low_pass_filter(float32_t, float32_t);


/* USER CODE END PFP */

int _write(int fd, char *ptr, int len)
{
	HAL_UART_Transmit_DMA(&huart4, (uint8_t*) ptr, len);
	return len;
}


/* Private user code ---------------------------------------------------------*/


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* MCU Configuration--------------------------------------------------------*/

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
  osThreadDef(LEDTask, LEDScreenTask, osPriorityNormal, 0, 128);
  LEDScreenTaskHandle = osThreadCreate(osThread(LEDTask), NULL);

  osThreadDef(DelayTask, TickDelayTask, osPriorityNormal, 0, 128);
  TickDelayTaskHandle = osThreadCreate(osThread(DelayTask), NULL);

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

/**
  * @brief System Clock Configuration
  * @retval None
  */
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
  RCC_OscInitStruct.PLL.PLLN = 168;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2S2 Initialization Function
  * @param None
  * @retval None
  */
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

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
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
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
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

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  //HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 5, 0);
  //HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC4 PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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

	// implement a simple tanh soft distortion mechanism

	float32_t float_left_in_1 = (float32_t)left_in_1;
	float32_t float_right_in_1 = (float32_t)right_in_1;
	float32_t float_left_in_2 = (float32_t)left_in_2;
	float32_t float_right_in_2 = (float32_t)right_in_2;

	//float_left_in_1 = low_pass_filter(float_left_in_1, float_right_in_1);
	//float_left_in_2 = low_pass_filter(float_left_in_2, float_right_in_2);

	//float_right_in_1 = float_left_in_1;
	//float_right_in_2 = float_left_in_2;

	int left_out_1 = (int)float_left_in_1;
	int right_out_1 = (int)float_right_in_1;
	int left_out_2 = (int)float_left_in_2;
	int right_out_2 = (int)float_right_in_2;

	tx_buf[start] = (left_out_1>>8) & 0xFFFF;
	tx_buf[start+1] = left_out_1 & 0xFFFF;
	tx_buf[start+2] = (right_out_1>>8) & 0xFFFF;
	tx_buf[start+3] = right_out_1 & 0xFFFF;
	tx_buf[start+4] = (left_out_2>>8) & 0xFFFF;
	tx_buf[start+5] = left_out_2 & 0xFFFF;
	tx_buf[start+6] = (right_out_2>>8) & 0xFFFF;
	tx_buf[start+7] = right_out_2 & 0xFFFF;
}



float32_t low_pass_filter(float32_t left, float32_t right) {

	float32_t return_value;
	input_list[3] = input_list[2];
	input_list[2] = input_list[1];
	input_list[1] = input_list[0];
	input_list[0] = left;

	output_list[3] = output_list[2];
	output_list[2] = output_list[1];
	output_list[1] = output_list[0];

  float32_t temp_output_list[3] = {output_list[1], output_list[2], output_list[3]};

	if (count < 4) {
		output_list[0] = left;
		return_value = output_list[0];
	}
	else {
    float32_t result_fir;
    float32_t result_iir;
    arm_dot_prod_f32(input_list, b, 4, &result_fir);
    arm_dot_prod_f32(temp_output_list, a, 3, &result_iir);
    output_list[0] = result_fir - result_iir;
    return_value = output_list[0];
    /*
		output_list[0] = input_list[0]*b0 + input_list[1]*b1 + input_list[2]*b2
						+ input_list[3]*b3 - output_list[1]*a1 - output_list[2]*a2
						- output_list[3]*a3;
		return_value = output_list[0];
    */
	}

	count++;
	if (count > 15) {
		count = 15;
	}
	return return_value;
}




void TickDelayTask(void const * argument) {
	osDelay(20000);
	turn_off_flag = 1;
	vTaskDelete(NULL);
}







void LEDScreenTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET); // set RESET to high
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET); // set PMOD_EN to high
  osDelay(20); // wait 20 ms
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET); // set RESET to low
  osDelay(1); // wait 1 ms
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET); // set RESET back to high
  osDelay(1);

  uint8_t pData[2] = {0xFD, 0x12};
  HAL_SPI_Transmit(&hspi1, pData, 2, 1000);
  uint8_t pData1[1] = {0xAE};
  HAL_SPI_Transmit(&hspi1, pData1, 1, 1000);
  uint8_t pData2[2] = {0xA0, 0x72};
  HAL_SPI_Transmit(&hspi1, pData2, 2, 1000);
  uint8_t pData3[2] = {0xA1, 0x00};
  HAL_SPI_Transmit(&hspi1, pData3, 2, 1000);
  uint8_t pData4[2] = {0xA2, 0x00};
  HAL_SPI_Transmit(&hspi1, pData4, 2, 1000);
  uint8_t pData5[1] = {0xA4};
  HAL_SPI_Transmit(&hspi1, pData5, 1, 1000);
  uint8_t pData6[2] = {0xA8, 0x3F};
  HAL_SPI_Transmit(&hspi1, pData6, 2, 1000);
  uint8_t pData7[2] = {0xAD, 0x8E};
  HAL_SPI_Transmit(&hspi1, pData7, 2, 1000);
  uint8_t pData8[2] = {0xB0, 0x0B};
  HAL_SPI_Transmit(&hspi1, pData8, 2, 1000);
  uint8_t pData9[2] = {0xB1, 0x31};
  HAL_SPI_Transmit(&hspi1, pData9, 2, 1000);
  uint8_t pData10[2] = {0xB3, 0xF0};
  HAL_SPI_Transmit(&hspi1, pData10, 2, 1000);
  uint8_t pData11[2] = {0x8A, 0x64};
  HAL_SPI_Transmit(&hspi1, pData11, 2, 1000);
  uint8_t pData12[2] = {0x8B, 0x78};
  HAL_SPI_Transmit(&hspi1, pData12, 2, 1000);
  uint8_t pData13[2] = {0x8C, 0x64};
  HAL_SPI_Transmit(&hspi1, pData13, 2, 1000);
  uint8_t pData14[2] = {0xBB, 0x3A};
  HAL_SPI_Transmit(&hspi1, pData14, 2, 1000);
  uint8_t pData15[2] = {0xBE, 0x3E};
  HAL_SPI_Transmit(&hspi1, pData15, 2, 1000);
  uint8_t pData16[2] = {0x87, 0x06};
  HAL_SPI_Transmit(&hspi1, pData16, 2, 1000);
  uint8_t pData17[2] = {0x81, 0x91};
  HAL_SPI_Transmit(&hspi1, pData17, 2, 1000);
  uint8_t pData18[2] = {0x82, 0x50};
  HAL_SPI_Transmit(&hspi1, pData18, 2, 1000);
  uint8_t pData19[2] = {0x83, 0x7D};
  HAL_SPI_Transmit(&hspi1, pData19, 2, 1000);
  uint8_t pData20[1] = {0x2E};
  HAL_SPI_Transmit(&hspi1, pData20, 1, 1000);
  uint8_t pData21[5] = {0x25, 0x00, 0x00, 0x5F, 0x3F};
  HAL_SPI_Transmit(&hspi1, pData21, 5, 1000);

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET); // set RESET back to high
  osDelay(25);

  uint8_t pData22[1] = {0xA5};
  HAL_SPI_Transmit(&hspi1, pData22, 1, 1000);
  osDelay(100);

  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
    if (turn_off_flag == 1) {
    	uint8_t pData23[1] = {0xAE};
    	HAL_SPI_Transmit(&hspi1, pData23, 1, 1000);
    	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
    	osDelay(400);
    	vTaskDelete(NULL);
    }
  }
  /* USER CODE END 5 */
}





/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
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

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
