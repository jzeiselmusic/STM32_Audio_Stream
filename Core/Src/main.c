/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2S_HandleTypeDef hi2s2;
DMA_HandleTypeDef hdma_i2s2_ext_rx;
DMA_HandleTypeDef hdma_spi2_tx;

UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */
uint16_t rx_buf[8];
uint16_t tx_buf[8];


double input_list[10];
double output_list[10];
int count;

double b0 = 0.5887;
double b1 = 1.7660;
double b2 = 1.7660;
double b3 = 0.5887;
double a1 = 1.9630;
double a2 = 1.4000;
double a3 = 0.3464;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2S2_Init(void);
static void MX_UART4_Init(void);
/* USER CODE BEGIN PFP */
void Process_Data(char *);
double low_pass_filter(double, double);


/* USER CODE END PFP */


int _write(int fd, char * ptr, int len)
{
  //HAL_UART_Transmit(&huart4, (uint8_t *) ptr, len, HAL_MAX_DELAY);
  HAL_UART_Transmit_DMA(&huart4, (uint8_t *) ptr, len);
  return len;
}

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2S2_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */

  HAL_I2SEx_TransmitReceive_DMA(&hi2s2, tx_buf, rx_buf, 4);

  //uint8_t message[] = "transmit receive started!\n\r";
  printf("transmit receive started!\r\n");
  //PRINT("transmit receive started!\n\r", sizeof("transmit receive started!\n\r"));

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /* USER CODE BEGIN I2S2_Init 0 */

  /* USER CODE END I2S2_Init 0 */

  /* USER CODE BEGIN I2S2_Init 1 */

  /* USER CODE END I2S2_Init 1 */
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
  /* USER CODE BEGIN I2S2_Init 2 */

  /* USER CODE END I2S2_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
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
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

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
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  //HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  //HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
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
		start += 4;
	}

	int left_in = (((int)rx_buf[start]<<16)|rx_buf[start+1])>>8;
	int right_in = (((int)rx_buf[start+2]<<16)|rx_buf[start+3])>>8;

	//printf("ints:\r\n");
	//printf("incoming left val:\t%d\r\n", left_in);
	//printf("incoming right val:\t%d\r\n", right_in);

	// left in and right in are both 16 bit integers converted to 24 bit ints
	// we can do processing on them here, as long as it is
	// done in time for the buffer to be passed on by the DMA unit

	// implement a simple tanh soft distortion mechanism

	double float_left_in = (double)left_in;
	double float_right_in = (double)right_in;

	float_left_in = low_pass_filter(float_left_in, float_right_in);
	float_right_in = float_left_in;

	//float_left_in = float_left_in / 16777216.0;
	//float_right_in = float_right_in / 16777216.0;

	//tanh_approx(&float_left_in);
	//tanh_approx(&float_right_in);

	//float_left_in = 2 * float_left_in * 16777216.0;
	//float_right_in = 2 * float_right_in * 16777216.0;


	int left_out = (int)float_left_in;
	int right_out = (int)float_right_in;

	//int left_out = left_in;
	//int right_out = right_in;

	tx_buf[start] = (left_out>>8) & 0xFFFF;
	tx_buf[start+1] = left_out & 0xFFFF;
	tx_buf[start+2] = (right_out>>8) & 0xFFFF;
	tx_buf[start+3] = right_out & 0xFFFF;
}



double low_pass_filter(double left, double right) {

	double return_value;
	input_list[9] = input_list[8];
	input_list[8] = input_list[7];
	input_list[7] = input_list[6];
	input_list[6] = input_list[5];
	input_list[5] = input_list[4];
	input_list[4] = input_list[3];
	input_list[3] = input_list[2];
	input_list[2] = input_list[1];
	input_list[1] = input_list[0];
	input_list[0] = left;

	output_list[9] = output_list[8];
	output_list[8] = output_list[7];
	output_list[7] = output_list[6];
	output_list[6] = output_list[5];
	output_list[5] = output_list[4];
	output_list[4] = output_list[3];
	output_list[3] = output_list[2];
	output_list[2] = output_list[1];
	output_list[1] = output_list[0];

	if (count < 10) {
		output_list[0] = left;
		return_value = output_list[0];
	}
	else {
		output_list[0] = input_list[0]*b0 + input_list[1]*b1 + input_list[2]*b2
						+ input_list[3]*b3 - output_list[1]*a1 - output_list[2]*a2
						- output_list[3]*a3;
		return_value = output_list[0];
	}

	count++;
	if (count > 15) {
		count = 15;
	}
	return return_value;
}

/* USER CODE END 4 */

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
