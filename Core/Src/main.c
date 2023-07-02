


/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <math.h>
#include "../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.h"
#include <stdio.h>
#include <stdlib.h>
#include <arm_math_types.h>
#include <basic_math_functions.h>
#include <support_functions.h>

/* Private defines ----------------------------------------------------------*/

#define SPI_TIMEOUT 100000
#define LIST_LEN 32
#define N 5
#define iir_inc .5

/* Private structs ----------------------------------------------------------*/
typedef struct complexNumber {
	float real;
	float img;
} complex;

/* function prototypes -----------------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2S2_Init(void);
static void MX_UART4_Init(void);
static void MX_SPI1_Init(void);
uint32_t update_and_calculate_average(uint32_t);
void send_SPI_message(uint8_t *, uint16_t);
void start_LED_screen(void);
void turnoff_LED_screen(void);
void clear_LED_screen(void);
void draw_Rectangle(uint8_t, uint8_t, uint8_t, uint8_t);
void LEDScreenTask(void const *);
void TickDelayTask(void const *);
void calculate_FFT(void);
float Q_rsqrt(float);
complex complexAdd(complex, complex);
complex complexMult(complex, complex);
float cabsf_0(complex);

/* USER CODE BEGIN PFP */
void Process_Data(char *);
float32_t low_pass_filter(float32_t);


/* Private variables ---------------------------------------------------------*/
I2S_HandleTypeDef hi2s2;
DMA_HandleTypeDef hdma_i2s2_ext_rx;
DMA_HandleTypeDef hdma_spi2_tx;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart4;

osThreadId LEDScreenTaskHandle;
/* USER CODE BEGIN PV */
uint16_t rx_buf[16];
uint16_t tx_buf[16];

float32_t input_list[N+1];
float32_t output_list[N+1];

int count = 0;
// butterworth filter with N= 3 and cutoff ~.83
//float32_t b[4] = {0.5887, 1.7660, 1.7660, 0.5887};
//float32_t a[3] = {1.9630, 1.4000, 0.3464};

// butterworth filter with N= 3 and cutoff = .6
//float32_t b[4] = {.2569, .7707, .7707, .2569};
//float32_t a[3] = {.577241, .421787, .056297};

// butterworth filter with N= 3 and cutoff = .5
//float32_t b[4] = {.1667, .500, .500, .1667};
//float32_t a[3] = {0, .33333, 0};

// butterworth filter with N= 5 and cutoff = .5
float32_t b[6] = {0.052786, 0.263932, 0.527864, 0.263932, 0.052786};
float32_t a[5] = {0, .63344, 0, .055728, 0};

volatile uint32_t avg = 0;
volatile uint8_t list_counter = 0;
volatile uint32_t average_volume_list[LIST_LEN];
volatile float32_t return_vals[16];
float first_vals[16];
complex second_vals[16] =
{
		[0] =  { .real = 0.0,     .img = 0.0 },
		[1] =  { .real = 0.0,     .img = 0.0 },
		[2] =  { .real = 0.0,     .img = 0.0 },
		[3] =  { .real = 0.0,     .img = 0.0 },
		[4] =  { .real = 0.0,     .img = 0.0 },
		[5] =  { .real = 0.0,     .img = 0.0 },
		[6] =  { .real = 0.0,     .img = 0.0 },
		[7] =  { .real = 0.0,     .img = 0.0 },
		[8] =  { .real = 0.0,     .img = 0.0 },
		[9] =  { .real = 0.0,     .img = 0.0 },
		[10] = { .real = 0.0,     .img = 0.0 },
		[11] = { .real = 0.0,     .img = 0.0 },
		[12] = { .real = 0.0,     .img = 0.0 },
		[13] = { .real = 0.0,     .img = 0.0 },
		[14] = { .real = 0.0,     .img = 0.0 },
		[15] = { .real = 0.0,     .img = 0.0 }
};
complex third_vals[16] =
{
		[0] =  { .real = 0.0,     .img = 0.0 },
		[1] =  { .real = 0.0,     .img = 0.0 },
		[2] =  { .real = 0.0,     .img = 0.0 },
		[3] =  { .real = 0.0,     .img = 0.0 },
		[4] =  { .real = 0.0,     .img = 0.0 },
		[5] =  { .real = 0.0,     .img = 0.0 },
		[6] =  { .real = 0.0,     .img = 0.0 },
		[7] =  { .real = 0.0,     .img = 0.0 },
		[8] =  { .real = 0.0,     .img = 0.0 },
		[9] =  { .real = 0.0,     .img = 0.0 },
		[10] = { .real = 0.0,     .img = 0.0 },
		[11] = { .real = 0.0,     .img = 0.0 },
		[12] = { .real = 0.0,     .img = 0.0 },
		[13] = { .real = 0.0,     .img = 0.0 },
		[14] = { .real = 0.0,     .img = 0.0 },
		[15] = { .real = 0.0,     .img = 0.0 }
};

complex fourth_vals[16] =
{
		[0] =  { .real = 0.0,     .img = 0.0 },
		[1] =  { .real = 0.0,     .img = 0.0 },
		[2] =  { .real = 0.0,     .img = 0.0 },
		[3] =  { .real = 0.0,     .img = 0.0 },
		[4] =  { .real = 0.0,     .img = 0.0 },
		[5] =  { .real = 0.0,     .img = 0.0 },
		[6] =  { .real = 0.0,     .img = 0.0 },
		[7] =  { .real = 0.0,     .img = 0.0 },
		[8] =  { .real = 0.0,     .img = 0.0 },
		[9] =  { .real = 0.0,     .img = 0.0 },
		[10] = { .real = 0.0,     .img = 0.0 },
		[11] = { .real = 0.0,     .img = 0.0 },
		[12] = { .real = 0.0,     .img = 0.0 },
		[13] = { .real = 0.0,     .img = 0.0 },
		[14] = { .real = 0.0,     .img = 0.0 },
		[15] = { .real = 0.0,     .img = 0.0 }
};

complex fifth_vals[16] =
{
		[0] =  { .real = 0.0,     .img = 0.0 },
		[1] =  { .real = 0.0,     .img = 0.0 },
		[2] =  { .real = 0.0,     .img = 0.0 },
		[3] =  { .real = 0.0,     .img = 0.0 },
		[4] =  { .real = 0.0,     .img = 0.0 },
		[5] =  { .real = 0.0,     .img = 0.0 },
		[6] =  { .real = 0.0,     .img = 0.0 },
		[7] =  { .real = 0.0,     .img = 0.0 },
		[8] =  { .real = 0.0,     .img = 0.0 },
		[9] =  { .real = 0.0,     .img = 0.0 },
		[10] = { .real = 0.0,     .img = 0.0 },
		[11] = { .real = 0.0,     .img = 0.0 },
		[12] = { .real = 0.0,     .img = 0.0 },
		[13] = { .real = 0.0,     .img = 0.0 },
		[14] = { .real = 0.0,     .img = 0.0 },
		[15] = { .real = 0.0,     .img = 0.0 }
};

complex twiddle_factors[16] =
{
		[0] =  { .real = 1.0,     .img = 0.0     },
		[1] =  { .real = 0.9239,  .img = -0.3827 },
		[2] =  { .real = 0.7071,  .img = -0.7072 },
		[3] =  { .real = 0.3827,  .img = -0.9239 },
		[4] =  { .real = 0.0,     .img = -1.0    },
		[5] =  { .real = -0.3827, .img = -0.9239 },
		[6] =  { .real = -0.7071, .img = -0.7071 },
		[7] =  { .real = -0.9239, .img = -0.3827 },
		[8] =  { .real = -1.0,    .img = 0.0     },
		[9] =  { .real = -0.9239, .img = 0.3827  },
		[10] = { .real = -0.7071, .img = 0.7071  },
		[11] = { .real = -0.3827, .img = 0.9239  },
		[12] = { .real = 0.0,     .img = 1.0     },
		[13] = { .real = 0.3827,  .img = 0.9239  },
		[14] = { .real = 0.7071,  .img = 0.7071  },
		[15] = { .real = 0.9239,  .img = 0.3827  }
};



/* Private user code ---------------------------------------------------------*/
int _write(int fd, char *ptr, int len)
{
	HAL_UART_Transmit(&huart4, (uint8_t*) ptr, len, SPI_TIMEOUT);
	return len;
}

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


  printf("starting up...\n\r");

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

	avg = update_and_calculate_average((uint32_t)abs(left_in_1)); // calculate average of last LIST_LEN samples

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



float32_t low_pass_filter(float32_t left) {

	int i;
	float32_t return_value;

	for (i = 0; i < N; i++) {
		input_list[N-i] = input_list[N-i-1];
		output_list[N-i] = output_list[N-i-1];
	}
	input_list[0] = left;
    float32_t temp_output_list[N];

    // this is where things get wonky
    for (i = 0; i < N; i++) {
    	temp_output_list[i] = output_list[i+1];
    }
	if (count < 16) {
		output_list[0] = left;
		return_value = output_list[0];
		count++;
	}
	else {
		float32_t result_fir;
		float32_t result_iir;
		arm_dot_prod_f32(input_list, b, N+1, &result_fir);
		arm_dot_prod_f32(temp_output_list, a, N, &result_iir);
		output_list[0] = result_fir - result_iir;
		return_value = output_list[0];
	}

	return return_value;
}

void calculate_FFT(void) {
	// use average value list to calculate 16 point FFT.
	// we only want the top 11 values of the FFT so Y[15]...Y[5]

	uint8_t counter_pos = list_counter + 1;

	uint32_t temp_average_volume_list[16];
	for (int i = 0; i < 16; i++) {
		temp_average_volume_list[i] = average_volume_list[(counter_pos+i)%LIST_LEN];
	}

	first_vals[0] = (float)temp_average_volume_list[0];
	first_vals[1] = (float)temp_average_volume_list[1];
	first_vals[2] = (float)temp_average_volume_list[2];
	first_vals[3] = (float)temp_average_volume_list[3];
	first_vals[4] = (float)temp_average_volume_list[4];
	first_vals[5] = (float)temp_average_volume_list[5];
	first_vals[6] = (float)temp_average_volume_list[6];
	first_vals[7] = (float)temp_average_volume_list[7];
	first_vals[8] = (float)temp_average_volume_list[8];
	first_vals[9] = (float)temp_average_volume_list[9];
	first_vals[10] = (float)temp_average_volume_list[10];
	first_vals[11] = (float)temp_average_volume_list[11];
	first_vals[12] = (float)temp_average_volume_list[12];
	first_vals[13] = (float)temp_average_volume_list[13];
	first_vals[14] = (float)temp_average_volume_list[14];
	first_vals[15] = (float)temp_average_volume_list[15];

	//printf("first_vals_9: %d\n\r", (int)(first_vals[9]));

	second_vals[0] = (complex){.real = first_vals[0] + first_vals[1], .img = 0.0};
	second_vals[1] = (complex){.real = first_vals[0] + first_vals[1], .img = 0.0};
	second_vals[2] = (complex){.real = first_vals[2] + first_vals[3], .img = 0.0};
	second_vals[3] = (complex){.real = first_vals[2] + first_vals[3], .img = 0.0};
		second_vals[3] = complexMult(second_vals[3], twiddle_factors[4]);
	second_vals[4] = (complex){.real = first_vals[4] + first_vals[5], .img = 0.0};
	second_vals[5] = (complex){.real = first_vals[4] + first_vals[5], .img = 0.0};
	second_vals[6] = (complex){.real = first_vals[6] + first_vals[7], .img = 0.0};
	second_vals[7] = (complex){.real = first_vals[6] + first_vals[7], .img = 0.0};
		second_vals[7] = complexMult(second_vals[7], twiddle_factors[4]);
	second_vals[8] = (complex){.real = first_vals[8] + first_vals[9], .img = 0.0};
	second_vals[9] = (complex){.real = first_vals[8] + first_vals[9], .img = 0.0};
	second_vals[10] =(complex){.real = first_vals[10] +first_vals[11],.img = 0.0};
	second_vals[11] =(complex){.real = first_vals[10] +first_vals[11],.img = 0.0};
		second_vals[11] = complexMult(second_vals[11], twiddle_factors[4]);
	second_vals[12] =(complex){.real = first_vals[12] +first_vals[13],.img = 0.0};
	second_vals[13] =(complex){.real = first_vals[12] +first_vals[13],.img = 0.0};
	second_vals[14] =(complex){.real = first_vals[14] +first_vals[15],.img = 0.0};
	second_vals[15] =(complex){.real = first_vals[14] +first_vals[15],.img = 0.0};
		second_vals[15]= complexMult(second_vals[15], twiddle_factors[4]);

	//printf("second_vals_9: %d + %dI\n\r", (int)(second_vals[9].real), (int)(second_vals[9].img));

	third_vals[0] = complexAdd(second_vals[0], second_vals[2]);
	third_vals[1] = complexAdd(second_vals[1], second_vals[3]);
	third_vals[2] = complexAdd(second_vals[0], second_vals[2]);
	third_vals[3] = complexAdd(second_vals[1], second_vals[3]);
	third_vals[4] = complexAdd(second_vals[4], second_vals[6]);
	third_vals[5] = complexAdd(second_vals[5], second_vals[7]);
		third_vals[5] = complexMult(third_vals[5], twiddle_factors[2]);
	third_vals[6] = complexAdd(second_vals[4], second_vals[6]);
		third_vals[6] = complexMult(third_vals[6], twiddle_factors[4]);
	third_vals[7] = complexAdd(second_vals[5], second_vals[7]);
		third_vals[7] = complexMult(third_vals[7], twiddle_factors[6]);
	third_vals[8] = complexAdd(second_vals[8], second_vals[10]);
	third_vals[9] = complexAdd(second_vals[9], second_vals[11]);
	third_vals[10] = complexAdd(second_vals[8], second_vals[10]);
	third_vals[11] = complexAdd(second_vals[9], second_vals[11]);
	third_vals[12] = complexAdd(second_vals[12], second_vals[14]);
	third_vals[13] = complexAdd(second_vals[13], second_vals[15]);
		third_vals[13] = complexMult(third_vals[13], twiddle_factors[2]);
	third_vals[14] = complexAdd(second_vals[12], second_vals[14]);
		third_vals[14] = complexMult(third_vals[14], twiddle_factors[4]);
	third_vals[15] = complexAdd(second_vals[13], second_vals[15]);
		third_vals[15] = complexMult(third_vals[15], twiddle_factors[6]);

	//printf("third_vals_9: %d + %dI\n\r", (int)(third_vals[9].real), (int)(third_vals[9].img));

	fourth_vals[0] = complexAdd(third_vals[0], third_vals[4]);
	fourth_vals[1] = complexAdd(third_vals[1], third_vals[5]);
	fourth_vals[2] = complexAdd(third_vals[2], third_vals[6]);
	fourth_vals[3] = complexAdd(third_vals[3], third_vals[7]);
	fourth_vals[4] = complexAdd(third_vals[0], third_vals[4]);
	fourth_vals[5] = complexAdd(third_vals[1], third_vals[5]);
	fourth_vals[6] = complexAdd(third_vals[2], third_vals[6]);
	fourth_vals[7] = complexAdd(third_vals[3], third_vals[7]);
	fourth_vals[8] = complexAdd(third_vals[8], third_vals[12]);
	fourth_vals[9] = complexAdd(third_vals[9], third_vals[13]);
		fourth_vals[9] = complexMult(fourth_vals[9], twiddle_factors[1]);
	fourth_vals[10] = complexAdd(third_vals[10], third_vals[14]);
		fourth_vals[10] = complexMult(fourth_vals[10], twiddle_factors[2]);
	fourth_vals[11] = complexAdd(third_vals[11], third_vals[15]);
		fourth_vals[11] = complexMult(fourth_vals[11], twiddle_factors[3]);
	fourth_vals[12] = complexAdd(third_vals[8], third_vals[12]);
		fourth_vals[12] = complexMult(fourth_vals[12], twiddle_factors[4]);
	fourth_vals[13] = complexAdd(third_vals[9], third_vals[13]);
		fourth_vals[13] = complexMult(fourth_vals[13], twiddle_factors[5]);
	fourth_vals[14] = complexAdd(third_vals[10], third_vals[14]);
		fourth_vals[14] = complexMult(fourth_vals[14], twiddle_factors[6]);
	fourth_vals[15] = complexAdd(third_vals[11], third_vals[15]);
		fourth_vals[15] = complexMult(fourth_vals[15], twiddle_factors[7]);

	//printf("fourth_vals_9: %d + %dI\n\r", (int)(fourth_vals[9].real), (int)(fourth_vals[9].img));

	fifth_vals[0] = complexAdd(fourth_vals[0],fourth_vals[8]);
	fifth_vals[1] = complexAdd(fourth_vals[1],fourth_vals[9]);
	fifth_vals[2] = complexAdd(fourth_vals[2],fourth_vals[10]);
	fifth_vals[3] = complexAdd(fourth_vals[3],fourth_vals[11]);
	fifth_vals[4] = complexAdd(fourth_vals[4],fourth_vals[12]);
	fifth_vals[5] = complexAdd(fourth_vals[5],fourth_vals[13]);
	fifth_vals[6] = complexAdd(fourth_vals[6],fourth_vals[14]);
	fifth_vals[7] = complexAdd(fourth_vals[7],fourth_vals[15]);
	fifth_vals[8] = complexAdd(fourth_vals[0],fourth_vals[8]);
	fifth_vals[9] = complexAdd(fourth_vals[1],fourth_vals[9]);
	fifth_vals[10] = complexAdd(fourth_vals[2],fourth_vals[10]);
	fifth_vals[11] = complexAdd(fourth_vals[3],fourth_vals[11]);
	fifth_vals[12] = complexAdd(fourth_vals[4],fourth_vals[12]);
	fifth_vals[13] = complexAdd(fourth_vals[5],fourth_vals[13]);
	fifth_vals[14] = complexAdd(fourth_vals[6],fourth_vals[14]);
	fifth_vals[15] = complexAdd(fourth_vals[7],fourth_vals[15]);

	//printf("fifth_vals_9: %d + %dI\n\r", (int)(fifth_vals[9].real), (int)(fifth_vals[9].img));

	return_vals[0] = cabsf_0(fifth_vals[0]) + iir_inc*return_vals[0];
	return_vals[1] = cabsf_0(fifth_vals[1]) + iir_inc*return_vals[1];
	return_vals[2] = cabsf_0(fifth_vals[2]) + iir_inc*return_vals[2];
	return_vals[3] = cabsf_0(fifth_vals[3]) + iir_inc*return_vals[3];
	return_vals[4] = cabsf_0(fifth_vals[4]) + iir_inc*return_vals[4];
	return_vals[5] = cabsf_0(fifth_vals[5]) + iir_inc*return_vals[5];
	return_vals[6] = cabsf_0(fifth_vals[6]) + iir_inc*return_vals[6];
	return_vals[7] = cabsf_0(fifth_vals[7]) + iir_inc*return_vals[7];
	return_vals[8] = cabsf_0(fifth_vals[8]) + iir_inc*return_vals[8];
	return_vals[9] = cabsf_0(fifth_vals[9]) + iir_inc*return_vals[9];
	return_vals[10] = cabsf_0(fifth_vals[10]) + iir_inc*return_vals[10];
	return_vals[11] = cabsf_0(fifth_vals[11]) + iir_inc*return_vals[11];
	return_vals[12] = cabsf_0(fifth_vals[12]) + iir_inc*return_vals[12];
	return_vals[13] = cabsf_0(fifth_vals[13]) + iir_inc*return_vals[13];
	return_vals[14] = cabsf_0(fifth_vals[14]) + iir_inc*return_vals[14];
	return_vals[15] = cabsf_0(fifth_vals[15]) + iir_inc*return_vals[15];
}


/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
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
	/*printf("return_val_0: %d\n\r", (int)return_vals[0]);
	printf("return_val_1: %d\n\r", (int)return_vals[1]);
	printf("return_val_2: %d\n\r", (int)return_vals[2]);
	printf("return_val_3: %d\n\r", (int)return_vals[3]);
	printf("return_val_4: %d\n\r", (int)return_vals[4]);
	printf("return_val_5: %d\n\r", (int)return_vals[5]);
	printf("return_val_6: %d\n\r", (int)return_vals[6]);
	printf("return_val_7: %d\n\r", (int)return_vals[7]);
	printf("return_val_8: %d\n\r", (int)return_vals[8]);
	printf("return_val_9: %d\n\r", (int)return_vals[9]);
	printf("return_val_10: %d\n\r", (int)return_vals[10]);
	printf("return_val_11: %d\n\r", (int)return_vals[11]);
	printf("return_val_12: %d\n\r", (int)return_vals[12]);
	printf("return_val_13: %d\n\r", (int)return_vals[13]);
	printf("return_val_14: %d\n\r", (int)return_vals[14]);
	printf("return_val_15: %d\n\n\n\r", (int)return_vals[15]);*/
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

	// for printing the total vu meter value to screen
	/*if (avg < 500000) {
		row_ender = 7;
	} else if (avg < 2000000) {
		row_ender = 15;
	} else if (avg < 3000000) {
		row_ender = 23;
	} else if (avg < 4000000) {
		row_ender = 31;
	} else if (avg < 5000000) {
		row_ender = 39;
	} else if (avg < 6000000) {
		row_ender = 47;
	} else {
		row_ender = 55;
	}
	*/
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



uint32_t update_and_calculate_average(uint32_t value) {
	average_volume_list[list_counter] = value;
	int sum = 0;
	for (int i = 0; i < LIST_LEN; i++) {
		sum += average_volume_list[i];
	}
	sum = sum / LIST_LEN;
	list_counter++;
	list_counter = list_counter % LIST_LEN;
	return sum;
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

float Q_rsqrt( float number )
{
	long i;
	float x2, y;
	const float threehalfs = 1.5F;

	x2 = number * 0.5F;
	y  = number;
	i  = * ( long * ) &y;                       // evil floating point bit level hacking
	i  = 0x5f3759df - ( i >> 1 );               // what the fuck?
	y  = * ( float * ) &i;
	y  = y * ( threehalfs - ( x2 * y * y ) );   // 1st iteration
//	y  = y * ( threehalfs - ( x2 * y * y ) );   // 2nd iteration, this can be removed

	return y;
}


complex complexAdd(complex a, complex b) {
	complex temp;
	temp.real = a.real + b.real;
	temp.img = a.img + b.img;
	return temp;
}

complex complexMult(complex a, complex b) {
	complex temp;
	temp.real = a.real*b.real - a.img*b.img;
	temp.img = a.real*b.img + a.img*b.real;
	return temp;
}

float cabsf_0(complex a) {
	float term1 = a.real*a.real;
	float term2 = a.img * a.img;
	return sqrt(term1+term2);
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
