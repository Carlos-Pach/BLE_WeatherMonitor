/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BME280.h"
#include "OLED.h"
#include "FXOS.h"
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define BME280_PERIOD_MS			(1000UL)	// period to check BME280 sensor status
#define OLED_PERIOD_MS				(40UL)		// period to update OLED display
#define ADC_PERIOD_MS				(10UL)		// period to check ADC value

#define ESP32_ADDR					(0x07)		// slave address for ESP32 I2C
#define ESP32_BUF_SIZE				(2UL)		// I2C buffer size
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s3;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart2;

/* Definitions for TaskBME280 */
osThreadId_t TaskBME280Handle;
const osThreadAttr_t TaskBME280_attributes = {
  .name = "TaskBME280",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for TaskOLED */
osThreadId_t TaskOLEDHandle;
const osThreadAttr_t TaskOLED_attributes = {
  .name = "TaskOLED",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for TaskADC */
osThreadId_t TaskADCHandle;
const osThreadAttr_t TaskADC_attributes = {
  .name = "TaskADC",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for QueuePeriphs */
osMessageQueueId_t QueuePeriphsHandle;
const osMessageQueueAttr_t QueuePeriphs_attributes = {
  .name = "QueuePeriphs"
};
/* Definitions for BME280_Timer */
osTimerId_t BME280_TimerHandle;
const osTimerAttr_t BME280_Timer_attributes = {
  .name = "BME280_Timer"
};
/* Definitions for adc_oled */
osMutexId_t adc_oledHandle;
const osMutexAttr_t adc_oled_attributes = {
  .name = "adc_oled"
};
/* Definitions for BME280_BinSem */
osSemaphoreId_t BME280_BinSemHandle;
const osSemaphoreAttr_t BME280_BinSem_attributes = {
  .name = "BME280_BinSem"
};
/* USER CODE BEGIN PV */

volatile Bool buttonIRQ_g = False ;
uint8_t dimmingVal_g = 0 ;	// only global in main.c

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S3_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
void StartTaskBME280(void *argument);
void StartTaskOLED(void *argument);
void StartTaskADC(void *argument);
void BME280_Callback(void *argument);

/* USER CODE BEGIN PFP */
void floatToStr(float f, int8_t *buf, uint8_t decPlaces) ;
/* USER CODE END PFP */

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
  MX_I2C1_Init();
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of adc_oled */
  adc_oledHandle = osMutexNew(&adc_oled_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of BME280_BinSem */
  BME280_BinSemHandle = osSemaphoreNew(1, 1, &BME280_BinSem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of BME280_Timer */
  BME280_TimerHandle = osTimerNew(BME280_Callback, osTimerPeriodic, NULL, &BME280_Timer_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */

  // start BME280_TimerHandle
  osTimerStart(BME280_TimerHandle, BME280_PERIOD_MS) ;

  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of QueuePeriphs */
  QueuePeriphsHandle = osMessageQueueNew (16, sizeof(uint64_t), &QueuePeriphs_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of TaskBME280 */
  TaskBME280Handle = osThreadNew(StartTaskBME280, NULL, &TaskBME280_attributes);

  /* creation of TaskOLED */
  TaskOLEDHandle = osThreadNew(StartTaskOLED, NULL, &TaskOLED_attributes);

  /* creation of TaskADC */
  TaskADCHandle = osThreadNew(StartTaskADC, NULL, &TaskADC_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_1);
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_8B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_96K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
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
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SPI_RES_Pin|SPI_DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, SPI_CS_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD6_Pin|GPIO_PIN_2|Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_BlueButton_Pin */
  GPIO_InitStruct.Pin = B1_BlueButton_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_BlueButton_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI_RES_Pin SPI_DC_Pin */
  GPIO_InitStruct.Pin = SPI_RES_Pin|SPI_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI_CS_Pin LD4_Pin LD3_Pin LD5_Pin
                           LD6_Pin PD2 Audio_RST_Pin */
  GPIO_InitStruct.Pin = SPI_CS_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD6_Pin|GPIO_PIN_2|Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */
void floatToStr(float f, int8_t *buf, uint8_t decPlaces){
	uint8_t i = 1 ;
	int8_t iPart = 0, fPart = 0 ;

	// do power of 10 by decPlaces
	while(decPlaces > 0){
		i = 10 * i ;
		decPlaces-- ;
	}

	iPart = (int8_t)f ;
	fPart = (f - iPart) * i ;

	sprintf(buf, "%i.%i\n", iPart, fPart) ;
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartTaskBME280 */
/**
  * @brief  Function implementing the TaskBME280 thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartTaskBME280 */
void StartTaskBME280(void *argument)
{
  /* init code for USB_HOST */
  MX_USB_HOST_Init();
  /* USER CODE BEGIN 5 */
  tBME280Conversion conv ;

  // create BME280 struct
  static struct weatherSensor BME280 ;

  // buffer for bme280
  int8_t buf[10] = { [0 ... 9] = '\n' },
		 n_buf = sizeof(buf)/sizeof(buf[0]) ;

  // buffer for esp32
  uint8_t espBuf[ESP32_BUF_SIZE] = { [0 ... (ESP32_BUF_SIZE - 1)] = '\n' },
		  m_buf = sizeof(espBuf)/sizeof(espBuf[0]) ;


  // reset BME280
  if(BME280_reset(&BME280, &hi2c1) != True){	// turn on LED D12 if error
	  GPIOD->ODR |= (1 << 12) ;
	  while(1) ;
  }
  // configure BME280
  if(BME280_initParams(&BME280)){
	  __asm("nop") ;
  }
  // initialize BME280 with configurations
  if(BME280_init(&BME280, &hi2c1) != True){	// turn on LED D13 if error
	  GPIOD->ODR |= (1 << 13) ;
	  while(1) ;
  }
  // send control measurement data for humidity sensor
  if(BME280_ctrlHumid(&BME280, &hi2c1) != True){	// turn on LED D14 if error
	  GPIOD->ODR |= (1 << 14) ;
	  while(1) ;
  }
  // send control measurement data for pressure + temp sensor
  if(BME280_ctrlMeas(&BME280, &hi2c1) != True){	// turn on LED D15 if error
	  GPIOD->ODR |= (1 << 15) ;
	  while(1) ;
  }
  // read BME280 ID
  if(BME280_readID(&BME280, &hi2c1) != True){	// turn on LED D12, D13 if error
	  GPIOD->ODR |= (1 << 12) ;
	  GPIOD->ODR |= (1 << 13) ;
	  while(1) ;
  }

  // call function that gets compensation vals
  BME280_getCompensationVals(&BME280, &hi2c1, DIG_T1_ADDR) ;
  BME280_getCompensationVals(&BME280, &hi2c1, DIG_P1_ADDR) ;
  BME280_getCompensationVals(&BME280, &hi2c1, DIG_H1_ADDR) ;

  /* Infinite loop */
  for(;;)
  {

	  // turn off external LED
	  GPIOD->ODR &= ~(1 << 2) ;
	  // check binary semaphore if data is ready to be read
	  if(osSemaphoreGetCount(BME280_BinSemHandle) != 0){
		  // light up all on-board LEDs when we are reading data
		  GPIOD->ODR |= (1 << 12) ;
		  GPIOD->ODR |= (1 << 13) ;
		  GPIOD->ODR |= (1 << 14) ;
		  GPIOD->ODR |= (1 << 15) ;

		  // read sensor data: temperature, pressure, humidity
		  BME280_readSensorData(&BME280, &hi2c1, BME280.temperatureAddr) ;
		  BME280_readSensorData(&BME280, &hi2c1, BME280.pressureAddr) ;
		  BME280_readSensorData(&BME280, &hi2c1, BME280.humidityAddr) ;

		  // call function that compensates vals
		  BME280.compTemp = BME280_compensateTemperature(&BME280) ;
		  BME280.compPressure = BME280_compensatePressure(&BME280) ;
		  BME280.compHumid = BME280_compensateHumidity(&BME280) ;

		  floatToStr(BME280.compTemp, buf, 2) ;
		  BME280.compTemp = (1.8 * BME280.compTemp) + 32 ;	// convert to degrees F

		  // fill esp32 buffer for data transfer via I2C
		  espBuf[0] = BME280.compHumid ;
		  espBuf[1] = BME280.compTemp ;

		  // transfer data to ESP32
		  HAL_I2C_Master_Transmit(&hi2c1, (ESP32_ADDR << 1), espBuf, m_buf, HAL_MAX_DELAY) ;

		  // check current number of messages in queue
		  // if it is below max - 3, allow data to be put into queue
		  if(osMessageQueueGetCount(QueuePeriphsHandle) < (osMessageQueueGetCapacity(QueuePeriphsHandle) - 6)){
			  // send temperature conversion ptr
			  conv = TEMP_CONV ;
			  // moved c to f conversion above
			  osMessageQueuePut(QueuePeriphsHandle, &conv, 0U, 0U) ;
			  osMessageQueuePut(QueuePeriphsHandle, &BME280.compTemp, 0U, 0U) ;

			  // send pressure conversion ptr
			  conv = PRESSURE_CONV ;
			  osMessageQueuePut(QueuePeriphsHandle, &conv, 0U, 0U) ;
			  osMessageQueuePut(QueuePeriphsHandle, &BME280.compPressure, 0U, 0U) ;

			  // send humidity conversion ptr
			  conv = HUMID_CONV ;
			  osMessageQueuePut(QueuePeriphsHandle, &conv, 0U, 0U) ;
			  osMessageQueuePut(QueuePeriphsHandle, &BME280.compHumid, 0U, 0U) ;
		  }

		  // turn off all on-board LEDs once data has been read + queue'd
		  GPIOD->ODR &= ~(1 << 12) ;
		  GPIOD->ODR &= ~(1 << 13) ;
		  GPIOD->ODR &= ~(1 << 14) ;
		  GPIOD->ODR &= ~(1 << 15) ;
	  } else{	// BME280 data not ready, turn on LED
		  GPIOD->ODR |= (1 << 2) ;
	  }

	  osThreadYield() ;
	  osDelay(BME280_PERIOD_MS);
  }
  osThreadTerminate(StartTaskBME280) ;
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTaskOLED */
/**
* @brief Function implementing the TaskOLED thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskOLED */
void StartTaskOLED(void *argument)
{
  /* USER CODE BEGIN StartTaskOLED */

	// create vars for BME280 value conversions
	double tempVarConv ;

	uint8_t msgLen = 0 ;

	int8_t temperatureStr[6] = { [0 ... 5] = '\n' },
		   humidityStr[6] = { [0 ... 5] = '\n' },
		   pressureStr[6] = { [0 ... 5] = '\n' },
		   *ptr[3] ;

	// create conversion enum var
	tBME280Conversion var ;

	// create OLED struct
	static struct display OLED ;

	OLED.periphOLED.isI2C = False ;

	// give *ptr[0 ... 2] address of arrs until work around is found
	ptr[0] = &humidityStr[0] ;
	ptr[1] = &temperatureStr[0] ;
	ptr[2] = &pressureStr[0] ;

	// setup OLED
	initOLEDParams(&OLED) ;
	resetOLED(&OLED, &hspi2) ;
	initOLED_buffer(&OLED, &hspi2) ;
	clearOLED(&OLED, &hspi2) ;
	createBorderOLED(&OLED, &hspi2) ;


  /* Infinite loop */
  for(;;)
  {
	  // check mutex to dim OLED panel
	  if(osMutexAcquire(adc_oledHandle, 0U) == osOK){
		  // dim OLED panel
		  OLED_changeContrast(&OLED, &hspi2, dimmingVal_g) ;
		  // release mutex
		  osMutexRelease(adc_oledHandle) ;
	  }

	  // get number of messages in queue
	  msgLen = osMessageQueueGetCount(QueuePeriphsHandle) ;

	  // enter only when queue has messages
	  if(msgLen > 0){
		  // go through all messages sent from BME280 task
		  for(uint8_t i = 0; i < (msgLen >> 1); i++){	// 1/2 messages for type, 1/2 messages for data
			  // get message from queue
			  osMessageQueueGet(QueuePeriphsHandle, &var, NULL, 0U) ;

			  switch(var){
				  case TEMP_CONV:		// begin converting temperature value
					  osMessageQueueGet(QueuePeriphsHandle, &tempVarConv, NULL, 0U) ;
					  //sprintf(temperatureStr, "%i\n", (int32_t)tempVarConv) ;
					  tempVarConv = ((tempVarConv - 32) * 5) / 9 ;
					  floatToStr(tempVarConv, temperatureStr, 1) ;
					  ptr[1] = temperatureStr ;
					  break ;

				  case HUMID_CONV:		// begin converting humidity value
					  osMessageQueueGet(QueuePeriphsHandle, &tempVarConv, NULL, 0U) ;
					  //sprintf(humidityStr, "%i\n", (int32_t)tempVarConv) ;
					  floatToStr(tempVarConv, humidityStr, 1) ;
					  ptr[0] = humidityStr ;
					  break ;

				  case PRESSURE_CONV:	// begin converting pressure value
					  osMessageQueueGet(QueuePeriphsHandle, &tempVarConv, NULL, 0U) ;
					  sprintf(pressureStr, "%i\n", (int32_t)tempVarConv) ;
					  ptr[2] = pressureStr ;
					  break ;

				  default:				// invalid entry ... must only be number
					  __asm("nop") ;
					  break ;
			  }
		  }

		  ptr[0] = &humidityStr[0] ;
		  ptr[1] = &temperatureStr[0] ;
		  ptr[2] = &pressureStr[0] ;

		  OLED_printStr(&OLED, &hspi2, ptr) ;
	  }

	  osThreadYield() ;
	  osDelay(OLED_PERIOD_MS);	// 1000 --> 40
  }
  osThreadTerminate(StartTaskOLED) ;
  /* USER CODE END StartTaskOLED */
}

/* USER CODE BEGIN Header_StartTaskADC */
/**
* @brief Function implementing the TaskADC thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskADC */
void StartTaskADC(void *argument)
{
  /* USER CODE BEGIN StartTaskADC */
	// adc value and cnt variable
	static uint8_t tempDMA = 0, i = 0 ;
	static uint16_t valDMA = 0 ;
	// status for mutex ... used for debugging purposes
	osStatus_t stat ;

  /* Infinite loop */
  for(;;)
  {
	  // read from DMA
	  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&tempDMA, 1) ;
	  valDMA += tempDMA ;

	  i++ ;

	  if(i >= 4){
		  // take mutex for adc dma read
		  stat = osMutexAcquire(adc_oledHandle, 0U) ;

		  if(stat == osOK){
			  // take average (valDMA / 4)
			  dimmingVal_g = valDMA >> 2 ;
			  // reset valDMA for next round of conversions
			  valDMA = 0 ;
			  // reset i
			  i = 0 ;
			  // release mutex
			  stat = osMutexRelease(adc_oledHandle) ;
		  }
	  }

	  osThreadYield() ;
	  osDelay(ADC_PERIOD_MS);
  }
  osThreadTerminate(StartTaskADC) ;
  /* USER CODE END StartTaskADC */
}

/* BME280_Callback function */
void BME280_Callback(void *argument)
{
  /* USER CODE BEGIN BME280_Callback */
	static struct weatherSensor BME280_timer ;

	BME280_timer.periph.devAddr = BME280_ADDR << 1 ;

	// check if BME280 data is ready
	if(BME280_checkStatus(&BME280_timer, &hi2c1) != True){
		// if data is not ready yet, acquire the semaphore and prevent reading in BME280 thread
		osSemaphoreAcquire(BME280_BinSemHandle, 0U) ;
	} else{
		// data is ready to be read, allow BME280 thread to read data
		osSemaphoreRelease(BME280_BinSemHandle) ;
	}

  /* USER CODE END BME280_Callback */
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
