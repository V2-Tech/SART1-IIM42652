/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
#include "math.h"
#include "string.h"
#include <stdbool.h>
#include "stream_buffer.h"
#include "common.h"
#include "DAC7571.h"
#include "IIM42652.h"
#include "RGB.h"
#include "RingBuffer.h"
#include "USBCommProtocol.h"
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
 I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_tx;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;

osThreadId MainTaskHandle;
osThreadId SignalElabTaskHandle;
osThreadId UsbTxTaskHandle;
osThreadId UsbRxTaskHandle;
osThreadId ImuTaskHandle;
osTimerId BlinkTimerHandle;
osTimerId DOTimerHandle;
osTimerId SPIReadTimoutTimerHandle;
osMutexId SPI1MutexHandle;
osSemaphoreId spiBinarySemHandle;
osStaticSemaphoreDef_t spiBinarySemControlBlock;
osSemaphoreId usbBinarySemHandle;
osSemaphoreId fifoBinarySemHandle;
osStaticSemaphoreDef_t fifoBinarySemControlBlock;
/* USER CODE BEGIN PV */
AC1_Config_t userConfig;

static StaticQueue_t usb_RX_QueueStruct;
static uint8_t usb_RX_QueueStorage[ APP_RX_DATA_SIZE * sizeof(uint8_t) ];
QueueHandle_t usb_RX_Queue;

static StaticQueue_t usb_TX_QueueStruct;
static uint8_t usb_TX_QueueStorage[ APP_TX_DATA_SIZE * sizeof(uint8_t) ];
QueueHandle_t usb_TX_Queue;

static StaticQueue_t IMUDataOutputQueueStruct;
static uint8_t IMUDataOutputQueueStorage[IMU_OUTPUT_DATA_QUEUE_SIZE * sizeof(IMU_Data_Buf_t)];
QueueHandle_t IMUDataOutputQueue;

char logBuf[APP_TX_DATA_SIZE];

#define SAMPLE 128U
#define FFT_SIZE SAMPLE/2

uint16_t freq = 1;
float32_t SignalTestBuffer[FFT_SIZE];
float32_t FFTOutBuffer[FFT_SIZE];
float32_t FFTMagOutBuffer[FFT_SIZE];
arm_rfft_fast_instance_f32 S;
arm_status status;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM2_Init(void);
void StartMainTask(void const * argument);
void StartSignalElabTask(void const * argument);
void StartUsbTxTask(void const * argument);
void StartUsbRx(void const * argument);
void StartImuTask(void const * argument);
void BlinkTimerCallback(void const * argument);
void DOTimerCallback(void const * argument);
void SPIReadTimoutTimerCallback(void const * argument);

/* USER CODE BEGIN PFP */
void initAll(void)
{
	#ifdef USB_DEBUG
  	  HAL_Delay(2000);
	#endif

  	osTimerStart(BlinkTimerHandle, BLINK_DELAY_MS/portTICK_PERIOD_MS);
  	arm_rfft_fast_init_f32(&S, IMU_FFT_SIZE);

	//--------------------------------------
	// -------- INITIALIZATIONS ------------
	//--------------------------------------
	//USB
  	OMDProtocolInit(&OMDCommProtocol);
	#ifdef USB_DEBUG
	  sprintf(logBuf, "\r\nStarting initialization of sensor and output devices...");
	  CDC_Transmit_FS((uint8_t *) logBuf, strlen(logBuf));
	#endif

	//IMU
	if (IIM42652_InitBusGuard(&IMU, spiBinarySemHandle) != HAL_OK)
	{
		AC1_StatusWord.IMUInit = false;
		AC1_StatusWord.IMUFault = true;
	}
	if (IIM42652_Init(&IMU, &hspi1, IMU_CS_GPIO_Port, IMU_CS_Pin, &userConfig) == HAL_OK)
	{
	  AC1_StatusWord.IMUInit = true; //IMU initialized
	  sprintf(logBuf, "\r\nIMU initialized...");
	  RGB_SetActColor(&RGB, RGB.rgbVIOLET, ENABLE);
	}
	else
	{
	  sprintf(logBuf, "\r\nIMU initialization failed");
	}
	#ifdef USB_DEBUG
		CDC_Transmit_FS((uint8_t *) logBuf, strlen(logBuf));
		HAL_Delay(500);
	#endif

	//DAC
	DAC7571_Init(&DAC, &hi2c1, DAC_ALIM_VOLTAGE);
	DAC7571_SetOutEnabling(&DAC, 1);
	DAC7571_SetOutVoltage(&DAC, DAC_ALIM_VOLTAGE);
	if (DAC7571_WriteDMA(&DAC) == 0)
	{
	  AC1_StatusWord.DACInit = true; //DAC initialized
	  sprintf(logBuf, "\r\nDAC initialized...");
	  RGB_SetActColor(&RGB, RGB.rgbYELLOW, ENABLE);
	}
	else
	{
	  sprintf(logBuf, "\r\nDAC initialization failed");
	}

	#ifdef USB_DEBUG
		CDC_Transmit_FS((uint8_t *) logBuf, strlen(logBuf));
		HAL_Delay(500);
	#endif

	if (AC1_StatusWord.IMUInit == 1 && AC1_StatusWord.DACInit == 1)
	{
		//All devices have been initialized
		RGB_SetActColor(&RGB, RGB.rgbCYAN, ENABLE);
		HAL_GPIO_WritePin(DO1_GPIO_Port, DO1_Pin, GPIO_PIN_SET);
		AC1_StatusWord.Booted = 1;
		sprintf(logBuf, "\r\nAC1 Initializations completed!\r\n Starting FreeRTOS...");
		#ifdef USB_DEBUG
			CDC_Transmit_FS((uint8_t *) logBuf, strlen(logBuf));
		#endif
	}
	else
	{
		//Device's initialization errors occurred. AC1 will be restarted.
		RGB_SetActColor(&RGB, RGB.rgbRED, ENABLE);
		HAL_GPIO_WritePin(DO1_GPIO_Port, DO1_Pin, GPIO_PIN_RESET);
		sprintf(logBuf, "\r\nAC1 Initializations procedure failed. Restarting...");
		CDC_Transmit_FS((uint8_t *) logBuf, strlen(logBuf));
		HAL_Delay(2000);
		__NVIC_SystemReset();
	}

	/* Start signals analysis */
	AC1_StatusWord.IMUAnalysisON = 1;

	/* Disable USB data output */
	IMU.TelemetryON = 0;
}
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
	_LOAD_AC1_SETTINGS(&userConfig); //TODO: work only at the top of application: why?!?!
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
  MX_SPI1_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_RTC_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  AC1_StatusWord.Power = true; //MCU Powered

  //Power LED ON
  RGB_Init(&RGB, &htim1);
  RGB_SetActColor(&RGB, RGB.rgbGREEN, ENABLE);
  HAL_Delay(500);

  //Start used timer
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim5);

  /* USER CODE END 2 */

  /* Create the mutex(es) */
  /* definition and creation of SPI1Mutex */
  osMutexDef(SPI1Mutex);
  SPI1MutexHandle = osMutexCreate(osMutex(SPI1Mutex));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of spiBinarySem */
  osSemaphoreStaticDef(spiBinarySem, &spiBinarySemControlBlock);
  spiBinarySemHandle = osSemaphoreCreate(osSemaphore(spiBinarySem), 1);

  /* definition and creation of usbBinarySem */
  osSemaphoreDef(usbBinarySem);
  usbBinarySemHandle = osSemaphoreCreate(osSemaphore(usbBinarySem), 1);

  /* definition and creation of fifoBinarySem */
  osSemaphoreStaticDef(fifoBinarySem, &fifoBinarySemControlBlock);
  fifoBinarySemHandle = osSemaphoreCreate(osSemaphore(fifoBinarySem), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of BlinkTimer */
  osTimerDef(BlinkTimer, BlinkTimerCallback);
  BlinkTimerHandle = osTimerCreate(osTimer(BlinkTimer), osTimerPeriodic, NULL);

  /* definition and creation of DOTimer */
  osTimerDef(DOTimer, DOTimerCallback);
  DOTimerHandle = osTimerCreate(osTimer(DOTimer), osTimerOnce, NULL);

  /* definition and creation of SPIReadTimoutTimer */
  osTimerDef(SPIReadTimoutTimer, SPIReadTimoutTimerCallback);
  SPIReadTimoutTimerHandle = osTimerCreate(osTimer(SPIReadTimoutTimer), osTimerOnce, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  usb_RX_Queue = xQueueCreateStatic(APP_RX_DATA_SIZE, sizeof(uint8_t), usb_RX_QueueStorage, &usb_RX_QueueStruct);
  usb_TX_Queue = xQueueCreateStatic(APP_TX_DATA_SIZE, sizeof(uint8_t), usb_TX_QueueStorage, &usb_TX_QueueStruct);
  IMUDataOutputQueue = xQueueCreateStatic(IMU_OUTPUT_DATA_QUEUE_SIZE, sizeof(IMU_Data_Buf_t), IMUDataOutputQueueStorage, &IMUDataOutputQueueStruct);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of MainTask */
  osThreadDef(MainTask, StartMainTask, osPriorityHigh, 0, 256);
  MainTaskHandle = osThreadCreate(osThread(MainTask), NULL);

  /* definition and creation of SignalElabTask */
  osThreadDef(SignalElabTask, StartSignalElabTask, osPriorityNormal, 0, 2048);
  SignalElabTaskHandle = osThreadCreate(osThread(SignalElabTask), NULL);

  /* definition and creation of UsbTxTask */
  osThreadDef(UsbTxTask, StartUsbTxTask, osPriorityAboveNormal, 0, 512);
  UsbTxTaskHandle = osThreadCreate(osThread(UsbTxTask), NULL);

  /* definition and creation of UsbRxTask */
  osThreadDef(UsbRxTask, StartUsbRx, osPriorityIdle, 0, 256);
  UsbRxTaskHandle = osThreadCreate(osThread(UsbRxTask), NULL);

  /* definition and creation of ImuTask */
  osThreadDef(ImuTask, StartImuTask, osPriorityRealtime, 0, 128);
  ImuTaskHandle = osThreadCreate(osThread(ImuTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

  /* USER CODE END RTOS_THREADS */

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.ClockSpeed = 400000;
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
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1024-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 9600;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 10000;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 9600;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 96-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, IMU_CS_Pin|DO1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : ACC_INT2_Pin ACC_INT1_Pin */
  GPIO_InitStruct.Pin = ACC_INT2_Pin|ACC_INT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : IMU_CS_Pin */
  GPIO_InitStruct.Pin = IMU_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(IMU_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DO1_Pin */
  GPIO_InitStruct.Pin = DO1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(DO1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

}

/* USER CODE BEGIN 4 */
unsigned long getRunTimeCounterValue(void)
{
	return TIM2->CNT;
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if (hspi->Instance == SPI1)
	{
		IIM42652_GetFIFOData_DMA_Complete(&IMU);
//		__IMU_INT1_ON();
//		xTaskNotifyFromISR(ImuTaskHandle, AC1_Notify_NewDataAvaiable, eSetBits, &xHigherPriorityTaskWoken);
		xTaskNotifyFromISR(SignalElabTaskHandle, AC1_Notify_NewDataAvaiable, eSetBits, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if (hi2c->Instance == I2C1)
	{
		DAC7571_WriteDMA_Complete(&DAC);
		DAC7571_TestOutput(&DAC);
	}
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if (GPIO_Pin == ACC_INT1_Pin)
	{
		if (AC1_StatusWord.IMUAnalysisON == 1 && IMU.m_FIFO_reading == 0)
		{
//			__IMU_INT1_OFF();
			IIM42652_GetFIFOData_DMA(&IMU);
			osTimerStart(SPIReadTimoutTimerHandle, SPI_READ_TIMEOUT_MS);
		}
	}
	if (GPIO_Pin == ACC_INT2_Pin)
	{
		if (AC1_StatusWord.IMUAnalysisON == 1)
		{
			IMU.m_INT2State = 1;
			HAL_GPIO_WritePin(DO1_GPIO_Port, DO1_Pin, GPIO_PIN_SET);
			osTimerStart(DOTimerHandle, DO_PULSE_DUR_MS/portTICK_PERIOD_MS);
		}
	}
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartMainTask */
/**
  * @brief  Function implementing the MainTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartMainTask */
void StartMainTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */
  BaseType_t xResult;
  uint32_t ulNotifiedValue;
  vc_vector* sendData = vc_vector_create(3, sizeof(uint8_t), NULL);
  uint8_t tempVal = 0;

  if (!AC1_StatusWord.Booted)
  {
	  /* Semaphores has been created "taken" so we need to "give" them
	   * to be able to access to guarded bus used on the next initialization function calls
	   */
	  osSemaphoreRelease(spiBinarySemHandle);
	  osSemaphoreRelease(usbBinarySemHandle);
	  osSemaphoreRelease(fifoBinarySemHandle);
	  initAll();

	  IIM42652_GetFIFOData_DMA(&IMU);
	  osTimerStart(SPIReadTimoutTimerHandle, SPI_READ_TIMEOUT_MS);
  }
  /* Infinite loop */
  for(;;)
  {
	  xResult = xTaskNotifyWait( pdFALSE,		/* Don't clear bits on entry. */
			  ULONG_MAX,			/* Clear all bits on exit. */
			  &ulNotifiedValue, 	/* Stores the notified value. */
			  portMAX_DELAY );

	  if( xResult == pdPASS )
	  {
		  if(BitCheckW(ulNotifiedValue, AC1_Notify_AnalysisON_Bit) == 1)
		  {
			  RGB_SetActColor(&RGB, RGB.rgbCYAN, ENABLE);
			  __IMU_TURN_ON(&IMU);
			  AC1_StatusWord.IMUAnalysisON = true;

			  //Send ack to configurator
			  tempVal = commandSetConfigMode;
			  vc_vector_push_back(sendData, &tempVal);
			  ackCommand(&OMDCommProtocol, sendData, OMD_COMM_ACK_OK);
		  }
		  if(BitCheckW(ulNotifiedValue, AC1_Notify_AnalysisOFF_Bit) == 1)
		  {
			  RGB_SetActColor(&RGB, RGB.rgbYELLOW, ENABLE);
			  AC1_StatusWord.IMUAnalysisON = false;
			  __IMU_TURN_OFF(&IMU);

			  //Send ack to configurator
			  tempVal = commandSetConfigMode;
			  vc_vector_push_back(sendData, &tempVal);
			  ackCommand(&OMDCommProtocol, sendData, OMD_COMM_ACK_OK);
		  }
		  if(BitCheckW(ulNotifiedValue, AC1_Notify_SaveConfig_Bit) == 1)
		  {
			  if (_SAVE_AC1_SETTINGS(userConfig) == HAL_OK)
			  {
				  //Send ack to configurator
				  tempVal = commandSaveUserData;
				  vc_vector_push_back(sendData, &tempVal);
				  ackCommand(&OMDCommProtocol, sendData, OMD_COMM_ACK_OK);
			  }
		  }
	  }
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartSignalElabTask */
/**
* @brief Function implementing the SignalElabTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSignalElabTask */
void StartSignalElabTask(void const * argument)
{
  /* USER CODE BEGIN StartSignalElabTask */
  /* Infinite loop */
  for(;;)
  {
	  BaseType_t xResult;
	  uint32_t ulNotifiedValue;
	  static uint8_t xFirst = 0;
	  static float32_t ImuFFTOut[6][IMU_FFT_SIZE];
	  static float32_t ImuFFTMagOut[6][IMU_FFT_SIZE];

	  xResult = xTaskNotifyWait( pdFALSE,    /* Don't clear bits on entry. */
			  ULONG_MAX,        /* Clear all bits on exit. */
			  &ulNotifiedValue, /* Stores the notified value. */
			  portMAX_DELAY );

	  if( xResult == pdPASS )
	  {
		  if (BitCheckW(ulNotifiedValue, AC1_Notify_NewDataAvaiable_Bit) == 1 )
		  {
			  ;
		  }
		  if (RINGBUFFER_FULL(&AccXBuffer))
		  {
			  arm_rfft_fast_f32(&S, (float32_t *)AccXBuffer.buffer, (float32_t *)ImuFFTOut[0], 0);
			  arm_cmplx_mag_f32(ImuFFTOut[0], ImuFFTMagOut[0], IMU_FFT_SIZE);
			  RINGBUFFER_CLEAR(&AccXBuffer);
		  }
		  if (RINGBUFFER_FULL(&AccYBuffer))
		  {
			  arm_rfft_fast_f32(&S, (float32_t *)AccYBuffer.buffer, (float32_t *)ImuFFTOut[1], 0);
			  arm_cmplx_mag_f32(ImuFFTOut[1], ImuFFTMagOut[1], IMU_FFT_SIZE);
			  RINGBUFFER_CLEAR(&AccYBuffer);
		  }
		  if (RINGBUFFER_FULL(&AccZBuffer))
		  {
			  arm_rfft_fast_f32(&S, (float32_t *)AccZBuffer.buffer, (float32_t *)ImuFFTOut[2], 0);
			  arm_cmplx_mag_f32(ImuFFTOut[2], ImuFFTMagOut[2], IMU_FFT_SIZE);
			  RINGBUFFER_CLEAR(&AccZBuffer);
		  }
		  if (RINGBUFFER_FULL(&GyroXBuffer))
		  {
			  arm_rfft_fast_f32(&S, (float32_t *)GyroXBuffer.buffer, (float32_t *)ImuFFTOut[3], 0);
			  arm_cmplx_mag_f32(ImuFFTOut[3], ImuFFTMagOut[3], IMU_FFT_SIZE);
			  RINGBUFFER_CLEAR(&GyroXBuffer);
		  }
		  if (RINGBUFFER_FULL(&GyroYBuffer))
		  {
			  arm_rfft_fast_f32(&S, (float32_t *)GyroYBuffer.buffer, (float32_t *)ImuFFTOut[4], 0);
			  arm_cmplx_mag_f32(ImuFFTOut[4], ImuFFTMagOut[4], IMU_FFT_SIZE);
			  RINGBUFFER_CLEAR(&GyroYBuffer);
		  }
		  if (RINGBUFFER_FULL(&GyroZBuffer))
		  {
			  arm_rfft_fast_f32(&S, (float32_t *)GyroZBuffer.buffer, (float32_t *)ImuFFTOut[5], 0);
			  arm_cmplx_mag_f32(ImuFFTOut[5], ImuFFTMagOut[5], IMU_FFT_SIZE);
			  RINGBUFFER_CLEAR(&GyroZBuffer);
		  }
		  if ((ImuFFTMagOut[0][5] > 20) | (ImuFFTMagOut[1][5] > 20) | (ImuFFTMagOut[3][5] > 20))
		  {
//			  xTaskGenericNotify(MainTaskHandle, AC1_Notify_INT2Event, eSetBits, NULL);
		  }
	  }
    osDelay(1);
  }
  /* USER CODE END StartSignalElabTask */
}

/* USER CODE BEGIN Header_StartUsbTxTask */
/**
* @brief Function implementing the UsbTxTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUsbTxTask */
void StartUsbTxTask(void const * argument)
{
  /* USER CODE BEGIN StartUsbTxTask */
	vc_vector* transmitData = vc_vector_create(2, sizeof(uint8_t), NULL);

  /* Infinite loop */
  for(;;)
  {
	  BaseType_t xResult;
	  uint32_t ulNotifiedValue;
	  IMU_Data_Buf_t imuPacketBuf;
	  size_t encBytes = 0;
	  uint8_t tempByte = 0;

	  xResult = xTaskNotifyWait( pdFALSE,    /* Don't clear bits on entry. */
			  ULONG_MAX,        /* Clear all bits on exit. */
			  &ulNotifiedValue, /* Stores the notified value. */
			  portMAX_DELAY );

	  if( xResult == pdPASS )
	  {
		  while(xQueueReceive(IMUDataOutputQueue, imuPacketBuf.buffer, 0) == pdPASS )
		  {
			  uint8_t imuEncodedBuffer[sizeof(IMU_Data_Buf_t) + COMMAND_DATA_SIZE + ENCODEING_EXTRA_SIZE];
			  memset(imuEncodedBuffer, 0x00, sizeof(imuEncodedBuffer));

			  encBytes = imuUsbPacketsCreator(&IMU, &imuPacketBuf, sizeof(imuPacketBuf.buffer),
					  imuEncodedBuffer, sizeof(imuEncodedBuffer));

			  if (encBytes>0)
			  {
//				  CDC_Transmit_FS(imuEncodedBuffer, encBytes);
				  _USB_SEND_OPT(imuEncodedBuffer, encBytes);
			  }
		  }

		  while(xQueueReceive(usb_TX_Queue, &tempByte, 0) == pdPASS )
		  {
			  vc_vector_push_back(transmitData, &tempByte);
		  }
		  if (!vc_vector_empty(transmitData))
		  {
			  CDC_Transmit_FS(vc_vector_begin(transmitData), vc_vector_count(transmitData));
//			  _USB_SEND_OPT(vc_vector_begin(transmitData), vc_vector_count(transmitData));
			  vc_vector_clear(transmitData);
		  }
	  }
    osDelay(1);
  }
  /* USER CODE END StartUsbTxTask */
}

/* USER CODE BEGIN Header_StartUsbRx */
/**
* @brief Function implementing the UsbRxTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUsbRx */
void StartUsbRx(void const * argument)
{
  /* USER CODE BEGIN StartUsbRx */
	uint8_t tempByte;
	uint8_t newReceivedData = 0;

	vc_vector* receivedData = vc_vector_create(3, sizeof(uint8_t), NULL);
	/* Infinite loop */
	for(;;)
	{
		BaseType_t xResult;
		uint32_t ulNotifiedValue;

//		xResult = xTaskNotifyWait( pdFALSE,    /* Don't clear bits on entry. */
//				ULONG_MAX,        /* Clear all bits on exit. */
//				&ulNotifiedValue, /* Stores the notified value. */
//				portMAX_DELAY );
//
//		if( xResult == pdPASS )
//		{
//			while(xQueueReceive(usb_RX_Queue, &tempByte, 0) == pdPASS )
//			{
//				vc_vector_push_back(receivedData, &tempByte);
//				tempByte = 1;
//				newReceivedData = 1;
//			}
//			if (!vc_vector_empty(receivedData) && (newReceivedData == 1))
//			{
//				newReceivedData = 0;
//				GiveRxData(&OMDCommProtocol, receivedData);
//			}
//		}

		while(xQueueReceive(usb_RX_Queue, &tempByte, portMAX_DELAY) == pdPASS )
		{
			vc_vector_push_back(receivedData, &tempByte);
			GiveRxData(&OMDCommProtocol, receivedData);
		}
		osDelay(1);
	}
  /* USER CODE END StartUsbRx */
}

/* USER CODE BEGIN Header_StartImuTask */
/**
* @brief Function implementing the ImuTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartImuTask */
void StartImuTask(void const * argument)
{
  /* USER CODE BEGIN StartImuTask */
  /* Infinite loop */
  for(;;)
  {
	  BaseType_t xResult;
	  uint32_t ulNotifiedValue;
	  static uint8_t FIFOFulled = 0;
	  uint8_t state = 0;

	  xResult = xTaskNotifyWait( pdFALSE,    /* Don't clear bits on entry. */
			  ULONG_MAX,        /* Clear all bits on exit. */
			  &ulNotifiedValue, /* Stores the notified value. */
			  portMAX_DELAY );

	  if( xResult == pdPASS )
	  {
		  if (BitCheckW(ulNotifiedValue, AC1_Notify_NewDataAvaiable_Bit) == 1 )
		  {
			  IIM42652_GetFIFOData_DMA(&IMU);
			  osTimerStart(SPIReadTimoutTimerHandle, SPI_READ_TIMEOUT_MS);

			  if (IMU.m_FIFO_Count >= 127)
			  {
				  RGB_SetActColor(&RGB, RGB.rgbRED, ENABLE);
				  FIFOFulled++;
			  }
			  else
			  {
				  if (FIFOFulled)
				  {
					  RGB_SetActColor(&RGB, RGB.rgbCYAN, ENABLE);
					  FIFOFulled = 0;
				  }
			  }
		  }
	  }
    osDelay(1);
  }
  /* USER CODE END StartImuTask */
}

/* BlinkTimerCallback function */
void BlinkTimerCallback(void const * argument)
{
  /* USER CODE BEGIN BlinkTimerCallback */
	RGB_Blink(&RGB);
  /* USER CODE END BlinkTimerCallback */
}

/* DOTimerCallback function */
void DOTimerCallback(void const * argument)
{
  /* USER CODE BEGIN DOTimerCallback */
	HAL_GPIO_WritePin(DO1_GPIO_Port, DO1_Pin, GPIO_PIN_RESET);
	IMU.m_INT2State = 0;
  /* USER CODE END DOTimerCallback */
}

/* SPIReadTimoutTimerCallback function */
void SPIReadTimoutTimerCallback(void const * argument)
{
  /* USER CODE BEGIN SPIReadTimoutTimerCallback */
	if (AC1_StatusWord.IMUAnalysisON)
	{
		RGB_SetActColor(&RGB, RGB.rgbVIOLET, ENABLE);
		IIM42652_GetFIFOData_DMA(&IMU);
		osTimerStart(SPIReadTimoutTimerHandle, SPI_READ_TIMEOUT_MS);
	}
  /* USER CODE END SPIReadTimoutTimerCallback */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM10 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM10) {
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
