/* USER CODE BEGIN Header */
/**
 * This whole project is in sync with its *.ioc file, and this includes the
 * ability to regenerate code. One thing that is not fully implemented is
 * handling of the touch screen input. Further on it will be added using default
 * BSP library. On the topic o BSP -- implementation of BSP in this project is a
 * custom implementation (it is not far from the original, but some minor tweaks
 * were made).
 */

/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdio.h>
#include "stm32f746g_discovery_lcd.h"
//#include "stm32746g_discovery_lcd.h"
#include "stm32746g_discovery_ts.h"
//#include "stm32f746g_discovery_ts_remote.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
typedef StaticQueue_t osStaticMessageQDef_t;
typedef StaticTimer_t osStaticTimerDef_t;
typedef StaticSemaphore_t osStaticMutexDef_t;
typedef StaticSemaphore_t osStaticSemaphoreDef_t;
typedef StaticEventGroup_t osStaticEventGroupDef_t;
/* USER CODE BEGIN PTD */

// TODO Rozwinąć funkcjonalnosć na potrzeby przekazywania nowych parametrów?
#define TYPE_NONE 0
#define TYPE_U1 2
#define TYPE_U2 3
#define TYPE_Y1 4
#define TYPE_Y2 5
#define TYPE_Z1 8
#define TYPE_Z2 9
typedef struct {
	uint8_t type;
	float value;
} Message;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc3;

DMA2D_HandleTypeDef hdma2d;

LTDC_HandleTypeDef hltdc;

UART_HandleTypeDef huart1;

SDRAM_HandleTypeDef hsdram1;

/* Definitions for ltdcDriver */
osThreadId_t ltdcDriverHandle;
uint32_t ltdcDriverBuffer[ 1024 ];
osStaticThreadDef_t ltdcDriverControlBlock;
const osThreadAttr_t ltdcDriver_attributes = {
  .name = "ltdcDriver",
  .cb_mem = &ltdcDriverControlBlock,
  .cb_size = sizeof(ltdcDriverControlBlock),
  .stack_mem = &ltdcDriverBuffer[0],
  .stack_size = sizeof(ltdcDriverBuffer),
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for processDriver */
osThreadId_t processDriverHandle;
uint32_t processDriverBuffer[ 512 ];
osStaticThreadDef_t processDriverControlBlock;
const osThreadAttr_t processDriver_attributes = {
  .name = "processDriver",
  .cb_mem = &processDriverControlBlock,
  .cb_size = sizeof(processDriverControlBlock),
  .stack_mem = &processDriverBuffer[0],
  .stack_size = sizeof(processDriverBuffer),
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for tsDriver */
osThreadId_t tsDriverHandle;
uint32_t tsDriverBuffer[ 512 ];
osStaticThreadDef_t tsDriverControlBlock;
const osThreadAttr_t tsDriver_attributes = {
  .name = "tsDriver",
  .cb_mem = &tsDriverControlBlock,
  .cb_size = sizeof(tsDriverControlBlock),
  .stack_mem = &tsDriverBuffer[0],
  .stack_size = sizeof(tsDriverBuffer),
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for controllerV */
osThreadId_t controllerVHandle;
uint32_t controllerVBuffer[ 512 ];
osStaticThreadDef_t controllerVControlBlock;
const osThreadAttr_t controllerV_attributes = {
  .name = "controllerV",
  .cb_mem = &controllerVControlBlock,
  .cb_size = sizeof(controllerVControlBlock),
  .stack_mem = &controllerVBuffer[0],
  .stack_size = sizeof(controllerVBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for controllerH */
osThreadId_t controllerHHandle;
uint32_t controllerHBuffer[ 512 ];
osStaticThreadDef_t controllerHControlBlock;
const osThreadAttr_t controllerH_attributes = {
  .name = "controllerH",
  .cb_mem = &controllerHControlBlock,
  .cb_size = sizeof(controllerHControlBlock),
  .stack_mem = &controllerHBuffer[0],
  .stack_size = sizeof(controllerHBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};

/* Definitions for staticAnimation */
osThreadId_t staticAnimationHandle;
uint32_t staticAnimationBuffer[ 512 ];
osStaticThreadDef_t staticAnimationControlBlock;
const osThreadAttr_t staticAnimation_attributes = {
  .name = "staticAnimation",
  .cb_mem = &staticAnimationControlBlock,
  .cb_size = sizeof(staticAnimationControlBlock),
  .stack_mem = &staticAnimationBuffer[0],
  .stack_size = sizeof(staticAnimationBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for qProcess */
osMessageQueueId_t qProcessHandle;
uint8_t qProcessBuffer[ 16 * sizeof( Message ) ];
osStaticMessageQDef_t qProcessControlBlock;
const osMessageQueueAttr_t qProcess_attributes = {
  .name = "qProcess",
  .cb_mem = &qProcessControlBlock,
  .cb_size = sizeof(qProcessControlBlock),
  .mq_mem = &qProcessBuffer,
  .mq_size = sizeof(qProcessBuffer)
};
/* Definitions for qLTDCReady */
osMessageQueueId_t qLTDCReadyHandle;
uint8_t qLTDCReadyBuffer[ 1 * sizeof( uint16_t ) ];
osStaticMessageQDef_t qLTDCReadyControlBlock;
const osMessageQueueAttr_t qLTDCReady_attributes = {
  .name = "qLTDCReady",
  .cb_mem = &qLTDCReadyControlBlock,
  .cb_size = sizeof(qLTDCReadyControlBlock),
  .mq_mem = &qLTDCReadyBuffer,
  .mq_size = sizeof(qLTDCReadyBuffer)
};
/* Definitions for qTSState */
osMessageQueueId_t qTSStateHandle;
uint8_t qTSStateBuffer[ 16 * sizeof( TS_StateTypeDef ) ];
osStaticMessageQDef_t qTSStateControlBlock;
const osMessageQueueAttr_t qTSState_attributes = {
  .name = "qTSState",
  .cb_mem = &qTSStateControlBlock,
  .cb_size = sizeof(qTSStateControlBlock),
  .mq_mem = &qTSStateBuffer,
  .mq_size = sizeof(qTSStateBuffer)
};
/* Definitions for qControllerV */
osMessageQueueId_t qControllerVHandle;
uint8_t qControllerVBuffer[ 16 * sizeof( Message ) ];
osStaticMessageQDef_t qControllerVControlBlock;
const osMessageQueueAttr_t qControllerV_attributes = {
  .name = "qControllerV",
  .cb_mem = &qControllerVControlBlock,
  .cb_size = sizeof(qControllerVControlBlock),
  .mq_mem = &qControllerVBuffer,
  .mq_size = sizeof(qControllerVBuffer)
};

/* Definitions for qControllerH */
osMessageQueueId_t qControllerHHandle;
uint8_t qControllerHBuffer[ 16 * sizeof( Message ) ];
osStaticMessageQDef_t qControllerHControlBlock;
const osMessageQueueAttr_t qControllerH_attributes = {
  .name = "qControllerH",
  .cb_mem = &qControllerHControlBlock,
  .cb_size = sizeof(qControllerHControlBlock),
  .mq_mem = &qControllerHBuffer,
  .mq_size = sizeof(qControllerHBuffer)
};

/* Definitions for process */
osTimerId_t processHandle;
osTimerId_t controllerVProcessHandle;
osTimerId_t controllerHProcessHandle;

osStaticTimerDef_t processControlBlock;
const osTimerAttr_t process_attributes = {
  .name = "process",
  .cb_mem = &processControlBlock,
  .cb_size = sizeof(processControlBlock),
};
/* Definitions for mProcess */
osMutexId_t mProcessHandle;
osStaticMutexDef_t mProcessControlBlock;
const osMutexAttr_t mProcess_attributes = {
  .name = "mProcess",
  .cb_mem = &mProcessControlBlock,
  .cb_size = sizeof(mProcessControlBlock),
};
/* Definitions for sLTDC */
osSemaphoreId_t sLTDCHandle;
osStaticSemaphoreDef_t sLTDCControlBlock;
const osSemaphoreAttr_t sLTDC_attributes = {
  .name = "sLTDC",
  .cb_mem = &sLTDCControlBlock,
  .cb_size = sizeof(sLTDCControlBlock),
};
/* Definitions for eLTDCReady */
osEventFlagsId_t eLTDCReadyHandle;
osStaticEventGroupDef_t eLTDCReadyControlBlock;
const osEventFlagsAttr_t eLTDCReady_attributes = {
  .name = "eLTDCReady",
  .cb_mem = &eLTDCReadyControlBlock,
  .cb_size = sizeof(eLTDCReadyControlBlock),
};
/* USER CODE BEGIN PV */
uint32_t ADC3_buffer[2] = { 0 }; /* !REMOTE */

uint8_t buffer_id = 0;
uint32_t layer[2] = { 0xC0000000 + 480 * 272 * 4 * 0, 0xC0000000
		+ 480 * 272 * 4 * 2 };
char LCD_Text0[100] = "LCD_TEST0";
char LCD_Text1[100] = "LCD_TEST1";
char LCD_Text2[100] = "LCD_TEST2";
TS_StateTypeDef TS_State = { 0 };

float cu1 = 0.0f;
float cu2 = 0.0f;
float cy1 = 0.0f;
float cy2 = 0.0f;
float cz1 = -0.25f;
float cz2 = 0.50f;

uint16_t Xlist[4] = {150, 100, 200, 100};
uint16_t Ylist[4] = {100, 50, 100, 150};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_LTDC_Init(void);
static void MX_FMC_Init(void);
static void MX_DMA2D_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC3_Init(void);
static void MX_USART1_UART_Init(void);
void task_ltdcDriver(void *argument);
void task_processDriver(void *argument);
void task_tsDriver(void *argument);
void task_controllerV(void *argument);
void task_controllerH(void *argument);
void task_staticAnimation(void *argument);
void timer_process(void *argument);

/* USER CODE BEGIN PFP */
void swap_buffers(void) {
	BSP_LCD_SetLayerAddress(0, layer[0]);
	BSP_LCD_SetLayerAddress(1, layer[1]);
	buffer_id = buffer_id == 0 ? 1 : 0;

	if (buffer_id == 1) {
		layer[0] = 0xC0000000 + 480 * 272 * 4 * (0 + 0);
		layer[1] = 0xC0000000 + 480 * 272 * 4 * (0 + 2);
	} else {
		layer[0] = 0xC0000000 + 480 * 272 * 4 * (1 + 0);
		layer[1] = 0xC0000000 + 480 * 272 * 4 * (1 + 2);
	}
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == LCD_INT_Pin) {
		BSP_TS_GetState(&TS_State);
		osMessageQueuePut(qTSStateHandle, &TS_State, 0, 0);
	}
}

void HAL_LTDC_LineEventCallback(LTDC_HandleTypeDef *hltdc) {
	if (osSemaphoreAcquire(sLTDCHandle, 0) == osOK) {
		swap_buffers();
		osSemaphoreRelease(sLTDCHandle);

		osEventFlagsSet(eLTDCReadyHandle, 0x00000001U);
	}
	HAL_LTDC_ProgramLineEvent(hltdc, 272);
}
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
	HAL_Delay(100); /*! Delay so that LCD will not restart during initialisation !*/
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_LTDC_Init();
  MX_FMC_Init();
  MX_DMA2D_Init();
  MX_ADC1_Init();
  MX_ADC3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	BSP_TS_Init(0, 0); // initialisation of TouchScreen -- arguments are irrelevant
	  BSP_TS_ITConfig(); // to cancel exti interrupts from the touch screen comment this line

	HAL_ADC_Start_DMA(&hadc3, ADC3_buffer, 2);
	HAL_Delay(100); // wait for everything to set up before the controller loop starts

	HAL_LTDC_ProgramLineEvent(&hltdc, 272);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of mProcess */
  mProcessHandle = osMutexNew(&mProcess_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of sLTDC */
  sLTDCHandle = osSemaphoreNew(1, 1, &sLTDC_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of process */
  processHandle = osTimerNew(timer_process, osTimerPeriodic, NULL, &process_attributes);
  controllerVProcessHandle = osTimerNew(task_controllerV, osTimerPeriodic, NULL, &process_attributes);
  controllerHProcessHandle = osTimerNew(task_controllerH, osTimerPeriodic, NULL, &process_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  // LOOKATME Tutaj należy dodać konfigurację timerów (ich okres odpalenia)
	osTimerStart(processHandle, 10);
  osTimerStart(controllerVProcessHandle, 200);
  osTimerStart(controllerHProcessHandle, 40);

  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of qProcess */
  qProcessHandle = osMessageQueueNew (16, sizeof(Message), &qProcess_attributes);

  /* creation of qLTDCReady */
  qLTDCReadyHandle = osMessageQueueNew (1, sizeof(uint16_t), &qLTDCReady_attributes);

  /* creation of qTSState */
  qTSStateHandle = osMessageQueueNew (16, sizeof(TS_StateTypeDef), &qTSState_attributes);

  /* creation of qControllerV */
  qControllerVHandle = osMessageQueueNew (16, sizeof(Message), &qControllerV_attributes);

  /* creation of qControllerH */
  qControllerHHandle = osMessageQueueNew (16, sizeof(Message), &qControllerH_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of ltdcDriver */
  ltdcDriverHandle = osThreadNew(task_ltdcDriver, NULL, &ltdcDriver_attributes);

  /* creation of processDriver */
  processDriverHandle = osThreadNew(task_processDriver, NULL, &processDriver_attributes);

  /* creation of tsDriver */
  tsDriverHandle = osThreadNew(task_tsDriver, NULL, &tsDriver_attributes);

  /* creation of controllerV */
  controllerVHandle = osThreadNew(task_controllerV, NULL, &controllerV_attributes);

  /* creation of controllerV */
  controllerHHandle = osThreadNew(task_controllerH, NULL, &controllerH_attributes);

  /* creation of staticAnimation */
  staticAnimationHandle = osThreadNew(task_staticAnimation, NULL, &staticAnimation_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* creation of eLTDCReady */
  eLTDCReadyHandle = osEventFlagsNew(&eLTDCReady_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	while (1) {
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
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
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 432;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
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
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
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
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc3.Init.ContinuousConvMode = ENABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 2;
  hadc3.Init.DMAContinuousRequests = ENABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief DMA2D Initialization Function
  * @param None
  * @retval None
  */
static void MX_DMA2D_Init(void)
{

  /* USER CODE BEGIN DMA2D_Init 0 */

  /* USER CODE END DMA2D_Init 0 */

  /* USER CODE BEGIN DMA2D_Init 1 */

  /* USER CODE END DMA2D_Init 1 */
  hdma2d.Instance = DMA2D;
  hdma2d.Init.Mode = DMA2D_M2M;
  hdma2d.Init.ColorMode = DMA2D_OUTPUT_ARGB8888;
  hdma2d.Init.OutputOffset = 0;
  hdma2d.LayerCfg[1].InputOffset = 0;
  hdma2d.LayerCfg[1].InputColorMode = DMA2D_INPUT_ARGB8888;
  hdma2d.LayerCfg[1].AlphaMode = DMA2D_NO_MODIF_ALPHA;
  hdma2d.LayerCfg[1].InputAlpha = 0;
  if (HAL_DMA2D_Init(&hdma2d) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DMA2D_ConfigLayer(&hdma2d, 1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DMA2D_Init 2 */

  /* USER CODE END DMA2D_Init 2 */

}

/**
  * @brief LTDC Initialization Function
  * @param None
  * @retval None
  */
static void MX_LTDC_Init(void)
{

  /* USER CODE BEGIN LTDC_Init 0 */

  /* USER CODE END LTDC_Init 0 */

  LTDC_LayerCfgTypeDef pLayerCfg = {0};
  LTDC_LayerCfgTypeDef pLayerCfg1 = {0};

  /* USER CODE BEGIN LTDC_Init 1 */

  /* USER CODE END LTDC_Init 1 */
  hltdc.Instance = LTDC;
  hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
  hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
  hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
  hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
  hltdc.Init.HorizontalSync = 40;
  hltdc.Init.VerticalSync = 9;
  hltdc.Init.AccumulatedHBP = 53;
  hltdc.Init.AccumulatedVBP = 11;
  hltdc.Init.AccumulatedActiveW = 533;
  hltdc.Init.AccumulatedActiveH = 283;
  hltdc.Init.TotalWidth = 543;
  hltdc.Init.TotalHeigh = 285;
  hltdc.Init.Backcolor.Blue = 0;
  hltdc.Init.Backcolor.Green = 0;
  hltdc.Init.Backcolor.Red = 0;
  if (HAL_LTDC_Init(&hltdc) != HAL_OK)
  {
    Error_Handler();
  }
  pLayerCfg.WindowX0 = 0;
  pLayerCfg.WindowX1 = 480;
  pLayerCfg.WindowY0 = 0;
  pLayerCfg.WindowY1 = 272;
  pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_ARGB8888;
  pLayerCfg.Alpha = 255;
  pLayerCfg.Alpha0 = 0;
  pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
  pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
  pLayerCfg.FBStartAdress = 0xC0000000;
  pLayerCfg.ImageWidth = 480;
  pLayerCfg.ImageHeight = 272;
  pLayerCfg.Backcolor.Blue = 0;
  pLayerCfg.Backcolor.Green = 0;
  pLayerCfg.Backcolor.Red = 0;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 0) != HAL_OK)
  {
    Error_Handler();
  }
  pLayerCfg1.WindowX0 = 0;
  pLayerCfg1.WindowX1 = 480;
  pLayerCfg1.WindowY0 = 0;
  pLayerCfg1.WindowY1 = 272;
  pLayerCfg1.PixelFormat = LTDC_PIXEL_FORMAT_ARGB8888;
  pLayerCfg1.Alpha = 255;
  pLayerCfg1.Alpha0 = 0;
  pLayerCfg1.BlendingFactor1 = LTDC_BLENDING_FACTOR1_PAxCA;
  pLayerCfg1.BlendingFactor2 = LTDC_BLENDING_FACTOR2_PAxCA;
  pLayerCfg1.FBStartAdress = 0xC0000000+480*272*4;
  pLayerCfg1.ImageWidth = 480;
  pLayerCfg1.ImageHeight = 272;
  pLayerCfg1.Backcolor.Blue = 0;
  pLayerCfg1.Backcolor.Green = 0;
  pLayerCfg1.Backcolor.Red = 0;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg1, 1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LTDC_Init 2 */
	BSP_LCD_LayerDefaultInit(0, pLayerCfg.FBStartAdress);
	BSP_LCD_LayerDefaultInit(1, pLayerCfg1.FBStartAdress);
	/* Assert display enable LCD_DISP pin */
	HAL_GPIO_WritePin(LCD_DISP_GPIO_Port, LCD_DISP_Pin, GPIO_PIN_SET);

	/* Assert backlight LCD_BL_CTRL pin */
	HAL_GPIO_WritePin(LCD_BL_CTRL_GPIO_Port, LCD_BL_CTRL_Pin, GPIO_PIN_SET);
  /* USER CODE END LTDC_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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

/* FMC initialization function */
static void MX_FMC_Init(void)
{

  /* USER CODE BEGIN FMC_Init 0 */
	FMC_SDRAM_CommandTypeDef Command;

  /* USER CODE END FMC_Init 0 */

  FMC_SDRAM_TimingTypeDef SdramTiming = {0};

  /* USER CODE BEGIN FMC_Init 1 */

  /* USER CODE END FMC_Init 1 */

  /** Perform the SDRAM1 memory initialization sequence
  */
  hsdram1.Instance = FMC_SDRAM_DEVICE;
  /* hsdram1.Init */
  hsdram1.Init.SDBank = FMC_SDRAM_BANK1;
  hsdram1.Init.ColumnBitsNumber = FMC_SDRAM_COLUMN_BITS_NUM_8;
  hsdram1.Init.RowBitsNumber = FMC_SDRAM_ROW_BITS_NUM_12;
  hsdram1.Init.MemoryDataWidth = FMC_SDRAM_MEM_BUS_WIDTH_16;
  hsdram1.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
  hsdram1.Init.CASLatency = FMC_SDRAM_CAS_LATENCY_2;
  hsdram1.Init.WriteProtection = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
  hsdram1.Init.SDClockPeriod = FMC_SDRAM_CLOCK_PERIOD_2;
  hsdram1.Init.ReadBurst = FMC_SDRAM_RBURST_ENABLE;
  hsdram1.Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_0;
  /* SdramTiming */
  SdramTiming.LoadToActiveDelay = 2;
  SdramTiming.ExitSelfRefreshDelay = 6;
  SdramTiming.SelfRefreshTime = 4;
  SdramTiming.RowCycleDelay = 6;
  SdramTiming.WriteRecoveryTime = 2;
  SdramTiming.RPDelay = 2;
  SdramTiming.RCDDelay = 2;

  if (HAL_SDRAM_Init(&hsdram1, &SdramTiming) != HAL_OK)
  {
    Error_Handler( );
  }

  /* USER CODE BEGIN FMC_Init 2 */
	__IO uint32_t tmpmrd = 0;
	/* Step 3:  Configure a clock configuration enable command */
	Command.CommandMode = FMC_SDRAM_CMD_CLK_ENABLE;
	Command.CommandTarget = FMC_SDRAM_CMD_TARGET_BANK1;
	Command.AutoRefreshNumber = 1;
	Command.ModeRegisterDefinition = 0;

	/* Send the command */
	HAL_SDRAM_SendCommand(&hsdram1, &Command, SDRAM_TIMEOUT);

	/* Step 4: Insert 100 us minimum delay */
	/* Inserted delay is equal to 1 ms due to systick time base unit (ms) */
	HAL_Delay(1);

	/* Step 5: Configure a PALL (precharge all) command */
	Command.CommandMode = FMC_SDRAM_CMD_PALL;
	Command.CommandTarget = FMC_SDRAM_CMD_TARGET_BANK1;
	Command.AutoRefreshNumber = 1;
	Command.ModeRegisterDefinition = 0;

	/* Send the command */
	HAL_SDRAM_SendCommand(&hsdram1, &Command, SDRAM_TIMEOUT);

	/* Step 6 : Configure a Auto-Refresh command */
	Command.CommandMode = FMC_SDRAM_CMD_AUTOREFRESH_MODE;
	Command.CommandTarget = FMC_SDRAM_CMD_TARGET_BANK1;
	Command.AutoRefreshNumber = 8;
	Command.ModeRegisterDefinition = 0;

	/* Send the command */
	HAL_SDRAM_SendCommand(&hsdram1, &Command, SDRAM_TIMEOUT);

	/* Step 7: Program the external memory mode register */
	tmpmrd = (uint32_t) SDRAM_MODEREG_BURST_LENGTH_1 |
	SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL |
	SDRAM_MODEREG_CAS_LATENCY_2 |
	SDRAM_MODEREG_OPERATING_MODE_STANDARD |
	SDRAM_MODEREG_WRITEBURST_MODE_SINGLE;

	Command.CommandMode = FMC_SDRAM_CMD_LOAD_MODE;
	Command.CommandTarget = FMC_SDRAM_CMD_TARGET_BANK1;
	Command.AutoRefreshNumber = 1;
	Command.ModeRegisterDefinition = tmpmrd;

	/* Send the command */
	HAL_SDRAM_SendCommand(&hsdram1, &Command, SDRAM_TIMEOUT);

	/* Step 8: Set the refresh rate counter */
	/* (15.62 us x Freq) - 20 */
	/* Set the device refresh counter */
	hsdram1.Instance->SDRTR |= ((uint32_t) ((1292) << 1));

  /* USER CODE END FMC_Init 2 */
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
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOJ_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOK_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_BL_CTRL_GPIO_Port, LCD_BL_CTRL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_DISP_GPIO_Port, LCD_DISP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LCD_BL_CTRL_Pin */
  GPIO_InitStruct.Pin = LCD_BL_CTRL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_BL_CTRL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_DISP_Pin */
  GPIO_InitStruct.Pin = LCD_DISP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_DISP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PI11 LCD_INT_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_11|LCD_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pin : TS_TOUCHED_Pin */
  GPIO_InitStruct.Pin = TS_TOUCHED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TS_TOUCHED_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void DrawPointOfTouch(TS_StateTypeDef *TSS) {
	static uint16_t lastx = 0;
	static uint16_t lasty = 0;
	BSP_LCD_SetTextColor(LCD_COLOR_TRANSPARENT);
	BSP_LCD_DrawCircle_AtAddr(lastx, lasty, 3, layer[1]);
	BSP_LCD_DrawCircle_AtAddr(lastx, lasty, 2, layer[1]);
	if (TSS->touchDetected > 0) {
		lastx = TSS->touchX[0];
		lasty = TSS->touchY[0];
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
		BSP_LCD_DrawCircle_AtAddr(lastx, lasty, 3, layer[1]);
		BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
		BSP_LCD_DrawCircle_AtAddr(lastx, lasty, 2, layer[1]);
	}
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_task_ltdcDriver */
/**
 * @brief  Function implementing the ltdcDriver thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_task_ltdcDriver */
void task_ltdcDriver(void *argument)
{
  /* USER CODE BEGIN 5 */
	/* Infinite loop */
	for (;;) {


		osEventFlagsWait(eLTDCReadyHandle, 0x00000001U, osFlagsWaitAny,
		osWaitForever);
		osSemaphoreAcquire(sLTDCHandle, osWaitForever);

		// clear all
		BSP_LCD_SelectLayer(0);
		BSP_LCD_Clear_AtAddr(LCD_COLOR_WHITE, layer[0]);
		BSP_LCD_SelectLayer(1);
		BSP_LCD_Clear_AtAddr(LCD_COLOR_TRANSPARENT, layer[1]);

		BSP_LCD_DrawLine_AtAddr(Xlist[0], Ylist[0], Xlist[1], Ylist[1], layer[0]);
		BSP_LCD_DrawLine_AtAddr(Xlist[1], Ylist[1], Xlist[2], Ylist[2], layer[0]);
		BSP_LCD_DrawLine_AtAddr(Xlist[2], Ylist[2], Xlist[3], Ylist[3], layer[0]);

		// draw on bottom layer
		BSP_LCD_SelectLayer(0); // select colors and fonts for the bottom layer
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
		BSP_LCD_DrawRect_AtAddr(10-5, 10-5, 200+5+5, 200+5+5, layer[0]); // +5 for each margin, so that the "ball" doesn't stick out of the frame
		BSP_LCD_SetTextColor(LCD_COLOR_RED);
		BSP_LCD_FillCircle_AtAddr(cz1 * 100 + 110, cz2 * 100 + 110, 4, layer[0]);
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
		BSP_LCD_DrawCircle_AtAddr(cy1 * 100 + 110, cy2 * 100 + 110, 5, layer[0]);



		// some text lines for debugging purposes
		BSP_LCD_SetFont(&Font12);
		BSP_LCD_DisplayStringAt_AtAddr(10, 225, (uint8_t*) LCD_Text0, LEFT_MODE,
				layer[0]);
		BSP_LCD_DisplayStringAt_AtAddr(10, 240, (uint8_t*) LCD_Text1, LEFT_MODE,
				layer[0]);
		BSP_LCD_DisplayStringAt_AtAddr(10, 255, (uint8_t*) LCD_Text2, LEFT_MODE,
				layer[0]);
		// TODO Rozwinąć funkcjonalność o rysowanie nowych elementów

//		BSP_LCD_DrawRect_AtAddr(150, 100, 20, 30, layer[0]);
//		BSP_LCD_DrawPolygon_AtAddr(list, 4, layer[0]);

		BSP_LCD_DrawLine_AtAddr(Xlist[3], Ylist[3], Xlist[0], Ylist[0], layer[0]);

		// draw on top layer
		BSP_LCD_SelectLayer(1); // select colors and fonts for the top layer
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
		DrawPointOfTouch(&TS_State);

		osSemaphoreRelease(sLTDCHandle);
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_task_processDriver */
/**
 * @brief Function implementing the processDriver thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_task_processDriver */
void task_processDriver(void *argument)
{
  /* USER CODE BEGIN task_processDriver */
	/* Infinite loop */
	Message m = { .type = 0, .value = 0.0f }; // structure initialization -- fancy C syntax
	for (;;) {
		osMessageQueueGet(qProcessHandle, &m, NULL, osWaitForever);
		// analyze message
		switch (m.type) {
		case TYPE_U1:
			cu1 = m.value;
			break;
		case TYPE_U2:
			cu2 = m.value;
			break;
		case TYPE_Y1:
			m.value = cy1;
//			osMessageQueuePut(qControllerHHandle, &m, 0, 0); // FIXME Wskazać gdzie sterownik procesu ma oddać pomiar
			break;
		case TYPE_Y2:
			m.value = cy2;
			osMessageQueuePut(qControllerVHandle, &m, 0, 0);
			break;
		default:
			break;
		}
	}
  /* USER CODE END task_processDriver */
}

/* USER CODE BEGIN Header_task_tsDriver */
/**
 * @brief Function implementing the tsDriver thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_task_tsDriver */
void task_tsDriver(void *argument)
{
  /* USER CODE BEGIN task_tsDriver */
	/* Infinite loop */
	TS_StateTypeDef tss = { 0 };
	for (;;) {
		osMessageQueueGet(qTSStateHandle, &tss, NULL, osWaitForever);
		if (tss.touchDetected > 0 && tss.touchX[0] >= 10 && tss.touchX[0] <= 210
				&& tss.touchY[0] >= 10 && tss.touchY[0] <= 210) {
			cz1 = (tss.touchX[0] - 110) / 100.0f;
			cz2 = (tss.touchY[0] - 110) / 100.0f;
		}
		// TODO Rozwinąć funkcjonalność o obsługę przycisku do zmiany parametrów PIDa
	}
  /* USER CODE END task_tsDriver */
}

/* USER CODE BEGIN Header_task_controllerV */
/**
 * @brief Function implementing the controllerV thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_task_controllerV */
void task_controllerV(void *argument)
{
  /* USER CODE BEGIN task_controllerV */
	// FIXME osDelay nie nadaje się do implementacji stałych okresów próbkowania
	// TODO Dodać możliwość zmiany parametrów z wykorzystaniem kolejki
	/* Infinite loop */
  Message m = {0};
  float y  = 0.0f;
  float e1 = 0.0f;
  float e0 = 0.0f;
  float u  = 0.0f;
  float up = 0.0f;
  float ui = 0.0f;
  float ud = 0.0f;

  float K  = 4.0f; // przyzwoite parametry regulacji dla Tp = 0.2
  float Ti = 4.0f; // przyzwoite parametry regulacji dla Tp = 0.2
  float Td = 0.3f; // przyzwoite parametry regulacji dla Tp = 0.2
  float Tp = 0.2f;
  /* Infinite loop */
  for(;;)
  {
	  m.type = TYPE_Y2;
	  osMessageQueuePut(qProcessHandle, &m, 1, 0); // send request for Y1
	  if((osOK == osMessageQueueGet(qControllerVHandle, &m, NULL, osWaitForever)) && (m.type == TYPE_Y2)){ // wait for response, as long as it is required
		  y = m.value;
	  }
	  e0 = cz2-y;
	  up = K*e0;
	  ui = ui+K/Ti*Tp*(e1+e0)/2.0f;
	  ud = K*Td*(e0-e1)/Tp;
	  u = up+ui+ud;
	  if(u > 1.0f)
	  	u = 1.0f;
	  if(u < -1.0f)
	  	u = -1.0f;

	  m.type = TYPE_U2;
	  m.value = u;
	  osMessageQueuePut(qProcessHandle, &m, 1, 0);

	  e1 = e0;
  }
  /* USER CODE END task_controllerV */
}

void task_controllerH(void *argument)
{
  Message m = {0};
  float y  = 0.0f;
  float e1 = 0.0f;
  float e0 = 0.0f;
  float u  = 0.0f;
  float up = 0.0f;
  float ui = 0.0f;
  float ud = 0.0f;

  float K  = 4.0f; // przyzwoite parametry regulacji dla Tp = 0.2
  float Ti = 4.0f; // przyzwoite parametry regulacji dla Tp = 0.2
  float Td = 0.3f; // przyzwoite parametry regulacji dla Tp = 0.2
  float Tp = 0.2f;
  /* Infinite loop */
  for(;;)
  {
	  m.type = TYPE_Y2;
	  osMessageQueuePut(qProcessHandle, &m, 1, 0); // send request for Y1
	  if((osOK == osMessageQueueGet(qControllerVHandle, &m, NULL, osWaitForever)) && (m.type == TYPE_Y2)){ // wait for response, as long as it is required
		  y = m.value;
	  }
	  e0 = cz2-y;
	  up = K*e0;
	  ui = ui+K/Ti*Tp*(e1+e0)/2.0f;
	  ud = K*Td*(e0-e1)/Tp;
	  u = up+ui+ud;
	  if(u > 1.0f)
	  	u = 1.0f;
	  if(u < -1.0f)
	  	u = -1.0f;

	  m.type = TYPE_U2;
	  m.value = u;
	  osMessageQueuePut(qProcessHandle, &m, 1, 0);

	  e1 = e0;
  }
  /* USER CODE END task_controllerV */
}

/* USER CODE BEGIN Header_task_staticAnimation */
/**
 * @brief Function implementing the staticAnimation thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_task_staticAnimation */
void task_staticAnimation(void *argument)
{
  /* USER CODE BEGIN task_staticAnimation */
	/* Infinite loop */
	// TODO Zaimplementować animację -- obracający się obiekt o stałej prędkości obrotowej bez wzlędu na wartość w osDelay(.)
	uint16_t center_x = 200;
	uint16_t center_y = 100;
	float half_len = 50.0;
	float angle;
	uint64_t first_tick = osKernelGetTickCount();
	uint64_t current_tick;
	uint64_t diff;
	for (;;) {
		current_tick = osKernelGetTickCount();
		diff = current_tick - first_tick;

		angle = diff % 5000;
		angle = angle*2*M_PI/5000;

		// lock
    // pierwsza wspolrzedna
		Xlist[0] = (uint16_t)(sin(angle)*half_len) + center_x;

		Ylist[0] = (uint16_t)(cos(angle)*half_len) + center_y;

		Xlist[1] = (uint16_t)(sin(angle+M_PI/2)*half_len) + center_x;
		Ylist[1] = (uint16_t)(cos(angle+M_PI/2)*half_len) + center_y;

		Xlist[2] = (uint16_t)(sin(angle+M_PI)*half_len) + center_x;
		Ylist[2] = (uint16_t)(cos(angle+M_PI)*half_len) + center_y;

		Xlist[3] = (uint16_t)(sin(angle+M_PI/2+M_PI)*half_len) + center_x;
		Ylist[3] = (uint16_t)(cos(angle+M_PI/2+M_PI)*half_len) + center_y;

		//unlock
		osDelay(1);
	}
  /* USER CODE END task_staticAnimation */
}

/* timer_process function */
void timer_process(void *argument)
{
  /* USER CODE BEGIN timer_process */
	static float y1[3] = { 0.0f };
	static float u1[2] = { 0.0f };
	static float y2[3] = { 0.0f };
	static float u2[2] = { 0.0f };
	const float a0 = -1.96535974577750070000f;
	const float a1 = +0.96560541625756646000f;
	const float b0 = +0.00012355177027502316f;
	const float b1 = +0.00012211870979073302f;

	u1[1] = u1[0];
	u1[0] = cu1;
	y1[2] = y1[1];
	y1[1] = y1[0];
	y1[0] = 0.0f;
	y1[0] = y1[0] + b0 * u1[0] + b1 * u1[1] - a0 * y1[1] - a1 * y1[2];

	u2[1] = u2[0];
	u2[0] = cu2;
	y2[2] = y2[1];
	y2[1] = y2[0];
	y2[0] = 0.0f;
	y2[0] = y2[0] + b0 * u2[0] + b1 * u2[1] - a0 * y2[1] - a1 * y2[2];

	cy1 = y1[0];
	cy2 = y2[0];
  /* USER CODE END timer_process */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM5 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM5) {
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

