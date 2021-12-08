/* USER CODE BEGIN Header */
/**
 * This whole project is in sync with its *.ioc file, and this includes the
 * ability to regenerate code. On the topic o BSP -- implementation of BSP in
 * this project is a custom implementation (it is not far from the original, but
 * some minor tweaks were made).
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
#include "math.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
//#include "stm32746g_discovery_lcd.h"
#include "stm32746g_discovery_ts.h"
//
#include "stm32f746g_discovery_lcd.h"
//#include "stm32f746g_discovery_ts_remote.h"
#include "string.h"
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
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc3;

DMA2D_HandleTypeDef hdma2d;

I2C_HandleTypeDef hi2c1;

LTDC_HandleTypeDef hltdc;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart;
UART_HandleTypeDef huart1;

SDRAM_HandleTypeDef hsdram1;

/* USER CODE BEGIN PV */
uint32_t ADC3_buffer[2] = {0};

#define ADC_BUFFER_LENGTH 100
uint32_t uhADCxConvertedValue[ADC_BUFFER_LENGTH] = {0};

__IO uint32_t input = 0;
__IO uint32_t output = 0;
char text[100] = "";
char text2[100] = "";
int c = 0 ;
char bufor[16] = {0};


float multipleVectors(float *Ku, float *Upast, int length) {
  uint16_t i = 0;
  float suma = 0.0f;
  for (i = 0; i < length; ++i) {
    suma = suma + Ku[i] * Upast[i];
  }
  return suma;
}

uint16_t time = 200;  // czas pomiaru
 uint16_t D = 60;       // horyzont dynamiki
 float ke = 0.0068f;    // Ke
 float Ku[] = {
     0.0011f, 0.0014f, 0.0015f, 0.0016f, 0.0016f, 0.0016f, 0.0016f, 0.0015f,
     0.0014f, 0.0013f, 0.0013f, 0.0012f, 0.0011f, 0.0010f, 0.0009f, 0.0009f,
     0.0008f, 0.0007f, 0.0007f, 0.0006f, 0.0006f, 0.0005f, 0.0004f, 0.0004f,
     0.0004f, 0.0003f, 0.0003f, 0.0003f, 0.0002f, 0.0002f, 0.0002f, 0.0002f,
     0.0002f, 0.0001f, 0.0001f, 0.0001f, 0.0001f, 0.0001f, 0.0001f, 0.0001f,
     0.0001f, 0.0000f, 0.0001f, 0.0001f, 0.0000f, 0.0001f, 0.0000f, 0.0000f,
     0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f,
     0.0000f, 0.0000f, 0.0000f};

TS_StateTypeDef TS_State;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_LTDC_Init(void);
static void MX_FMC_Init(void);
static void MX_DMA2D_Init(void);
static void MX_ADC3_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == TS_INT_PIN){
		BSP_TS_GetState(&TS_State); /*!*/
	}	else if(GPIO_Pin == TAMPER_BUTTON_PIN){
		//		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);/
		HAL_Delay(1000);
		//		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
	}
}

void DrawPointOfTouch(TS_StateTypeDef *TSS){
	static uint16_t lastx = 0;
	static uint16_t lasty = 0;
	BSP_LCD_SelectLayer(1);
	BSP_LCD_SetTextColor(LCD_COLOR_TRANSPARENT);
	BSP_LCD_DrawCircle(lastx,lasty, 3);
	BSP_LCD_DrawCircle(lastx,lasty, 2);
	if(TSS->touchDetected > 0){
		lastx = TSS->touchX[0];
		lasty = TSS->touchY[0];
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
		BSP_LCD_DrawCircle(lastx,lasty, 3);
		BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
		BSP_LCD_DrawCircle(lastx,lasty, 2);
	}
	BSP_LCD_SelectLayer(0);
}

void updateControlSignalValue(uint16_t out){
	static uint8_t buffertx[] = {0x0F, 0xFF};
	buffertx[0] = (out>>8)&0x0F;
	buffertx[1] = (out>>0)&0xFF;
	HAL_I2C_Master_Transmit(&hi2c1, 0xC2, (uint8_t*)buffertx, 2,1000);
}

void HAL_LTDC_LineEventCallback(LTDC_HandleTypeDef *hltdc){
	DrawPointOfTouch(&TS_State);
	BSP_LCD_DisplayStringAt( 0, 10, (uint8_t*)text, CENTER_MODE); // TODO For testing purposes -- can be removed
	BSP_LCD_DisplayStringAt( 0, 40, (uint8_t*)text2, CENTER_MODE); // TODO For testing purposes -- can be removed
	HAL_LTDC_ProgramLineEvent(hltdc, 272);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* AdcHandle)
{
	static int i=0;
	static uint32_t tmpval= 0;
	for(i=0,tmpval=0;i<ADC_BUFFER_LENGTH; ++i){
		tmpval += uhADCxConvertedValue[i];
	}
	input = tmpval/ADC_BUFFER_LENGTH;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM2){
		//okres 100ms
		static uint16_t licznik = 0;
//		static float y = 0.0f;
//		static float bias = 500.0f;
//		static float u_delta = 160.0f;
//		static float y_sp = 600.0f;
//		static float y_eps = 80.0f;
//		static float u = 24.0f;
//		static float Tu = 24.0f;
//		const float A = 90.0f;
//		const float Ku = 4*u_delta/(M_PI*sqrt(A*A-y_eps*y_eps));

//		y = (input-2048.0f); // przejscie z 0 - 4095 do -2048 - 2047

		/* --- TUTAJ ALGORYTM REGULACJI --- */
//		static float yzad = 0.0f; //najpierw 1500, po 1s 2500
//		static float ypast = 0.0f;
//		static float e = 0.0f;
//		static float epast = 0.0f;
//		static float up = 0.0f;
//		static float ui = 0.0f;
//		static float ud = 0.0f;
//		static float uipast = 0.0f;
////		static float K = 29.0f;
////		static float K = 18.0f;
////		static float Ti = 100000.0f;
////		static float Td = 0.0f;
//		const float K = 5;//relay 0.6*Ku;
//		const float Ti =  2;//relay0.8*Tu;
//		const float Td = 0.01;//relay 0.125*Tu;
////		static float T = 0.05f;
//		static float T = 0.1f;
//
//		ypast = y;
//		y = (input-2048.0f); // przejscie z 0 - 4095 do -2048 - 2047
//
//		if (licznik < 150)
//		{
//			yzad = 400.0f;
//			++licznik;
////			u = 100;
//		}
//		else
//		{
//			yzad = 700.0f;
//			++licznik;
////			u = 200;
//		}
//
//
////		odp. skokowa : sterowanie ze 100 na 500
////		u = 100;
//
////		regulator przekaznikowy
////		if (y > y_sp+y_eps)
////		{
////			u = bias - u_delta;
////		}
////		else if (y < y_sp - y_eps)
////		{
////			u = bias + u_delta;
////		}
//
//		if (y > yzad + y_eps)
//		{
//			u = bias - u_delta;
//		}
//		else if (y < yzad - y_eps)
//		{
//			u = bias + u_delta;
//		}
//
//
////		PID
////		epast = e;
////		uipast = ui;
////		e = yzad - y;
////
////		up = K * e;
////		ui = uipast + (K/Ti) * T * (epast+e)/2;
////		ud = K * Td * (e-epast)/T;
////
////		u = up + ui + ud;

		  static float y = 0.0f;
	      y = (input-2048.0f); // przejscie z 0 - 4095 do -2048 - 2047
		  float u = 0.0f;

		  float yzad = 0.0f;   // najpierw -500, po 1s 500
		  float ypast = 0.0f;  // poprzednia wartosc wyjscia
		  float upast = 0.0f;  // poprzednia wartosc sterowania
		  float e = 0.0f;      // uchyb

		  // zmienne obliczeniowe
		  float ue = 0.0f;
		  float uu = 0.0f;

		  float a = 3.026e-03;  // parametry rownania roznicowego
		  float b = 3.009e-03;
		  float c = 1.983f;
		  float d = 0.9831f;

		  float U[time];
		  float Y[time];
		  float Upast[D - 1];  // wektor przeszlych przyrostow sterowan

		  int i;
		  for (i = 0; i < time - 1; ++i) {
		    U[i] = 0;
		    Y[i] = 0;

		    if (i < D) Upast[i] = 0;
		  }

//		  int licznik = 2;

//		  while (licznik < time) {
		    if (licznik < 500.0f) {
		      yzad = -500.0f;
		      ++licznik;
		    }

		    else {
		      yzad = 500.0f;
		      ++licznik;
		    }

		    ypast = y;
		    upast = u;

		    y = a * U[licznik - 1] + b * U[licznik - 2] + c * Y[licznik - 1]; //- d * Y[licznik - 2];
		    Y[licznik] = y;

		    e = yzad - ypast;

		    ue = ke * e;
		    uu = multipleVectors(Ku, Upast, D - 1);

		    u = upast + ue - uu;

		    if (u > 2047.0f) u = 2047.0f;

		    if (u < -2048.0f) u = -2048.0f;

		    U[licznik] = u;

		    // Przesowanie o 1 w prawo
		    for (i = 0; i < time - 1; ++i) U[i + 1] = U[i];

		    Upast[0] = u - upast;

		    licznik = licznik + 1;
//		  }

		if(u >  2047.0f) u =  2047.0f;
		if(u < -2048.0f) u = -2048.0f;

		output = u+2048.0f; // przejscie z -2048 - 2047 do 0 - 4095
		updateControlSignalValue(output);

		while(HAL_UART_GetState(&huart) == HAL_UART_STATE_BUSY_TX);
		sprintf(text,"U=%+8.2f;Y=%+8.2f;",u,y); // 22 znaki
		if(HAL_UART_Transmit_IT(&huart, (uint8_t*)text, 22)!= HAL_OK){
			Error_Handler();
		}


//		y = (input-2048.0f); // przejscie z 0 - 4095 do -2048 - 2047
//
//		if(licznik == 0)
//		{
//			u = 2048.0f;
//			++licznik;
//		}
//		else if (licznik < 100)
//		{
//			u = 2048.0f;
//			updateControlSignalValue(u);
//			++licznik;
//		}
//		else if(licznik<=200)
//		{
//			u = 3048.0f;
//			updateControlSignalValue(u);
//
//
//			//if(u >  2047.0f) u =  2047.0f;
//			//if(u < -2048.0f) u = -2048.0f;
//			//output = u+2048.0f; // przejscie z -2048 - 2047 do 0 - 4095
//			//updateControlSignalValue(u); // aplikacja natychmiast po wyznaczeniu sterowania czy opoznic?
//
//			++licznik;
//			if (licznik == 201) licznik = 0;
//		}
//
//		while(HAL_UART_GetState(&huart) == HAL_UART_STATE_BUSY_TX)
//		{
//			sprintf(text,"U=%+8.2f;Y=%+8.2f;",u-2048.0f,y);  // 22 znaki
//		}
//				if(HAL_UART_Transmit_IT(&huart, (uint8_t*)text, 22)!= HAL_OK){
//				Error_Handler();
//			}
		/* -- KONIEC ALGORYTMU REGULACJI -- */

		if(u >  2047.0f) u =  2047.0f;
		if(u < -2048.0f) u = -2048.0f;
		output = u+2048.0f; // przejscie z -2048 - 2047 do 0 - 4095
		updateControlSignalValue(output);

		while(HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX);
		sprintf(text,"U=%.2f;Y=%.2f;\n",u,y);
		if(HAL_UART_Transmit_IT(&huart1, (uint8_t*)text, strlen(text))!= HAL_OK){
			Error_Handler();
		}
	} else if (htim->Instance == TIM3){
	} else if (htim->Instance == TIM4){
	} else if (htim->Instance == TIM5){
	}
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

	/* Enable I-Cache---------------------------------------------------------*/
	SCB_EnableICache();

	/* Enable D-Cache---------------------------------------------------------*/
	SCB_EnableDCache();

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
	MX_ADC3_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_TIM4_Init();
	MX_TIM5_Init();
	MX_I2C1_Init();
	MX_USART1_UART_Init();
	/* USER CODE BEGIN 2 */
	BSP_LCD_SetFont(&Font20); // choose size of the font: Font8, Font12, Font16, Font20, Font24
	BSP_LCD_SetBackColor(LCD_COLOR_WHITE); // each character has background!!!
	BSP_TS_Init(0,0); // initialization of TouchScreen -- arguments are irrelevant
	BSP_TS_ITConfig(); // to cancel exti interrupts from the touch screen comment this line


	//BSP_PB_Init(BUTTON_TAMPER, BUTTON_MODE_EXTI);

	BSP_LCD_SelectLayer(0);
	BSP_LCD_Clear(LCD_COLOR_WHITE);
	BSP_LCD_SelectLayer(1);
	BSP_LCD_Clear(LCD_COLOR_TRANSPARENT);
	HAL_LTDC_ProgramLineEvent(&hltdc, 272);

	if(HAL_ADC_Start_DMA(&hadc3, (uint32_t*)uhADCxConvertedValue, ADC_BUFFER_LENGTH) != HAL_OK)
		Error_Handler();
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	HAL_Delay(100);
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_Base_Start_IT(&htim4);
	HAL_TIM_Base_Start_IT(&htim5);
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0); // preemption priority and subpriority of SysTick
	HAL_NVIC_SetPriority(TIM2_IRQn   , 0, 0); // preemption priority and subpriority of TIM2
	HAL_NVIC_SetPriority(TIM3_IRQn   , 0, 0); // preemption priority and subpriority of TIM3
	HAL_NVIC_SetPriority(TIM4_IRQn   , 0, 0); // preemption priority and subpriority of TIM4
	HAL_NVIC_SetPriority(TIM5_IRQn   , 0, 0); // preemption priority and subpriority of TIM5

	HAL_NVIC_EnableIRQ(TIM3_IRQn); // enabling handling functions of TIM3
	HAL_NVIC_EnableIRQ(TIM4_IRQn); // enabling handling functions of TIM4
	HAL_NVIC_EnableIRQ(TIM5_IRQn); // enabling handling functions of TIM5
	HAL_NVIC_EnableIRQ(TS_INT_EXTI_IRQn);


	HAL_Delay(100); // wait for everything to set up before the controller loop starts
	HAL_NVIC_EnableIRQ(TIM2_IRQn); // enabling handling functions of TIM2

	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		//		BSP_TS_GetState(&TS_State); /*!*/
		if(TS_State.touchDetected > 0){
			sprintf(text2,"X=%d;Y=%d;\n",TS_State.touchX[0],TS_State.touchY[0]); // 22 znaki
		} else {
			sprintf(text2,"NO TOUC   H\n"); // 22 znaki
		}
		HAL_Delay(100);
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
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC|RCC_PERIPHCLK_USART1
			|RCC_PERIPHCLK_I2C1;
	PeriphClkInitStruct.PLLSAI.PLLSAIN = 192;
	PeriphClkInitStruct.PLLSAI.PLLSAIR = 5;
	PeriphClkInitStruct.PLLSAI.PLLSAIQ = 2;
	PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV2;
	PeriphClkInitStruct.PLLSAIDivQ = 1;
	PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_4;
	PeriphClkInitStruct.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
	PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
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
	hadc3.Init.NbrOfConversion = 1;
	hadc3.Init.DMAContinuousRequests = ENABLE;
	hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc3) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_8;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
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
	hi2c1.Init.Timing = 0x10606AA0;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_DISABLE) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

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
	hltdc.Init.TotalWidth = 565;
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
	htim2.Init.Prescaler = 10800-1;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 1000;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
	htim3.Init.Prescaler = 10800-1;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 200;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void)
{

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 10800-1;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 10000;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */

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
	htim5.Init.Prescaler = 10800-1;
	htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim5.Init.Period = 10000;
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
	HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
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
	__IO uint32_t tmpmrd =0;
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
	tmpmrd = (uint32_t)SDRAM_MODEREG_BURST_LENGTH_1          |
			SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL   |
			SDRAM_MODEREG_CAS_LATENCY_2           |
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
	hsdram1.Instance->SDRTR |= ((uint32_t)((1292)<< 1));

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
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOG_CLK_ENABLE();
	__HAL_RCC_GPIOJ_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOK_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOI_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LCD_BL_CTRL_GPIO_Port, LCD_BL_CTRL_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LCD_DISP_GPIO_Port, LCD_DISP_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);

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

	/*Configure GPIO pins : PC7 PC6 */
	GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_6;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : TS_TOUCHED_Pin */
	GPIO_InitStruct.Pin = TS_TOUCHED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(TS_TOUCHED_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : PA0 */
	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 15, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
