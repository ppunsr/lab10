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
#include <string.h>
#include <stdio.h>
#include <math.h>

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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint16_t ADCin = 0;
uint64_t _micro = 0;
uint16_t dataOut = 0;
uint8_t DACConfig = 0b0011;
float frequency=1.0;
float Heigh_Wave_Volt =3.3;
float Low_Wave_Volt=0;
float Voltage_Output=0;
float T_On=0;
float T_Off=0;
float Duty_Cycle=30.0;
double Radian=0;
int Slope=0;
int Wave=0;
char TxDataBuffer[32] =
{ 0 };
char RxDataBuffer[32] =
{ 0 };
uint16_t STATE_Display = 0;
enum _StateDisplay
		{
			StateDisplay_Start = 0,
			StateDisplay_Menu_Print =10,
			StateDisplay_Menu_Wait_Input =20,
			StateDisplay_Wave_Form_Sawtooth=30,
			StateDisplay_Wave_Form_Sine =40,
			StateDisplay_Wave_Form_Square=50,
			StateDisplay_Increase_Frequency =60,
			StateDisplay_Decrease_Frequency=70,
			StateDisplay_Frequency_Control=80,
			StateDisplay_Wave_Form_Sawtooth_Menu=90,
			StateDisplay_Wave_Form_Sine_Menu=100,
			StateDisplay_Wave_Form_Square_Menu=110,
			StateDisplay_Volt_Control=120,
			StateDisplay_Slope_Control=130,
			StateDisplay_Duty_Control=140,
			StateDisplay_Duty_Menu=150,
			StateDisplay_Volt_Menu=160,
			StateDisplay_Slope_Menu=170,
			StateDisplay_Frequency_Menu_Print=180,

		};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void MCP4922SetOutput(uint8_t Config, uint16_t DACOutput);
uint64_t micros();
int16_t UARTRecieveIT();
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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_SPI3_Init();
  MX_TIM11_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start(&htim3);
	HAL_TIM_Base_Start_IT(&htim11);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) &ADCin, 1);
	 char Menu[]= "Sawtooth wave form  please press 1\r\n Sine wave form please press 2\r\n  Square wave form please press 3\r\n";
	 char Sawtooth_Menu[]="a: Control Frequency\r\n v:Control Voltage\r\n s: Control Slope\r\n x:Back\r\n ";
	 char Sine_Menu[]="a: Control Frequency\r\n v:Control Voltage\r\n x:Back\r\n";
	 char Square_Menu[]="a:Control Frequency\r\n v:Control Voltage\r\n D:Duty Control\r\n";
	 char Control_Frequency[]="a:+0.1 Frequency\r\n d:-0.1 Frequency\r\n x:Back\r\n";
	 char Control_Voltage[]="a:+0.1 Heigh_Wave_Volt\r\n d:-0.1 High_Wave_Volt\r\n m:+0.1 Low_Wave_Volt\r\n n: -0.1 Low_Wave_Volt\r\n";
	 char Control_Duty_Cycle[]="a: +10% Duty Cycle\r\n d: -10% Duty Cycle\r\n";
	 char Control_Slope[]="a: Slope Up\r\n d: Slope Down\r\n";
	 char Volt[30] = "";
	 char Frequency[20] = "";
	 char Slope_Characteristic[25]="";
	 char Sawtooth[]="Wave form Sawtooth\r\n";
	 char Sine[]="Wave form Sine\r\n";
	 char Square[]="Wave form Square\r\n";


	HAL_GPIO_WritePin(LOAD_GPIO_Port, LOAD_Pin, GPIO_PIN_RESET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		HAL_UART_Receive_IT(&huart2, (uint8_t*)RxDataBuffer, 32);
		int16_t inputchar = UARTRecieveIT();
		if(inputchar!=-1)
				{

					sprintf(TxDataBuffer, "ReceivedChar:[%c]\r\n", inputchar);
					HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);
				}

		switch(STATE_Display)
		{
		case StateDisplay_Start:
			STATE_Display = StateDisplay_Menu_Print;
			break;
		case StateDisplay_Menu_Print:
			HAL_UART_Transmit(&huart2, (uint8_t*)Menu, strlen(Menu), 10);
			STATE_Display = StateDisplay_Menu_Wait_Input;
			break;
		case StateDisplay_Menu_Wait_Input:
			if(inputchar=='1')
			{
				STATE_Display = StateDisplay_Wave_Form_Sawtooth;
				HAL_UART_Transmit(&huart2, (uint8_t*)Sawtooth, strlen(Sawtooth), 1000);
				break;
			}
			else if(inputchar=='2')
			{
				STATE_Display = StateDisplay_Wave_Form_Sine;
				HAL_UART_Transmit(&huart2, (uint8_t*)Sine, strlen(Sine), 1000);
				break;
			}
			else if (inputchar=='3')
			{
				STATE_Display = StateDisplay_Wave_Form_Square;
				HAL_UART_Transmit(&huart2, (uint8_t*)Square, strlen(Square), 1000);
				break;
			}
			break;
		case StateDisplay_Frequency_Control:
			switch(inputchar)
			{
			case '0':
				break;
			case 'a':
				STATE_Display = StateDisplay_Increase_Frequency;
				break;
			case 'd':
				STATE_Display = StateDisplay_Decrease_Frequency;
				break;
			case 'x':
				if(Wave==1)
				{
					STATE_Display=StateDisplay_Wave_Form_Sawtooth;
				}
				else if(Wave==2)
				{
					STATE_Display=StateDisplay_Wave_Form_Sine;
				}
				else if(Wave==3)
				{
					STATE_Display=StateDisplay_Wave_Form_Square;
				}
				break;
			}
			break;
		case StateDisplay_Decrease_Frequency:
			frequency-=0.1;
			if(frequency<0)
			{
				frequency=0;
			}
			sprintf(Frequency,"frequency:%f\r\n",frequency);
			HAL_UART_Transmit(&huart2, (uint8_t*)Frequency, strlen(Frequency), 10);
			STATE_Display =StateDisplay_Frequency_Control;
			break;
		case StateDisplay_Increase_Frequency:
			frequency+=0.1;
			if(frequency>10)
			{
				frequency=10;
			}
			sprintf(Frequency,"Frequency: %f \r\n",frequency);
			HAL_UART_Transmit(&huart2, (uint8_t*)Frequency, strlen(Frequency), 10);
			STATE_Display=StateDisplay_Frequency_Control;
			break;
		case StateDisplay_Wave_Form_Sawtooth:
			HAL_UART_Transmit(&huart2, (uint8_t*)Sawtooth_Menu, strlen(Sawtooth_Menu), 10);
			STATE_Display =StateDisplay_Wave_Form_Sawtooth_Menu;
			break;
		case StateDisplay_Wave_Form_Sine:
			HAL_UART_Transmit(&huart2, (uint8_t*)Sine_Menu, strlen(Sine_Menu), 10);
			STATE_Display =StateDisplay_Wave_Form_Sine_Menu;
			break;
		case StateDisplay_Wave_Form_Square:
			HAL_UART_Transmit(&huart2, (uint8_t*)Square_Menu, strlen(Square_Menu), 10);
			STATE_Display =StateDisplay_Wave_Form_Square_Menu;
			break;
		case StateDisplay_Volt_Control:
			HAL_UART_Transmit(&huart2, (uint8_t*)Control_Voltage, strlen(Control_Voltage), 10);
			STATE_Display= StateDisplay_Volt_Menu;
			break;
		case StateDisplay_Frequency_Menu_Print:
			HAL_UART_Transmit(&huart2, (uint8_t*)Control_Frequency, strlen(Control_Frequency), 10);
			STATE_Display= StateDisplay_Frequency_Control;
			break;
		case StateDisplay_Wave_Form_Sawtooth_Menu:
			Wave=1;
			if(inputchar=='a')
			{
				STATE_Display =StateDisplay_Frequency_Menu_Print;
				break;
			}
			else if(inputchar=='v')
			{
				STATE_Display =StateDisplay_Volt_Control;
				break;
			}
			else if(inputchar=='s')
			{
				STATE_Display =StateDisplay_Slope_Control;
				break;
			}
			else if(inputchar=='x')
			{
				STATE_Display=StateDisplay_Start;
				break;
			}
			break;
		case StateDisplay_Wave_Form_Sine_Menu:
			Wave=2;
			if(inputchar=='a')
			{
				STATE_Display =StateDisplay_Frequency_Menu_Print;
				break;
			}
			else if(inputchar=='v')
			{
				STATE_Display =StateDisplay_Volt_Control;
				break;
			}
			else if(inputchar=='x')
			{
				STATE_Display=StateDisplay_Start;
				break;
			}
			break;
		case StateDisplay_Wave_Form_Square_Menu:
			Wave=3;
			if(inputchar=='a')
			{
				STATE_Display =StateDisplay_Frequency_Menu_Print;
			}
			else if(inputchar=='v')
			{
				STATE_Display =StateDisplay_Volt_Control;
			}
			else if(inputchar=='d')
			{
				STATE_Display =StateDisplay_Duty_Control;
			}
			else if(inputchar == 'x')
			{
				STATE_Display = StateDisplay_Start;
			}
			break;
		case StateDisplay_Volt_Menu:
			if(inputchar=='a')
			{
				Heigh_Wave_Volt+=0.1;
				{
					if(Heigh_Wave_Volt>3.3)
					{
						Heigh_Wave_Volt=3.3;
					}
					sprintf(Volt,"Height_Volt: %f \r\n  Low_Volt:%f\r\n",Heigh_Wave_Volt,Low_Wave_Volt);
					HAL_UART_Transmit(&huart2, (uint8_t*)Volt, strlen(Volt), 10);
					STATE_Display=StateDisplay_Volt_Control;
				}
			}
			else if(inputchar=='d')
			{
				Heigh_Wave_Volt-=0.1;
				{
					if(Heigh_Wave_Volt<=0)
					{
						Heigh_Wave_Volt=0;
					}
						sprintf(Volt,"Height_Volt: %f \r\n  Low_Volt:%f\r\n",Heigh_Wave_Volt,Low_Wave_Volt);
					HAL_UART_Transmit(&huart2, (uint8_t*)Volt, strlen(Volt), 10);
					STATE_Display=StateDisplay_Volt_Control;
				}
			}
			else if(inputchar=='m')
			{
				Low_Wave_Volt+=0.1;
				if(Low_Wave_Volt>3.3)
				{
					Low_Wave_Volt=3.3;
				}
				sprintf(Volt,"Height_Volt: %f \r\n  Low_Volt:%f\r\n",Heigh_Wave_Volt,Low_Wave_Volt);
				HAL_UART_Transmit(&huart2, (uint8_t*)Volt, strlen(Volt), 10);
				STATE_Display=StateDisplay_Volt_Control;
			}
			else if(inputchar=='n')
			{
				Low_Wave_Volt-=0.1;
				if(Low_Wave_Volt<=0)
				{
					Low_Wave_Volt=0;
				}
				sprintf(Volt,"Height_Volt: %f \r\n  Low_Volt:%f\r\n",Heigh_Wave_Volt,Low_Wave_Volt);
				HAL_UART_Transmit(&huart2, (uint8_t*)Volt, strlen(Volt), 10);
				STATE_Display=StateDisplay_Volt_Control;
			}
			else if(inputchar=='x')
			{
				if(Wave==1)
				{
					STATE_Display=StateDisplay_Wave_Form_Sawtooth;
				}
				else if(Wave==2)
				{
					STATE_Display=StateDisplay_Wave_Form_Sine;
				}
				else if(Wave==3)
				{
					STATE_Display=StateDisplay_Wave_Form_Square;
				}
			}
			break;
		case StateDisplay_Slope_Control:
			HAL_UART_Transmit(&huart2, (uint8_t*)Control_Slope, strlen(Control_Slope), 10);
			STATE_Display = StateDisplay_Slope_Menu;
			break;
		case StateDisplay_Slope_Menu:
			if(inputchar=='a')
			{
				Slope=0;
				sprintf(Slope_Characteristic,"Slope Up\r\n");
				HAL_UART_Transmit(&huart2, (uint8_t*)Slope_Characteristic, strlen(Slope_Characteristic), 10);
				STATE_Display = StateDisplay_Slope_Control;
			}
			if(inputchar=='d')
			{
				Slope=1;
				sprintf(Slope_Characteristic,"Slope Down\r\n");
				HAL_UART_Transmit(&huart2, (uint8_t*)Slope_Characteristic, strlen(Slope_Characteristic), 10);
				STATE_Display = StateDisplay_Slope_Control;
			}
			if(inputchar=='x')
			{
				STATE_Display = StateDisplay_Wave_Form_Sawtooth;
			}
			break;
		case StateDisplay_Duty_Control:
			HAL_UART_Transmit(&huart2, (uint8_t*)Control_Duty_Cycle, strlen(Control_Duty_Cycle), 10);
			STATE_Display=StateDisplay_Duty_Menu;
			break;
		case StateDisplay_Duty_Menu:
			if(inputchar=='a')
			{
				 Duty_Cycle+= 10;
				if(Duty_Cycle > 100)
				{
					Duty_Cycle = 100;
				}
				sprintf(Control_Duty_Cycle,"Duty Cycle:%f % \r\n",Duty_Cycle);
				HAL_UART_Transmit(&huart2, (uint8_t*)Control_Duty_Cycle, strlen(Control_Duty_Cycle), 10);
				STATE_Display=StateDisplay_Duty_Control;
			}
			else if(inputchar=='d')
			{
				Duty_Cycle-=10;
				if(Duty_Cycle<=0)
				{
					Duty_Cycle=0;
				}
				sprintf(Control_Duty_Cycle,"Duty Cycle:%f % \r\n",Duty_Cycle);
				HAL_UART_Transmit(&huart2, (uint8_t*)Control_Duty_Cycle, strlen(Control_Duty_Cycle), 10);
				STATE_Display=StateDisplay_Duty_Control;
			}
			else if(inputchar=='x')
			{
				STATE_Display=StateDisplay_Wave_Form_Square;
			}
		}

		static uint64_t timestamp = 0;
		static uint64_t timestamp_final = 0;
		if(micros()-timestamp>100)
		{
			if(Wave==1)// saw tooth
			{
				if(Slope==0)//Slope Up
				{
					if(micros()-timestamp_final<=1000000/frequency)
					{
						Voltage_Output= Low_Wave_Volt+((Heigh_Wave_Volt-Low_Wave_Volt)*((micros()-timestamp_final)/(1000000/frequency)));
					}
					else if(micros()-timestamp_final>(1000000/frequency))
					{
						timestamp_final=micros();
					}
				}
				else if(Slope==1)//Slope Down
				{
					if(micros()-timestamp_final<=1000000/frequency)
					{
						Voltage_Output= Heigh_Wave_Volt-((Heigh_Wave_Volt-Low_Wave_Volt)*((micros()-timestamp_final)/(1000000/frequency)));
					}
					else if(micros()-timestamp_final>(1000000/frequency))
					{
						timestamp_final=micros();
					}
				}
			}
			if(Wave==3)//square
			{
				T_On=(Duty_Cycle/100)*(1000000/frequency);
				if(micros()-timestamp_final<=(1000000/frequency))
				{
					if(micros()-timestamp_final<=T_On)
					{
						Voltage_Output=Heigh_Wave_Volt;
					}
					else if(micros()-timestamp_final>T_On)
					{
						Voltage_Output=Low_Wave_Volt;
					}
				}
				else if(micros()-timestamp_final>(1000000/frequency))
				{
					timestamp_final=micros();
				}
			}
			if(Wave==2)//sine
			{
				if(micros()-timestamp_final<=(1000000/frequency))
				{
					Radian = ((micros() - timestamp_final)/(1000000/frequency))*2*3.14;
					Voltage_Output = ((Heigh_Wave_Volt-Low_Wave_Volt)/2+Low_Wave_Volt) + (((Heigh_Wave_Volt-Low_Wave_Volt)/2)*sin(Radian+1));
				}
				else if(micros()-timestamp_final>(1000000/frequency))
				{
					timestamp_final=micros();
				}
			}
			dataOut=(int)(Voltage_Output*4096/3.3); //0-4096
			timestamp=micros();
			if (hspi3.State == HAL_SPI_STATE_READY && HAL_GPIO_ReadPin(SPI_SS_GPIO_Port, SPI_SS_Pin)== GPIO_PIN_SET)
			{
				MCP4922SetOutput(DACConfig, dataOut);
			}
		}

//		static uint64_t timestamp = 0;
//		if (micros() - timestamp > 100)
//		{
//			timestamp = micros();
//			dataOut++;
//			dataOut %= 4096;
//			if (hspi3.State == HAL_SPI_STATE_READY
//					&& HAL_GPIO_ReadPin(SPI_SS_GPIO_Port, SPI_SS_Pin)
//							== GPIO_PIN_SET)
//			{
//				MCP4922SetOutput(DACConfig, dataOut);
//			}
//		}
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
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
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
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
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

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
  htim3.Init.Prescaler = 99;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 99;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 65535;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

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
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI_SS_GPIO_Port, SPI_SS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SHDN_GPIO_Port, SHDN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LOAD_GPIO_Port, LOAD_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin LOAD_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|LOAD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI_SS_Pin */
  GPIO_InitStruct.Pin = SPI_SS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI_SS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SHDN_Pin */
  GPIO_InitStruct.Pin = SHDN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SHDN_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
int16_t UARTRecieveIT()
{
	static uint32_t dataPos =0;
	int16_t data=-1;
	if(huart2.RxXferSize - huart2.RxXferCount!=dataPos)
	{
		data=RxDataBuffer[dataPos];
		dataPos= (dataPos+1)%huart2.RxXferSize;
	}
	return data;
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	sprintf(TxDataBuffer, "Received:[%s]\r\n", RxDataBuffer);
	HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);
}
void MCP4922SetOutput(uint8_t Config, uint16_t DACOutput)
{
	uint32_t OutputPacket = (DACOutput & 0x0fff) | ((Config & 0xf) << 12);
	HAL_GPIO_WritePin(SPI_SS_GPIO_Port, SPI_SS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit_IT(&hspi3, &OutputPacket, 1);
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if (hspi == &hspi3)
	{
		HAL_GPIO_WritePin(SPI_SS_GPIO_Port, SPI_SS_Pin, GPIO_PIN_SET);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim11)
	{
		_micro += 65535;
	}
}

inline uint64_t micros()
{
	return htim11.Instance->CNT + _micro;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
