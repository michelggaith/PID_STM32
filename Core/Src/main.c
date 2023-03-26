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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NUM_ADC 3
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void ADC_Select_CH4(void){
	/** Configure Regular Channel
	  */
	  ADC_ChannelConfTypeDef sConfig = {0};
	  sConfig.Channel = ADC_CHANNEL_4;
	  sConfig.Rank = ADC_REGULAR_RANK_1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }

}
void ADC_Select_CH5(void){
	/** Configure Regular Channel
		  */
		  ADC_ChannelConfTypeDef sConfig = {0};
		  sConfig.Channel = ADC_CHANNEL_5;
		  sConfig.Rank = ADC_REGULAR_RANK_1;
		  sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
		  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		  {
		    Error_Handler();
		  }

}

void ADC_Select_CH6(void){
	/** Configure Regular Channel
			  */
			  ADC_ChannelConfTypeDef sConfig = {0};
			  sConfig.Channel = ADC_CHANNEL_6;
			  sConfig.Rank = ADC_REGULAR_RANK_1;
			  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
			  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
			  {
			    Error_Handler();
			  }
}

	 uint16_t ADCValue[3];
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	 int32_t dutyCycle = 0;
	 int PWM;
	 int i;
	 uint8_t charValue[50];
	 uint8_t inDig[8];
	 uint8_t dataRec[8]={};
	 int dato[9]={};
	 float temperatura;
	 int rec = 0;
  // ----------------------------Variables usadas para el filtrado-----------------------------------
	 float y[2] = {0.0,0.0};
	 float x[2] = {0.0,0.0};
	 float frec_corte = 2*3.14*0.1;
	 float A1, A2, A3;
	 float suavizar = 0;
	 int ADC_filtrado;
 // ----------------------------Variables usadas para la discretizacion del PID-----------------------------------
	 float referencia;
	 float retro; 					// Retroalimentacion
	 float e[3] = {0.0,0.0,0.0}; 	//Almacenamos el error actual, el error anterior y el anterior 2 veces
	 float u[3] = {0.0,0.0,0.0}; 	// Almacena la salida actual, la anterior y la anterior 2 veces
	 float kp = 44;
	 float ki = 1.53;
	 float kd = 100;
	 float T = 1;
	 float K1, K2, K3;			 	 // Variables auxiliares
	 float a1, a2, a3;
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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
	  suavizar = 0;
//------------LECTURA PUERTO SERIE: Los datos estan guardados como ASCII, el programa no avanza hasta que se reciben datos----------------------//
	  HAL_UART_Receive_IT(&huart2, (uint16_t*)dataRec, sizeof(dataRec));

	  dato[0] = (int)(dataRec[0]);
	  dato[1] = (int)(dataRec[1]);

	  if(rec<=-500)
	  {
		  rec=0;
	  }
	  else
	  {
	  rec = (dato[0]-48)*10 + (dato[1]-48);
	  }

 //-------------------------------------------DEFINICION DEL ESTADO DE LAS SALIDAS--------------------------------------------------------------//

	  if(dato[0]==49)
	  {
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
	  }
	  else
	  {
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
	  }
	  if(dato[1]==49)
	  {
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
	  }
	  else
	  {
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
	  }
	  if(dato[2]==49)
	  {
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
	  }else
	  {
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
	  }
	  if(dato[3]==49)
	  {
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
	  }
	  else
	  {
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
	  }
	  if(dato[4]==49)
	  {
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
	  }
	  else
	  {
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
	  }
	  if(dato[5]==49)
	  {
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
	  }else
	  {
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
	  }
	  if(dato[6]==49)
	  {
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
	  }
	  else
	  {
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
	  }
	  if(dato[7]==49)
	  {
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	  }
	  else
	  {
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	  }
	  //-------------------------------------------ETAPA LECTURA DE ENTRADAS DIGITALES--------------------------------------------------------------//

	  /*if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0))
	   {inDig[0] = 1;}
	   else{inDig[0] = 0;}*/

	  	if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7))
	  	{inDig[2] = 1;}
	  	else{inDig[2] = 0;}

	  	if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0))
	  	{inDig[3] = 1;}
	  	else{inDig[3] = 0;}

	  	if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1))
	  	{inDig[4] = 1;}
	  	else{inDig[4] = 0;}

	  	if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10))
	  	{inDig[5] = 1;}
	  	else{inDig[5] = 0;}

	  	if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11))
	  	{inDig[6] = 1;}
	  	else{inDig[6] = 0;}

	  	if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9))
	  	{inDig[7] = 1;}
	  	else{inDig[7] = 0;}

 //-------------------------------------------ETAPA MANEJO DE ADC--------------------------------------------------------------//
	  for(i = 0; i <10; i++)
	  {
	  	//CANAL 4
	  	ADC_Select_CH4();
	  	HAL_ADC_Start(&hadc1);
	  	HAL_ADC_PollForConversion(&hadc1, 1000);
	  	ADCValue[0]= HAL_ADC_GetValue(&hadc1);
	  	HAL_ADC_Stop(&hadc1);
	  	suavizar = ADCValue[0] + suavizar;
	  }

	  	//CANAL 5
	  	ADC_Select_CH5();
	  	HAL_ADC_Start(&hadc1);
	  	HAL_ADC_PollForConversion(&hadc1, 1000);
	  	ADCValue[1]= HAL_ADC_GetValue(&hadc1);
	  	HAL_ADC_Stop(&hadc1);
	  	//CANAL 6
	 	ADC_Select_CH6();
	  	HAL_ADC_Start(&hadc1);
	  	HAL_ADC_PollForConversion(&hadc1, 1000);
	  	ADCValue[2]= HAL_ADC_GetValue(&hadc1);
	  	HAL_ADC_Stop(&hadc1);

	  	x[0] = suavizar/10;
  //-------------------------------------------FILTRADO ADC--------------------------------------------------------------------------//
	  	A1 = (frec_corte*T)/(2+frec_corte*T);
	  	A2 = A1;
	  	A3 = (frec_corte*T-2)/(2+frec_corte*T);
	  	y[0] = A1*x[0]+A2*x[1]-y[1]*A3;
	  	ADC_filtrado = (int)y[0];
  //-------------------------------------------FLAG CONTROL TEMPERATURA--------------------------------------------------------------//
	  	temperatura = (3.3*y[0]/4095)/0.01;

  //-------------------------------------------CALCULO DEL PID--------------------------------------------------------------------
	  	referencia = rec;	 				 // Referencia deseada de temperatura
	  	retro = temperatura; 				 // Valor leido por parte del sensor LM35
  	  	if(temperatura<20)
  	  	{
  	  		e[0] = 0;
  	  	}
  	  	else
  	  	{
  	  		e[0] = referencia - retro;
  	  	}
  	//-------------------------------------------CALCULO DEL PID--------------------------------------------------------------------
	  	K1 = kp+ki*(T/2)+kd*(2/T);   		 //Coeficiente que multiplica el error actual.
	  	K2 = ki*T-kd*(4/T);			 		 //Coeficiente que multiplica el error previo.
	  	K3 = ki*(T/2)+kd*(2/T)-kp;			 //Coeficiente que multiplica el error dos veces previo.
	  	u[0] = K1*e[0]+K2*e[1]+K3*e[2]+u[2];
	  	a1 = K1*e[0];
	  	a2 = K2* e[1];
	  	a3 = K3*e[2];
	  	//Ecuacion PID
  //-------------------------------------------Ajuste del PID (se establecen limitadores)-------------------------------------------
	  	if(u[0]>100)
	  	{
	  		u[0] = 100;
	  	}
	  	if(u[0]<0.0)
	  	{
	  	    u[0] = 0;
	  	}
  //---------------------------------------------------CONTROL PWM------------------------------------------------------//
	  	dutyCycle =u[0]*65535/100;
	  	TIM2->CCR2 = dutyCycle;
	  	PWM = dutyCycle*100/65535;
  //---------------------------------------------------CORRIMIENTOS------------------------------------------------------//
  //Corrimiento de errores
	  	e[2] = e[1];
	  	e[1] = e[0];
  //Corrimiento de salidas
	  	u[2] = u[1];
	  	u[1] = u[0];
  //Corrimiento elementos del filtro
	  	y[1] = y[0];
	  	x[1] = x[0];
  //-------------------------------------------TRANSMISION VIA UART--------------------------------------------------------------//
  //	 sprintf(charValue, "%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,2\n",ADCValue[0],ADCValue[1],ADCValue[2], inDig[0],inDig[1],inDig[2],inDig[3],inDig[4],inDig[5],inDig[6],inDig[7]);
	  sprintf(charValue, "0,%lu,%lu,2\n",ADC_filtrado,PWM);
	  //sprintf(charValue, ",%lu\n",auxiliar);
	     HAL_UART_Transmit(&huart2, (uint8_t*)charValue, strlen(charValue),HAL_MAX_DELAY);
 	     HAL_Delay(1000);

  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
//  sConfig.Channel = ADC_CHANNEL_4;
//  sConfig.Rank = ADC_REGULAR_RANK_1;
//  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
//  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /** Configure Regular Channel
//  */
//  sConfig.Rank = ADC_REGULAR_RANK_2;
//  sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
//  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /** Configure Regular Channel
//  */
//  sConfig.Rank = ADC_REGULAR_RANK_3;
//  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
//  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Salida8_Pin|Salida7_Pin|Salida6_Pin|Salida5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Salida4_Pin|Salida3_Pin|Salida2_Pin|Salida1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Entrada1_Pin Entrada3_Pin */
  GPIO_InitStruct.Pin = Entrada1_Pin|Entrada3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Entrada4_Pin Entrada5_Pin Entrada6_Pin Entrada7_Pin
                           Entrada8_Pin */
  GPIO_InitStruct.Pin = Entrada4_Pin|Entrada5_Pin|Entrada6_Pin|Entrada7_Pin
                          |Entrada8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Salida8_Pin Salida7_Pin Salida6_Pin Salida5_Pin */
  GPIO_InitStruct.Pin = Salida8_Pin|Salida7_Pin|Salida6_Pin|Salida5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Salida4_Pin Salida3_Pin Salida2_Pin Salida1_Pin */
  GPIO_InitStruct.Pin = Salida4_Pin|Salida3_Pin|Salida2_Pin|Salida1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
