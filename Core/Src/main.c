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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "NRF24L01.h"
#include "i2c.h"
#include "QMC5883L.h"
#include "VL53L0X.h"
#include "ESC.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define LONG_RANGE

#define HIGH_SPEED
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/*--------------- Variaveis para sensores e comunicação USB -----------------*/
struct VL53L0X myTOFsensor = {.io_2v8 = true, .address = 0b0101001, .io_timeout = 500, .did_timeout = false};
struct QMC5883L myMAGsensor = {.io_2v8 = true, .address = 0x0D, .io_timeout = 500, .did_timeout = false};
uint16_t dist = 0;
int16_t MagX,MagY,MagZ = 0;
int16_t angulo;
uint16_t duty_T,duty_T1, duty_T2,duty_T3,duty_T4;
uint16_t cont=0;
uint16_t valor_ad1;
//char buf[8];

/* -------------- Variáveis para NRF24l01+ --------------------------------*/

uint8_t RxAddress[] = {0x00,0xDD,0xCC,0xBB,0xAA};
uint8_t RxData[10] = {0,0,0,0,0,0,0,0,0,0};

uint8_t TxAddress[] = {0xEE,0xDD,0xCC,0xBB,0xAA};
uint8_t TxData[10] = {0,0,0,0,0,0,0,0,0,0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

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
  SysTick_Config(SystemCoreClock / 1000);
  //-------------- Inicializa o móduloHMC5883L ---------------------------//
    	// init the I2C1 peripheral and the SDA/SCL GPIO pins
    		init_i2c1();
    	// Initialize HMC5883L
    		QMC5883L_Initialize(&myMAGsensor,MODE_CONTROL_CONTINUOUS,OUTPUT_DATA_RATE_100HZ,FULL_SCALE_2G,OVER_SAMPLE_RATIO_128);

    			// GPIO pin PB5 connected to XSHUT pin of sensor
    			GPIOB->CRL &=~GPIO_CRL_CNF5;
    			GPIOB->CRL |= GPIO_CRL_MODE5;
    			GPIOB->ODR &=~GPIO_ODR_ODR5; // shut off VL53L0X
    			HAL_Delay(1);
    			GPIOB->ODR |= GPIO_ODR_ODR5; // power up VL53L0X again
    			HAL_Delay(2);
//---------------- Inicializa o módulo VL53L0x ---------------------------//
    			VL53L0X_init(&myTOFsensor);

    			#ifdef LONG_RANGE
    			// lower the return signal rate limit (default is 0.25 MCPS)
    			VL53L0X_setSignalRateLimit(&myTOFsensor, 0.1);
    			// increase laser pulse periods (defaults are 14 and 10 PCLKs)
    			VL53L0X_setVcselPulsePeriod(&myTOFsensor, VcselPeriodPreRange, 18);
    			VL53L0X_setVcselPulsePeriod(&myTOFsensor, VcselPeriodFinalRange, 14);
    		#endif
    		#ifdef HIGH_SPEED
    			// reduce timing budget to 20 ms (default is about 33 ms)
    			VL53L0X_setMeasurementTimingBudget(&myTOFsensor, 20000);
    			//USART1_transmitString("step1\n");
    		#else //HIGH_ACCURACY
    			// increase timing budget to 200 ms
    			VL53L0X_setMeasurementTimingBudget(&myTOFsensor, 200000);
    		#endif

    			VL53L0X_startContinuous(&myTOFsensor, 0);

    			 //------------------------------------------------------



  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  // Inicializa SPI

         NRF24_Init();

//    HAL_TIM_Base_Start_IT(&htim3);
    HAL_TIM_Base_Start_IT(&htim2);

    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); 			// inicializa o PWM
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);


//------------------------------------------------------------//
    duty_T = 0.05*htim2.Init.Period*1.0345; 			// variável de controle do duty do PWM do timer 1. Varia de (5%)2000 a (10%) 4000
    		     TIM2->CCR1 = duty_T;
    		     TIM2->CCR2 = duty_T;								 	//atribui o valor de duty ao PWM
    		     TIM2->CCR3 = duty_T;
    		     TIM2->CCR4 = duty_T;
    		     HAL_Delay(2000);

    /*duty_T = 0.073*htim2.Init.Period*1.0345; 			// variável de controle do duty do PWM do timer 1. Varia de (5%)2000 a (10%) 4000
     		     TIM2->CCR1 = duty_T;
       		     TIM2->CCR2 = duty_T;								 	//atribui o valor de duty ao PWM
      		     TIM2->CCR3 = duty_T;
       		     TIM2->CCR4 = duty_T;
       		     HAL_Delay(2000);*/
NRF24_TxMode(TxAddress,10);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
while (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK);
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


	  if (isDataAvailable(2) == 1)
				 {
					  NRF24_Receive(RxData);
					  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

				  }

	  NRF24_TxMode(TxAddress, 10);
	   /*envia valores pelo wifi*/
	  // HAL_Delay(10);

	 	 if(NRF24_Transmit(TxData) == 1){
	 		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
	 		NRF24_RxMode(RxAddress, 10);
	 	 }

	 	  	  dist = VL53L0X_readRangeContinuousMillimeters(&myTOFsensor);
	 	  	  angulo = QMC5883L_Azimuth(&myMAGsensor);
	 		  duty_T1 = RxData[0]*htim2.Init.Period*1.0345*0.0005 + 5*htim2.Init.Period*1.0345*0.01;  // variável de controle do duty do PWM do timer 1. Varia de (5%)2000 a (10%) 4000
	 		  duty_T2 = RxData[1]*htim2.Init.Period*1.0345*0.0005 + 5*htim2.Init.Period*1.0345*0.01;  // variável de controle do duty do PWM do timer 1. Varia de (5%)2000 a (10%) 4000
	 		  duty_T3 = RxData[2]*htim2.Init.Period*1.0345*0.0005 + 5*htim2.Init.Period*1.0345*0.01;  // variável de controle do duty do PWM do timer 1. Varia de (5%)2000 a (10%) 4000
	 		  duty_T4 = RxData[3]*htim2.Init.Period*1.0345*0.0005 + 5*htim2.Init.Period*1.0345*0.01;  // variável de controle do duty do PWM do timer 1. Varia de (5%)2000 a (10%) 4000

	 		  TIM2->CCR1 = duty_T1;
	 		  TIM2->CCR2 = duty_T2;								 	//atribui o valor de duty ao PWM
	 		  TIM2->CCR3 = duty_T3;
	 		  TIM2->CCR4 = duty_T4;

	 		    HAL_ADC_Start(&hadc1);						// Inicia o conversor A/D
				while (HAL_ADC_PollForConversion(&hadc1, 1) != HAL_OK);						// wait for completing the conversion
				HAL_ADC_Stop(&hadc1);
				valor_ad1 = HAL_ADC_GetValue(&hadc1);// Atribui valor à variável utilizada

	 		   TxData [0] = duty_T1;
	 		   TxData [1] = duty_T1 >> 8;
	 		   TxData [2] = angulo;
	 		   TxData [3] = angulo >> 8;
	 		   TxData [4] = valor_ad1;
	 		   TxData [5] = valor_ad1 >> 8;

	  //HAL_Delay(2);


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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
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
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
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
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
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
  htim2.Init.Prescaler = 36-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 40000-1;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
  * @brief  Period elapsed callback in non-blocking mode
  * @param  htim TIM handle
  * @retval None
  */
 void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim3)
{



  /*envia valor de duty para matlab*/
   //int tam = sizeof(UserTxBuffer);  //define o tamanho da string
   //UserTxBuffer [tam1] = '\n';            //adciona quebra de linha
   //UserTxBuffer2[tam1+1] = '\0';        //adiciona id ao final da string

   //CDC_Transmit_FS(UserTxBuffer, tam);  //envia dados para PC

     //UserTxBuffer[tam] = '\n';            //adciona quebra de linha
     //UserTxBuffer[tam+1] = '\0';        //adiciona id ao final da string
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
