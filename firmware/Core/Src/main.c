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
#include "string.h"
#include "stdio.h"
#include "VL53L0X.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

//Configuration structure to keep in last (31) page of FLASH
typedef struct {
	uint32_t token;			//CONF_TOKEN
	uint16_t dist_on;		//distance to turn on lights
	uint16_t dist_off;	//distance to turn off lights
} conf_t;
struct FLASH_sector {
	uint8_t  data[8];		//Config data
	uint32_t counter;		//Configuration write counter
	uint32_t checksum;	//Checksum to verify saved data
};

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define DEFAULT_DIST_ON		500
#define DEFAULT_DIST_OFF	1000

#define CONF_TOKEN				0x000A0101							//32bit token to check the configuration struct in the FLASH (000A - DiLight; rev1.1)
#define CONF_FLASH_ADDR 	((uint32_t)0x0800F800)	//FLASH address of the page to save configuration to
#define CONF_FLASH_PAGE 	31											//FLASH page number (from 0 to PgCount-1)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

/* USER CODE BEGIN PV */

union FLASH_conf {
	  conf_t config;
	  struct FLASH_sector sector;
	  uint32_t data32[4];
	  uint64_t data64[2];
} configuration;
#define FLASH_CONF_SIZE 16

uint8_t  TOF_Ready = 0;						// Flag to ignore TOF IRQ before it is initialized
uint8_t  curLightLevel = 0;				// Current light level (0-99)
uint16_t curDist = 0;							// Current measured distance in mm
uint16_t btn_ticks;								// How many ticks the button is pressed (1 tick = 100ms)
uint8_t  needConfig = 1;					// Need to configure the device
uint8_t  startConfig = 0;					// Flag to start configuration sequence
int8_t   dLevel = 1;							// Direction of fade: -1 for fade out; 1 for fade in;
uint8_t  manualOn = 0;						// Flag to indicate a manual on

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM17_Init(void);
static void MX_CRC_Init(void);
static void MX_TIM16_Init(void);
/* USER CODE BEGIN PFP */

void loadConfig(void);
void saveConfig(void);
void setLightLevel(uint8_t level);
void configure(void);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//load configuration from FLASH
void loadConfig(void) {
	uint32_t l_Address = CONF_FLASH_ADDR;
	uint32_t l_Index = 0;

	//Reading from FLASH
	while (l_Address < (CONF_FLASH_ADDR + FLASH_CONF_SIZE)) {
	  configuration.data64[l_Index] = *(__IO uint64_t *)l_Address;
	  l_Index += 1;
	  l_Address += 8;
	}


	if (HAL_CRC_Calculate(&hcrc, configuration.data32, 2) != configuration.sector.checksum
			|| configuration.config.token != CONF_TOKEN) {
		//First start or configuration is corrupted

		saveConfig();  //Save dafault config
	} else {
		//Successfully read the configuration

	}
	needConfig = 0;
}

//save configuration to FLASH
void saveConfig(void) {
  static FLASH_EraseInitTypeDef EraseInitStruct;
  uint32_t l_Address = CONF_FLASH_ADDR;
  uint32_t l_Index = 0;
  uint32_t l_Error = 0;

  //We need it to erase a page
  EraseInitStruct.TypeErase		= FLASH_TYPEERASE_PAGES;
  EraseInitStruct.Page			= CONF_FLASH_PAGE;
  EraseInitStruct.NbPages		= 1;

  if (configuration.config.token != CONF_TOKEN) {
	  //first start
	  //Nullify the struct
	  memset(configuration.data64, 0, sizeof(configuration.data64));
	  //set default values
	  configuration.config.token = CONF_TOKEN;
	  configuration.config.dist_on = DEFAULT_DIST_ON;
	  configuration.config.dist_off = DEFAULT_DIST_OFF;

	  configuration.sector.counter = 0;
  }

  if (configuration.config.dist_off > 1400) configuration.config.dist_off = 1400;
  if (configuration.config.dist_on > configuration.config.dist_off) configuration.config.dist_on = configuration.config.dist_off;
  configuration.sector.counter += 1;
  configuration.sector.checksum = HAL_CRC_Calculate(&hcrc, configuration.data32, 2);

  HAL_FLASH_Unlock();															//Unlock the FLASH
  HAL_FLASHEx_Erase(&EraseInitStruct, &l_Error);	//Erase the page

  // Programming the config

  while (l_Address < (CONF_FLASH_ADDR + FLASH_CONF_SIZE)) {
	  if  (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, l_Address, configuration.data64[l_Index]) == HAL_OK) {
		  l_Index += 1;
		  l_Address += 8;
	  }
  }
  HAL_FLASH_Lock();
  HAL_Delay(50);
}

void configure(void) {
	uint8_t i;

	startConfig = 0;
	//stopContinuous();

	//Fast blink 3 times
	needConfig = 1;
	setLightLevel(0);
	for (i = 0; i < 3; i++) {
		setLightLevel(99);
		HAL_Delay(80);
		setLightLevel(0);
		HAL_Delay(80);
	}

	//setMeasurementTimingBudget( 230 * 1000UL );		// integrate over 230 ms per measurement
	// Start measurements every 250ms
	//Configure ON distance
	//startContinuous(250);
	needConfig = 1;
	while (needConfig) {
		HAL_Delay(250);
		if (curDist > 1400) setLightLevel(0);
		// (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
		// (x - 0) * (50 - 0) / (1400 - 0) + 0
		// x * 50 / 1400
		// (99 - curDist * 99 / 2000)     //old version
		else setLightLevel((uint8_t)(50 - curDist * 50 / 1400));
	}
	configuration.config.dist_on = curDist;
	//stopContinuous();

	//Fast blink 2 times
	needConfig = 1;
	setLightLevel(0);
	for (i = 0; i < 2; i++) {
		setLightLevel(99);
		HAL_Delay(80);
		setLightLevel(0);
		HAL_Delay(80);
	}

	//Configure OFF distance
	//startContinuous(250);
	needConfig = 1;
	while (needConfig) {
		HAL_Delay(250);
		if (curDist > 1400) setLightLevel(0);
		else setLightLevel((uint8_t)(50 - curDist * 50 / 1400));
	}
	configuration.config.dist_off = curDist;
	//stopContinuous();

  saveConfig();

	//Fast blink 5 times
  needConfig = 1;
  setLightLevel(0);
	for (i = 0; i < 5; i++) {
		setLightLevel(99);
		HAL_Delay(80);
		setLightLevel(0);
		HAL_Delay(80);
	}

  //setMeasurementTimingBudget( 500 * 1000UL );		// integrate over 500 ms per measurement
	// Start measurements every second
	//startContinuous(1000);
	needConfig = 0;
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_TIM17_Init();
  MX_CRC_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */
  //TIM3 - PWM timer
  //TIM16 - ticks timer (10Hz - 100ms)
  //TIM17 - timer for light fading

  loadConfig();

  initVL53L0X(1);
	// lower the return signal rate limit (default is 0.25 MCPS)
	// setSignalRateLimit(0.1);
	// increase laser pulse periods (defaults are 14 and 10 PCLKs)
	// setVcselPulsePeriod(VcselPeriodPreRange, 18);
	// setVcselPulsePeriod(VcselPeriodFinalRange, 14);
  TOF_Ready = 1;
  setMeasurementTimingBudget( 500 * 1000UL );		// integrate over 500 ms per measurement

  // Start measurements every second
  startContinuous(1000);

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	//uint8_t t = 50;
	//int8_t di = 1;
  //char str[12];
  while (1)
  {
//  	setLightLevel(t);
//  	if (t == 99) di = -1;
//  	if (t == 0) di = 1;
//  	t += di;;
//  	HAL_Delay(10);

//  	HAL_Delay(900);
//  	//curDist = readRangeContinuousMillimeters(0);
//  	snprintf(str, 12, "Dist: %4d\n", curDist);
//  	HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 100);

  	if (startConfig) {
  		configure();
  	}
  	HAL_Delay(100);
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

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
  hi2c1.Init.Timing = 0x10707DBC;
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
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 20-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100-1;
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
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 640-1;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 10000-1;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 320-1;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 3000-1;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Sens_SHUT_GPIO_Port, Sens_SHUT_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : Btn_INT_Pin */
  GPIO_InitStruct.Pin = Btn_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Btn_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Sens_SHUT_Pin */
  GPIO_InitStruct.Pin = Sens_SHUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(Sens_SHUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Sens_INT_Pin */
  GPIO_InitStruct.Pin = Sens_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Sens_INT_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */

void setLightLevel(uint8_t level) {
	if (level > 99) level = 99;
	curLightLevel = level;

	TIM3->CCR1 = curLightLevel;
}

//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
//	if (GPIO_Pin == Btn_INT_Pin) {
//		if (HAL_GPIO_ReadPin(Btn_INT_GPIO_Port, Btn_INT_Pin)) {
//			//rising edge interrupt
//			btn_ticks = 0;
//			HAL_TIM_Base_Start_IT(&htim16);
//		} else {
//			//falling edge interrupt
//			HAL_TIM_Base_Stop_IT(&htim16);
//			if (btn_ticks < 60) { //button was not held for more than 6 seconds
//				//click();
//				if (needConfig) {
//					needConfig = 0;
//				} else {
//					if (curLightLevel == 99) curLightLevel = 0;
//					else curLightLevel = 99;
//					setLightLevel(curLightLevel);
//				}
//			}
//		}
//	}
//
//
//}

void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == Btn_INT_Pin) {
		btn_ticks = 0;
		HAL_TIM_Base_Start_IT(&htim16);
	}
}

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin) {
	uint16_t dist;

	if (TOF_Ready && GPIO_Pin == Sens_INT_Pin) {
		dist = readRangeContinuousMillimeters(0);
		if (dist < 14000)	curDist = dist;
		else curDist = 1400;
		if (!needConfig && !manualOn) {
			if (curDist <= configuration.config.dist_on && curLightLevel < 90) {
				//Turn on the lights
				dLevel = 2;
				HAL_TIM_Base_Start_IT(&htim17);
			}
			if (curDist >= configuration.config.dist_off && curLightLevel == 99) {
				//Turn off the lights
				dLevel = -1;
				HAL_TIM_Base_Start_IT(&htim17);
			}
		}
	}

	if (GPIO_Pin == Btn_INT_Pin) {
		HAL_TIM_Base_Stop_IT(&htim16);
		if (btn_ticks < 60) { //button was not held for more than 6 seconds
			//click();
			if (needConfig) {
				needConfig = 0;
			} else {
				if (curLightLevel < 90){
					//Manual turn on
					manualOn = 1;
					dLevel = 2;
					HAL_TIM_Base_Start_IT(&htim17);
				}	else {
					//Manual turn off
					manualOn = 0;
					dLevel = -1;
					HAL_TIM_Base_Start_IT(&htim17);
				}
				setLightLevel(curLightLevel);
			}
		}
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if(htim->Instance == TIM16) { //check if the interrupt comes from TIM16
		if (btn_ticks < 60) {
			btn_ticks++;
		} else {		// the button is held for more than 6 seconds
			HAL_TIM_Base_Stop_IT(&htim16);
			startConfig = 1;
		}
	}

	if(htim->Instance == TIM17) { //check if the interrupt comes from TIM17
		curLightLevel += dLevel;
		setLightLevel(curLightLevel);
		if (curLightLevel >= 99 || curLightLevel <= 0) {
			HAL_TIM_Base_Stop_IT(&htim17);
		}
	}
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
