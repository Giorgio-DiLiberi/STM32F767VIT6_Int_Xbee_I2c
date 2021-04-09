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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include "BMX160.h"
#include <string.h>

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

I2C_HandleTypeDef hi2c4;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart7;
DMA_HandleTypeDef hdma_uart7_rx;

/* USER CODE BEGIN PV */

BMX_IMU_typedef BMX; //BMX strut creation

uint8_t buff[32]; //buffer for data transmission
char string[] = "Hello World!\r\n";
uint8_t main_counter = 0; //counter to trigger events only after some main cycles
uint8_t tlm_counter = 0; //counter to send telemetry
HAL_StatusTypeDef ret;

float Tlm_Tx_buff[6]; //buffer to store the floating point values to send via telemetry

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C4_Init(void);
static void MX_TIM2_Init(void);
static void MX_UART4_Init(void);
static void MX_UART7_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

void BMX_Init(BMX_IMU_typedef *BMX_str);//init for BMX

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
  MX_I2C4_Init();
  MX_TIM2_Init();
  MX_UART4_Init();
  MX_UART7_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  //BMX initialization

  HAL_Delay(1000);

  LL_TIM_EnableCounter(TIM1);

  BMX_Init(&BMX);

  //writing hello world on xbee
  sprintf((char*)buff, string);

  HAL_UART_Transmit(&huart4, buff, strlen((char*)buff), HAL_MAX_DELAY);

  //BMX calibration
  //BMX_calibration(&BMX);

  //Starting timer 2 in interrupt mode
  HAL_TIM_Base_Start_IT(&htim2); //this function start the timer in interrupt mode and calls the irq handler 
  //every time that an interrupt event occurs, but the only event which will call a function is period elapsed

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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 192;
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
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_UART4|RCC_PERIPHCLK_UART7
                              |RCC_PERIPHCLK_I2C4;
  PeriphClkInitStruct.Uart4ClockSelection = RCC_UART4CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Uart7ClockSelection = RCC_UART7CLKSOURCE_PCLK1;
  PeriphClkInitStruct.I2c4ClockSelection = RCC_I2C4CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C4_Init(void)
{

  /* USER CODE BEGIN I2C4_Init 0 */

  /* USER CODE END I2C4_Init 0 */

  /* USER CODE BEGIN I2C4_Init 1 */

  /* USER CODE END I2C4_Init 1 */
  hi2c4.Instance = I2C4;
  hi2c4.Init.Timing = 0x2010091A;
  hi2c4.Init.OwnAddress1 = 0;
  hi2c4.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c4.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c4.Init.OwnAddress2 = 0;
  hi2c4.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c4.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c4.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c4) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c4, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c4, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C4_Init 2 */

  /* USER CODE END I2C4_Init 2 */

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

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  TIM_InitStruct.Prescaler = 191;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 65000;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_InitStruct.RepetitionCounter = 0;
  LL_TIM_Init(TIM1, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM1);
  LL_TIM_SetClockSource(TIM1, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_RESET);
  LL_TIM_SetTriggerOutput2(TIM1, LL_TIM_TRGO2_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM1);
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  htim2.Init.Prescaler = 95;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9999;
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
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART7_Init(void)
{

  /* USER CODE BEGIN UART7_Init 0 */

  /* USER CODE END UART7_Init 0 */

  /* USER CODE BEGIN UART7_Init 1 */

  /* USER CODE END UART7_Init 1 */
  huart7.Instance = UART7;
  huart7.Init.BaudRate = 100000;
  huart7.Init.WordLength = UART_WORDLENGTH_9B;
  huart7.Init.StopBits = UART_STOPBITS_1;
  huart7.Init.Parity = UART_PARITY_NONE;
  huart7.Init.Mode = UART_MODE_TX_RX;
  huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart7.Init.OverSampling = UART_OVERSAMPLING_16;
  huart7.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart7.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART7_Init 2 */

  /* USER CODE END UART7_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin : PE2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void BMX_Init(BMX_IMU_typedef *BMX_str){

  //initialization for BMX struct values
  BMX_str->i2c_handler_ptr=&hi2c4;
  BMX_str->uart_hndler_ptr=&huart4;
  BMX_str->acc_FS_conv=0.000122f; //[mg/LSB] to multiply the int16 value and obtain a float
  BMX_str->gyro_FS_conv=0.0153f; //[deg/s/LSB] to multiply the int16 value and obtain a float

  //Initialization for values of BMX---> device register settings
  //gyro init and config
	buff[0] = GYRO_CONF_REG;
	buff[1] = 0b00101001;//set the Gyro conf register (see the red notebook page 10)
  //set for 200Hz ODR and digital filter normal mode
  if(HAL_I2C_Master_Transmit(BMX_str->i2c_handler_ptr, BMX160Address, buff, 2, HAL_MAX_DELAY)==HAL_OK){
    sprintf((char*)buff, "Gyro conf OK!\n\r");
    HAL_UART_Transmit(&huart4, buff, strlen((char*)buff), HAL_MAX_DELAY);
  } else{
    sprintf((char*)buff, "ERROR BMX\n\r");
    HAL_UART_Transmit(&huart4, buff, strlen((char*)buff), HAL_MAX_DELAY);
  }

  //wait 
  LL_TIM_SetCounter(TIM1, 0);
  while(LL_TIM_GetCounter(TIM1)<10000){}

	//acc init and config
	buff[0] = ACC_CONF_REG;
	buff[1] = 0b00101001;//set the Acc conf register to 200Hz for output data rate and digital filter normal mode 
  if(HAL_I2C_Master_Transmit(BMX_str->i2c_handler_ptr, BMX160Address, buff, 2, HAL_MAX_DELAY)==HAL_OK){
    sprintf((char*)buff, "Gyro conf OK!\n\r");
    HAL_UART_Transmit(&huart4, buff, strlen((char*)buff), HAL_MAX_DELAY);
  } else{
    sprintf((char*)buff, "ERROR BMX\n\r");
    HAL_UART_Transmit(&huart4, buff, strlen((char*)buff), HAL_MAX_DELAY);
  }

  //wait 
  LL_TIM_SetCounter(TIM1, 0);
  while(LL_TIM_GetCounter(TIM1)<10000){}

	//acc range config
	buff[0] = ACC_RANGE_REG;
	buff[1] = 0b00000101;//set the Acc range register (see the red notebook appropriate page)
    //this set for a +-4g full scale -> 0.000122 g/LSB Coeff to multiply to in16 data to obtain float
	if(HAL_I2C_Master_Transmit(BMX_str->i2c_handler_ptr, BMX160Address, buff, 2, HAL_MAX_DELAY)==HAL_OK){
    sprintf((char*)buff, "Gyro conf OK!\n\r");
    HAL_UART_Transmit(&huart4, buff, strlen((char*)buff), HAL_MAX_DELAY);
  } else{
    sprintf((char*)buff, "ERROR BMX\n\r");
    HAL_UART_Transmit(&huart4, buff, strlen((char*)buff), HAL_MAX_DELAY);
  }

  //wait 
  LL_TIM_SetCounter(TIM1, 0);
  while(LL_TIM_GetCounter(TIM1)<10000){}

    //gyro range config
	buff[0]=GYRO_RANGE_REG;
	buff[1]=0b00000010;//set the gyro range register (see the red notebook appropriate page)
    //this set for a +-500 deg/s full scale -> 0.0153 deg/s/LSB Coeff to multiply to in16 data to obtain float
	if(HAL_I2C_Master_Transmit(BMX_str->i2c_handler_ptr, BMX160Address, buff, 2, HAL_MAX_DELAY)==HAL_OK){
    sprintf((char*)buff, "Gyro conf OK!\n\r");
    HAL_UART_Transmit(&huart4, buff, strlen((char*)buff), HAL_MAX_DELAY);
  } else{
    sprintf((char*)buff, "ERROR BMX\n\r");
    HAL_UART_Transmit(&huart4, buff, strlen((char*)buff), HAL_MAX_DELAY);
  }

  //wait 
  LL_TIM_SetCounter(TIM1, 0);
  while(LL_TIM_GetCounter(TIM1)<10000){}

  //power mode to normal is set
  //change the power mode to normal for the gyro
	buff[0] = BMX_PM_REG;
  buff[1] = 0x15;//command for normal mode for gyro;
	if(HAL_I2C_Master_Transmit(BMX_str->i2c_handler_ptr, BMX160Address, buff, 2, HAL_MAX_DELAY)==HAL_OK){
    sprintf((char*)buff, "Gyro conf OK!\n\r");
    HAL_UART_Transmit(&huart4, buff, strlen((char*)buff), HAL_MAX_DELAY);
  } else{
    sprintf((char*)buff, "ERROR BMX\n\r");
    HAL_UART_Transmit(&huart4, buff, strlen((char*)buff), HAL_MAX_DELAY);
  }

  //wait 
  HAL_Delay(500);

	//change the power mode to normal for the Acc see appropriate page of the notebook
  //the power mode is changed writing two times to the PMU reg
	buff[0] = BMX_PM_REG;
	buff[1] = 0x11;//command for normal mode for accelerometer;
	if(HAL_I2C_Master_Transmit(BMX_str->i2c_handler_ptr, BMX160Address, buff, 2, HAL_MAX_DELAY)==HAL_OK){
    sprintf((char*)buff, "Acc pow OK!\n\r");
    HAL_UART_Transmit(&huart4, buff, strlen((char*)buff), HAL_MAX_DELAY);
  } else{
    sprintf((char*)buff, "ERROR ACP\n\r");
    HAL_UART_Transmit(&huart4, buff, strlen((char*)buff), HAL_MAX_DELAY);
  }

  //wait 
  LL_TIM_SetCounter(TIM1, 0);
  while(LL_TIM_GetCounter(TIM1)<600){}

}


//definition of IRQhandler functions callback
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim){

  if (htim==&htim2){

    if(main_counter>=100){
      main_counter = 0;
      HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_2);//led toggle
      
      // sprintf((char*)buff, "accX= %f\n\r", BMX.Acceleration[0]);
      // HAL_UART_Transmit(&huart4, buff, strlen((char*)buff), HAL_MAX_DELAY);

    }

    BMX_read(&BMX); //Imu data read and save to the struct: data are raw unfiltered and maybe biased 

    if (1){

      //fill the telemetry array with red values
      for (int i_tlm=0; i_tlm<6; i_tlm++) {
        if (i_tlm<3) {
          *(Tlm_Tx_buff+i_tlm) = BMX.Omega[i_tlm];
        }
        else if (i_tlm>=3 && i_tlm<6) {
          *(Tlm_Tx_buff+i_tlm) = BMX.Acceleration[i_tlm - 3];
        }
      }

      /* sprintf((char*)buff, "OmX= %f\n\r", Tlm_Tx_buff[0]);
      HAL_UART_Transmit(&huart4, buff, strlen((char*)buff), HAL_MAX_DELAY);

      sprintf((char*)buff, "OmY= %f\n\r", Tlm_Tx_buff[1]);
      HAL_UART_Transmit(&huart4, buff, strlen((char*)buff), HAL_MAX_DELAY);

      sprintf((char*)buff, "OmZ= %f\n\r", Tlm_Tx_buff[2]);
      HAL_UART_Transmit(&huart4, buff, strlen((char*)buff), HAL_MAX_DELAY);

      sprintf((char*)buff, "aX= %f\n\r", Tlm_Tx_buff[3]);
      HAL_UART_Transmit(&huart4, buff, strlen((char*)buff), HAL_MAX_DELAY);

      sprintf((char*)buff, "aY= %f\n\r", Tlm_Tx_buff[4]);
      HAL_UART_Transmit(&huart4, buff, strlen((char*)buff), HAL_MAX_DELAY);

      sprintf((char*)buff, "aZ= %f\n\r", Tlm_Tx_buff[5]);
      HAL_UART_Transmit(&huart4, buff, strlen((char*)buff), HAL_MAX_DELAY); */

      //send the telemetry array typecasted
      HAL_UART_Transmit(&huart4, (uint8_t*)Tlm_Tx_buff, 24, HAL_MAX_DELAY);

      //tlm_counter = 0;

    }
    
    //tlm_counter += 1;
    main_counter += 1;
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
