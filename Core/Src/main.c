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
#include<string.h>
#include<stdio.h>
#include "sim800l.h"
#include "command_handler.h"
#include "adc_handler.h"
#include "lcd_handler.h"
#include "lcd16x2_i2c.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define USART_BUFFER_SIZE 1
#define ENABLE 			  1
#define DISABLE			  0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */

extern uint32_t temperature_status;

char temp_message[50];
char uart1_rcvd_msg_char[50];

uint8_t uart1_rcv_buffer;
uint16_t count = 0;
uint16_t count_uart1 = 0;
uint16_t count_dummy = 0;

const char NL_msg[] = "\r\n";

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void ADC1_Init(void);
static void I2C1_INIT(void);
/* USER CODE BEGIN PFP */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

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
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  I2C1_INIT();



  /* USER CODE BEGIN 2 */
  lcd16x2_i2c_init(&hi2c1);
  HAL_Delay(500);

  ADC1_Init();

  HAL_ADC_Start_IT(&hadc1);
  HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_1);

  //SETTING SIM800L MODULE IN RECEIVING MODE
  check_module_info(&huart1);
  HAL_Delay(100);
  EnorDi_command_echo(&huart1, DISABLE);
  HAL_Delay(100);
  set_text_mode(&huart1);
  HAL_Delay(100);
  set_sms_recieve_format(&huart1);
  HAL_Delay(100);

  HAL_UART_Receive_IT(&huart1, &uart1_rcv_buffer, USART_BUFFER_SIZE);

  /* USER CODE END 2 */

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 INITIALIZATION, TIM1 IS USED TO TRIGGER ADC CONVERSION
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

	  /* USER CODE BEGIN TIM1_Init 0 */

	  /* USER CODE END TIM1_Init 0 */

	  TIM_MasterConfigTypeDef sMasterConfig = {0};
	  TIM_OC_InitTypeDef sOcConfig = {0};

	  /* USER CODE BEGIN TIM1_Init 1 */

	  /* USER CODE END TIM1_Init 1 */
	  htim2.Instance = TIM2;
	  htim2.Init.Prescaler = 4999;
	  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	  htim2.Init.Period = 20000;
	  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
	  {
	    Error_Handler();
	  }

	  sOcConfig.OCIdleState = TIM_OCIDLESTATE_RESET;
	  sOcConfig.OCMode = TIM_OCMODE_PWM1;
	  sOcConfig.OCPolarity = TIM_OCPOLARITY_HIGH;
	  sOcConfig.Pulse = 10000;

	  if(HAL_TIM_OC_ConfigChannel(&htim2, &sOcConfig, TIM_CHANNEL_1) != HAL_OK){
		  Error_Handler();
	  }

	  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}

/**
  * @brief TIM3 INITIALIZATION. TIMER 3 IS USED TO CALL FUNCTION WHICH DISPLAY
  * MESSAGE ON LCD
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 4999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 4000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART1 INITIALIZATION. UART1 IS USED TO COMMUNICATION WITH SIM800L
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
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 INITIALIZATION. UART2 IS USET TO SEND INFORMATIONS TO COMUPER
  * VIA SERIAL INTERFACE
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
  huart2.Init.BaudRate = 9600;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/**
  * @brief RECEIVING DATA FROM SIM800L MODULE
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

	if(huart->Instance == USART1){

		if(count_dummy == 0 ){
			uint8_t dummy_read = uart1_rcv_buffer;
			count_dummy++;
		}else{

			count++;

			if(count > 3){

				if((uart1_rcv_buffer != '\r') && (uart1_rcv_buffer != '.')){
					uart1_rcvd_msg_char[count_uart1] = (char)uart1_rcv_buffer;
					count_uart1++;
				}else{

					if((count_uart1 > 3) && (uart1_rcv_buffer != '.')){

						uart1_rcvd_msg_char[count_uart1] = (char)uart1_rcv_buffer;
						count_uart1++;

					}else{
						uart1_rcvd_msg_char[count_uart1] = '\0';
						uint16_t length = strlen(uart1_rcvd_msg_char);

						HAL_UART_Transmit(&huart2, (uint8_t*)uart1_rcvd_msg_char, length, 200);
						HAL_UART_Transmit(&huart2, (uint8_t*)NL_msg, 2, 2);

						if(count>10){
									PARSE_SMS_MESSAGE(uart1_rcvd_msg_char);
						}

						count_uart1 = 0;
						count = 0;
						memset(&uart1_rcvd_msg_char, 0, sizeof(uart1_rcvd_msg_char));
					}

				}

			}
		}
		HAL_UART_Receive_IT(&huart1, &uart1_rcv_buffer, USART_BUFFER_SIZE);
	}

}

/**
  * @brief CONVERSION OF TEMPERATURE, SIGNALIZATION OF EXCEEDING TEMPERATURE VIA SMS
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){

	uint32_t adc_value = HAL_ADC_GetValue(&hadc1);
	float temperature = CONV_ADC_VOLTAGE_TO_TEMPERATURE(adc_value);

	sprintf(temp_message, "Temp val: %f\n\r", temperature);

	if(temperature_status == CORRECT_TEMPERATURE){

		if(temperature >= MAX_THRESHOLD_TEMPERATURE){
			temperature_status = INCORRECT_TEMPERATURE;
			HAL_UART_Transmit(&huart2, (uint8_t*)sms_msg_temperature_exceeded, strlen(sms_msg_temperature_exceeded), HAL_MAX_DELAY);
			// SYGNALIZATION OF EXCEEDING TEMPERATURE ON LCD
			set_msg_to_print_on_lcd(sms_msg_temperature_exceeded);
			print_on_lcd();
			// SYGNALIZATION OF EXCEEDING TEMPERATURE VIA SMS
			sms_send(&huart1, sms_msg_temperature_exceeded);
			//SETTING SMI800L MODULE BACK TO RECEIVE MODE
			set_text_mode(&huart1);
			set_sms_recieve_format(&huart1);
		}

	}else if(temperature_status == INCORRECT_TEMPERATURE){

		if(temperature <= MIN_THRESHOLD_TEMPERATURE){
			temperature_status = CORRECT_TEMPERATURE;
			HAL_UART_Transmit(&huart2, (uint8_t*)sms_msg_temperature_correct, strlen(sms_msg_temperature_correct), HAL_MAX_DELAY);
			// SYGNALIZATION OF COMING BACK TO CORRECT TEMPERATURE ON LCD
			set_msg_to_print_on_lcd(sms_msg_temperature_correct);
			print_on_lcd();
			// SYGNALIZATION OF COMING BACK TO CORRECT TEMPERATURE VIA SMS
			sms_send(&huart1, sms_msg_temperature_correct);
			//SETTING SMI800L MODULE BACK TO RECEIVE MODE
			set_text_mode(&huart1);
			set_sms_recieve_format(&huart1);
		}

	}

	HAL_UART_Transmit(&huart2, (uint8_t*)temp_message, strlen(temp_message), HAL_MAX_DELAY);
}


/**
  * @brief TIMER CALLBACK USED AS A HELPER FUNCTION TO DISPLAY MESSAGES ON LCD
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

	if(htim->Instance == TIM3){

		print_on_lcd();
	}
}

static void ADC1_Init(void){

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
	  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
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

static void I2C1_INIT(void){

	hi2c1.Instance = I2C1;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_16_9;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

	if(HAL_I2C_Init(&hi2c1) != HAL_OK){
		Error_Handler();
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
