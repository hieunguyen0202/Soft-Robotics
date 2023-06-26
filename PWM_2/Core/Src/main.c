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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f1xx_hal.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// Khai báo bi?n và c?u trúc GPIO
GPIO_InitTypeDef GPIO_InitStruct;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t var = 0;
uint32_t temp1 = 0;
uint32_t temp2 = 0;
uint8_t ok = 0;
int PWM = 0;
int count = 0;
int count_2 = 0;
uint32_t PWM_Receive = 0;
uint32_t PWM_Select = 0;
int PWM_Temp = 0;
uint8_t rxBuffer[4]; //rx
uint8_t txBuffer[5]; //tx
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
//void Read_sensor(void);
void ConvertIntToByte(uint32_t value);
void ConvertByteToInt(uint8_t value[4]);
void HAL_Delay10us(uint32_t delay);
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
  __HAL_RCC_GPIOA_CLK_ENABLE(); // B?t clock cho GPIOA
  GPIO_InitStruct.Pin = GPIO_PIN_0;           // Ch?n chân PA6
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;    // Ch?n ch? d? d?u vào
  GPIO_InitStruct.Pull = GPIO_NOPULL;       // Không kích ho?t di?n tr? kéo lên/kép xu?ng
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);   // C?u hình GPIOA theo dúng thi?t l?p trên
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
uint8_t data[] = "HelloHieu";
HAL_UART_Receive_DMA(&huart1,rxBuffer, sizeof(rxBuffer));
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		ConvertByteToInt(rxBuffer); 
		 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);

//		 uint8_t i;
//    for (i = 0; i < 10; i++) {
//        if (i < PWM_Select) 
//					{
//            PWM_Temp = 750;
//					__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_1,PWM_Temp);
//        } else 
//				{
//          PWM_Temp = 400;
//					__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_1,PWM_Temp);
//        }
//        HAL_Delay(0.1); // Ð? tr? gi?a các xung là 10 us, tuong ?ng v?i t?n s? xung 1 kHz
//			}
		
		//code new
		__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_1,PWM_Select);
		// Ð?c giá tr? c?a chân PA0
    GPIO_PinState gpioState = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);

    // Ki?m tra tr?ng thái c?a chân PA0
    if (gpioState == GPIO_PIN_RESET)
    {
      // Tr?ng thái LOW
         PWM_Temp = 400;
					__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_1,PWM_Temp);
    }
    else
    {
      // Tr?ng thái HIGH
        PWM_Temp = 750;
     	__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_1,PWM_Temp);
    }

		
		
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
//void Read_sensor(void)
//{   
//	
//		HAL_ADC_Start(&hadc1);
//		HAL_Delay(100);
//		var = HAL_ADC_GetValue(&hadc1);
////	  temp1 = var*3.3/1023;
////	  temp2 = temp1*50000-25000;
//		HAL_ADC_Stop(&hadc1);
//}

void ConvertIntToByte(uint32_t value)
{ int value_1 = (int)value;
	txBuffer[4] = value_1%10 + 0x30;
	value_1 = value_1/10;
	txBuffer[3] = value_1%10 + 0x30;
	value_1 = value_1/10;
	txBuffer[2] = value_1%10 + 0x30;
	value_1 = value_1/10;
	txBuffer[1] = value_1%10 + 0x30;
	value_1 = value_1/10;
	txBuffer[0] = value_1 + 0x30;
	
}
void ConvertByteToInt(uint8_t value[3])
{    
	if (value[3] == 0x0A) 
	{
        
				 PWM_Receive = ( value[0] - 0x30) * 100;
			   PWM_Receive = PWM_Receive + ( value[1] - 0x30) * 10;
	    		PWM_Receive = PWM_Receive + ( value[2] - 0x30) * 1;
//			PWM_Receive = PWM_Receive + ( value[3] - 0x30) * 1;
		   PWM_Select = PWM_Receive;
}
	else
	{
		     PWM_Receive = 0;
         PWM_Select = 100;
	}
}


void HAL_Delay10us(uint32_t delay) {
    uint32_t tickstart = HAL_GetTick();
    uint32_t wait = delay * (HAL_RCC_GetSysClockFreq() / 10000000);
    while ((HAL_GetTick() - tickstart) < wait) {
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

