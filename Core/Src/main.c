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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */





void display7SEG_B(int num)
{

	if(num<0||num>9)return;

	int carry[7];

	int val;

	switch(num)
	{

	case 0:{
		val=1;
		break;
	}
	case 1:
	{
		val=1001111;
		break;
	}
	case 2:
	{
		val=10010;
		break;
	}
	case 3:
	{
		val=110;
		break;
	}
	case 4:
	{
		val=1001100;
		break;
	}
	case 5:
	{
		val=100100;
		break;
	}
	case 6:
	{
		val=100000;
		break;
	}
	case 7:
	{
		val=1111;
		break;
	}
	case 8:
	{
		val=0;
		break;
	}
	case 9:
	{
		val=100;
		break;
	}
	default: break;

	}



	for (int i=6; i>=0; i--)
	{
		int get= val%10;
		val/=10;
		carry[i]=get;
	}


	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, carry[0]);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, carry[1]);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, carry[2]);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, carry[3]);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, carry[4]);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, carry[5]);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, carry[6]);


}










void display7SEG_A(int num)
{

	if(num<0||num>9)return;

	int carry[7];

	int val;

	switch(num)
	{

	case 0:{
		val=1;
		break;
	}
	case 1:
	{
		val=1001111;
		break;
	}
	case 2:
	{
		val=10010;
		break;
	}
	case 3:
	{
		val=110;
		break;
	}
	case 4:
	{
		val=1001100;
		break;
	}
	case 5:
	{
		val=100100;
		break;
	}
	case 6:
	{
		val=100000;
		break;
	}
	case 7:
	{
		val=1111;
		break;
	}
	case 8:
	{
		val=0;
		break;
	}
	case 9:
	{
		val=100;
		break;
	}
	default: break;

	}



	for (int i=6; i>=0; i--)
	{
		int get= val%10;
		val/=10;
		carry[i]=get;
	}


	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, carry[0]);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, carry[1]);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, carry[2]);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, carry[3]);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, carry[4]);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, carry[5]);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, carry[6]);

}




void count_down(int time, int init_segA, int init_segB)
{
	int n= (time)/1000;

	while(n>0)
	{

		display7SEG_A(init_segA);
		display7SEG_B(init_segB);


		init_segA--;
		init_segB--;
		n--;

		HAL_Delay(1000);


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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  int state=0;

  while (1)
  {
    /* USER CODE END WHILE */


	  switch(state)
	    {


	    case 0:
	      {
	        //for lane1
	        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1, GPIO_PIN_SET);
	        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);
	       // HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3, GPIO_PIN_RESET);

	        //for lane2
	        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);
	       // HAL_GPIO_WritePin(LY_LANE2_GPIO_Port,LY_LANE2_PIN, GPIO_PIN_RESET);
	        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6, GPIO_PIN_SET);


	      //  HAL_Delay(3000);

	        count_down(3000,4,2);
	        break;
	      }
	    case 1:
	    {

	      //lane1 stays unchanged

	      //for lane2
	       HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5, GPIO_PIN_SET);
	       HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6, GPIO_PIN_RESET);

	      // HAL_Delay(2000);
	       count_down(2000,1,1);
	       break;

	    }
	    case 2:
	    {

	        //for lane1
	        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1, GPIO_PIN_RESET);
	        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3, GPIO_PIN_SET);

	        //for lane2
	        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4, GPIO_PIN_SET);
	        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5, GPIO_PIN_RESET);

	        //HAL_Delay(3000);
	        count_down(3000,2,4);
	        break;
	      }
	    case 3:
	      {
	        //for lane1
	        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2, GPIO_PIN_SET);
	        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3, GPIO_PIN_RESET);

	       // HAL_Delay(2000);
	        count_down(2000,1,1);
	        break;
	        //lane 2 stays unchanged

	      }
	      default: break;

	    }


	  state++;

	  if(state==4)state=0;

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA1 PA2 PA3 PA4
                           PA5 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10
                           PB11 PB12 PB13 PB3
                           PB4 PB5 PB6 PB7
                           PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
