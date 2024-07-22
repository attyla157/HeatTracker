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
#include "math.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"
#include "MLX90640_API.cpp"
//#include "MLX90640_I2C_Driver.cpp"
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
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
#define  FPS2HZ   0x02
#define  FPS4HZ   0x03
#define  FPS8HZ   0x04
#define  FPS16HZ  0x05
#define  FPS32HZ  0x06

#define  MLX90640_ADDR 0x33
#define	 RefreshRate FPS16HZ
#define  TA_SHIFT 8 //Default shift for MLX90640 in open air

static uint16_t eeMLX90640[832];
static float mlx90640To[768];
uint16_t frame[834];
float emissivity=0.95;
int status;
const char density[] = "            `-.',~\"\_:;^r>*?|\\/Licl7vz1xt{}]Ffujy2SoaZemwXPEhk6$9qKOdHDR8MWgN#BQ@"; // _.,-=+:;cba!?0123456789$W#@Ñ  Ñ@#W$9876543210?!abc;:+=-,._
//@QB#NgWM8RDHdOKq9$6khEPXwmeZaoS2yjufF]}{tx1zv7lciL/\\|?*>r^;:_\"~,'.-`
//`-.',~"\_:;^r>*?|\\/Licl7vz1xt{}]Ffujy2SoaZemwXPEhk6$9qKOdHDR8MWgN#BQ@
int output_pixel_value;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
 void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
struct Cordinates
{
	int x;
	int y;
};
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
    return 1;
}


void IncHorizontalAngle(void)
{

	htim2.Instance->CCR1 = htim2.Instance->CCR1 + 1;
	if(htim2.Instance->CCR1 > 125) htim2.Instance->CCR1 = 125;

}
void DecHorizontalAngle(void)
{

	htim2.Instance->CCR1 = htim2.Instance->CCR1 - 1;
	if(htim2.Instance->CCR1 < 25) htim2.Instance->CCR1 = 25;

}
void IncVerticalAngle(void)
{

	htim2.Instance->CCR2 = htim2.Instance->CCR2 + 1;
	if(htim2.Instance->CCR2 > 100) htim2.Instance->CCR2 = 100;

}
void DecVerticalAngle(void)
{

	htim2.Instance->CCR2 = htim2.Instance->CCR2 - 1;
	if(htim2.Instance->CCR2 < 50) htim2.Instance->CCR2 = 50;

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
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
		MLX90640_SetRefreshRate(MLX90640_ADDR, RefreshRate);
		MLX90640_SetChessMode(MLX90640_ADDR);
		paramsMLX90640 mlx90640;
	  status = MLX90640_DumpEE(MLX90640_ADDR, eeMLX90640);
	  if (status != 0) printf("\r\nload system parameters error with code:%d\r\n",status);
	  status = MLX90640_ExtractParameters(eeMLX90640, &mlx90640);
	  if (status != 0) printf("\r\nParameter extraction failed with error code:%d\r\n",status);
	  HAL_UART_Transmit(&huart2, "Hello world!", 12, HAL_MAX_DELAY);
	  float highiest_temp[2];
	  			highiest_temp[0] = 0;
	  			highiest_temp [1] = 1;
	  			float lowest_temp[2];
	  			lowest_temp[0] = 30;
	  			lowest_temp [1] = 0;

	  //starting the timers
	  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	  htim2.Instance->CCR1 = 75;
	  htim2.Instance->CCR2 = 75;
 	  HAL_Delay(2000);
	 // htim2.Instance->CCR1 = 125;
	 //htim2.Instance->CCR2 = 125 ;
	  struct Cordinates my_cordinates ;
	 // HANDLE hConsole = GetStdHandle(STD_OUTPUT_HANDLE);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	  while (1)
	  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		  /*
		  	  htim2.Instance->CCR1 = 25;  // duty cycle is 2. ms position -90
		  	  HAL_Delay(2000);
		  	  htim2.Instance->CCR1 = 50;  // duty cycle is .5 ms position 0
		  	  HAL_Delay(2000);
		  	  htim2.Instance->CCR1 = 75;  // duty cycle is 1.5 ms position"0"
		  	  HAL_Delay(2000);
		  	  htim2.Instance->CCR1 = 100;  // duty cycle is 2. ms position -90
		  	  HAL_Delay(2000);
		  	  htim2.Instance->CCR1 = 125;  // duty cycle is 2. ms position -90
		  	  HAL_Delay(2000);
		  	  htim2.Instance->CCR1 = 150;  // duty cycle is 2. ms position -90
		  	  HAL_Delay(2000);

		*/
			int status = MLX90640_GetFrameData(MLX90640_ADDR, frame);
			int highiest_temp_pixel_number[2];

			if (status < 0)
			{
				//printf("GetFrame Error: %d\r\n",status);
			}
			float vdd = MLX90640_GetVdd(frame, &mlx90640);
			float Ta = MLX90640_GetTa(frame, &mlx90640);

			float tr = Ta - TA_SHIFT; //Reflected temperature based on the sensor ambient temperature
			//printf("vdd:  %f Tr: %f\r\n",vdd,tr);
			MLX90640_CalculateTo(frame, &mlx90640, emissivity , tr, mlx90640To);

			printf("\r\n==========================IAMLIUBO MLX90640 WITH STM32 SWI2C EXAMPLE Github:github.com/imliubo==========================\r\n");
			highiest_temp[0] = mlx90640To[0];
			system("Color 07");
			for(int i = 768; i > 0; i--)
			{
				if(i%32 == 0 && i != 768)
				{
					printf("\r\n");
				}

					if (highiest_temp[0] < mlx90640To[i])
					{
						highiest_temp_pixel_number[0] = i;
						highiest_temp[0] = mlx90640To[i];
					}

				output_pixel_value = (mlx90640To[i] -20) / (/*highiest_temp[1]*/50-20) * (sizeof(density)/sizeof(density[0]));
				//printf("%2.2f ",mlx90640To[i]);
				if(output_pixel_value < 0) output_pixel_value = 0;
				if(output_pixel_value > (sizeof(density)/sizeof(density[0]))) output_pixel_value = (sizeof(density)/sizeof(density[0]));

				if(i == highiest_temp_pixel_number[1] )
				{
					printf("\x1b[31m" "%c" "\x1b[0m", density[output_pixel_value]);
				}
				else
				{
				printf("%c", density[output_pixel_value]);
				}
			}
			if((highiest_temp[0] > highiest_temp[1]+2) || (highiest_temp[0] < highiest_temp[1]-2 ))
			{
				highiest_temp[1] = highiest_temp[0];
				highiest_temp_pixel_number[1] = highiest_temp_pixel_number[0];
			}
			//finding cordinates of hotest point
			my_cordinates.x = highiest_temp_pixel_number[1] % 32;
			my_cordinates.y = ceil( (float) highiest_temp_pixel_number[1] /32);
			//moving the motors to center on the hotest points

			if(my_cordinates.x > 16) DecHorizontalAngle();
			else IncHorizontalAngle();

			if(my_cordinates.y > 12) DecVerticalAngle();
			else IncVerticalAngle();



			highiest_temp[1] = highiest_temp[0]+5;
			lowest_temp[1] = lowest_temp[0]-5;
			printf("\r\n==========================IAMLIUB0 MLX90640 WITH STM32 SWI2C EXAMPLE Github:github.com/imliubo==========================\r\n");

			printf( "\x1B[2J" );
	  }
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

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
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
 void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
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
  htim2.Init.Prescaler = 959;
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
  huart2.Init.BaudRate = 230400;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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
