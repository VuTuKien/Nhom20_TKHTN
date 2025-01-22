/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DHT22_PORT GPIOB
#define DHT22_PIN GPIO_PIN_9
uint8_t RH1, RH2, TC1, TC2, SUM, CHECK;
uint32_t pMillis, cMillis;
float tCelsius = 0;
float tFahrenheit = 0;
float RH = 0;

char uart_buffer[50]; // Buffer chuoi uart
uint8_t send_temp = 1; // Co nhiet do do am
uint8_t data_RX[2];
uint16_t T, H;
const uint8_t digits[10] = {
    0x3F, // 0
    0x06, // 1
    0x5B, // 2
	0b01001111, // 3
    0x66, // 4
    0x6D, // 5
    0x7D, // 6
    0x07, // 7
    0x7F, // 8
    0x6F  // 9
};

// C?u hình các chân GPIO tuong ?ng v?i các segment (a, b, c, d, e, f, g)
GPIO_TypeDef *segmentPorts[] = {GPIOA, GPIOA, GPIOA, GPIOA, GPIOB, GPIOA, GPIOA};
uint16_t segmentPins[] = {GPIO_PIN_1, GPIO_PIN_3, GPIO_PIN_5, GPIO_PIN_7, GPIO_PIN_0, GPIO_PIN_2, GPIO_PIN_4};
// M?ng chân GPIO di?u khi?n ch?n t?ng LED
GPIO_TypeDef* digitPorts[3] = {GPIOB, GPIOB, GPIOB}; // Gi? s? dùng port A
uint16_t digitPins[3] = {GPIO_PIN_1, GPIO_PIN_10, GPIO_PIN_11}; // Chân PA7, PA8, PA9 dùng d? ch?n t?ng LED
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;

osThreadId defaultTaskHandle;
osThreadId myTask02Handle;
osThreadId myTask03Handle;
osThreadId myTask04Handle;
osThreadId myTask05Handle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void const * argument);
void StartTask02(void const * argument);
void StartTask03(void const * argument);
void StartTask04(void const * argument);
void StartTask05(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void microDelay (uint16_t delay)
{
  __HAL_TIM_SET_COUNTER(&htim1, 0);
  while (__HAL_TIM_GET_COUNTER(&htim1) < delay);
}
void DisplayDigit(uint8_t digit, uint8_t position) {
	    // T?t t?t c? các LED (ngan trùng l?p)
	    HAL_GPIO_WritePin(GPIOB, digitPins[0] | digitPins[1] | digitPins[2], GPIO_PIN_RESET);
	    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
	    uint8_t segments = digits[digit];
	    for (int i = 0; i < 7; i++) {
	    	if (segments & (1 << i)) {
	    	            // N?u bit th? i là 1, b?t segment, t?c là n?i chân v?i GND (LOW)
	    	            HAL_GPIO_WritePin(segmentPorts[i], segmentPins[i], GPIO_PIN_SET);  // B?t
	    	        } else {
	    	            // N?u bit th? i là 0, t?t segment, t?c là n?i chân v?i VCC (HIGH)
	    	            HAL_GPIO_WritePin(segmentPorts[i], segmentPins[i], GPIO_PIN_RESET);  // T?t
	    	        }
	    }

	    // B?t LED 7 thanh theo v? trí
	    if (position == 0) HAL_GPIO_WritePin(digitPorts[0], digitPins[0], GPIO_PIN_SET); // LED1
	    if (position == 1){ HAL_GPIO_WritePin(digitPorts[1], digitPins[1], GPIO_PIN_SET);
	    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);// LED2
	    }
	    if (position == 2) HAL_GPIO_WritePin(digitPorts[2], digitPins[2], GPIO_PIN_SET); // LED3
	}
	void DisplayNumber(uint16_t number) {
	    uint8_t hundreds = number / 100;        // L?y hàng tram
	    uint8_t tens = (number / 10) % 10;     // L?y hàng ch?c
	    uint8_t units = number % 10;           // L?y hàng don v?

	    // Hi?n th? t?ng ch? s? v?i quét nhanh
	    DisplayDigit(hundreds, 0);
	    HAL_Delay(1); // D?ng ng?n d? hi?n th?
	    DisplayDigit(tens, 1);
	    HAL_Delay(1);
	    DisplayDigit(units, 2);
	    HAL_Delay(1);
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
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim1);
   HAL_UART_Init(&huart1);
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityHigh, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of myTask02 */
  osThreadDef(myTask02, StartTask02, osPriorityAboveNormal, 0, 256);
  myTask02Handle = osThreadCreate(osThread(myTask02), NULL);

  /* definition and creation of myTask03 */
  osThreadDef(myTask03, StartTask03, osPriorityNormal, 0, 128);
  myTask03Handle = osThreadCreate(osThread(myTask03), NULL);

  /* definition and creation of myTask04 */
  osThreadDef(myTask04, StartTask04, osPriorityBelowNormal, 0, 256);
  myTask04Handle = osThreadCreate(osThread(myTask04), NULL);

  /* definition and creation of myTask05 */
  osThreadDef(myTask05, StartTask05, osPriorityLow, 0, 128);
  myTask05Handle = osThreadCreate(osThread(myTask05), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA1 PA2 PA3 PA4
                           PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB10 PB11
                           PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
	uint8_t DHT22_Start (void)
		{
		  uint8_t Response = 0;
		  GPIO_InitTypeDef GPIO_InitStructPrivate = {0};
		  GPIO_InitStructPrivate.Pin = DHT22_PIN;
		  GPIO_InitStructPrivate.Mode = GPIO_MODE_OUTPUT_PP;
		  GPIO_InitStructPrivate.Speed = GPIO_SPEED_FREQ_LOW;
		  GPIO_InitStructPrivate.Pull = GPIO_NOPULL;
		  HAL_GPIO_Init(DHT22_PORT, &GPIO_InitStructPrivate); // set the pin as output
		  HAL_GPIO_WritePin (DHT22_PORT, DHT22_PIN, 0);   // pull the pin low
		  microDelay (1300);   // wait for 1300us
		  HAL_GPIO_WritePin (DHT22_PORT, DHT22_PIN, 1);   // pull the pin high
		  microDelay (30);   // wait for 30us
		  GPIO_InitStructPrivate.Mode = GPIO_MODE_INPUT;
		  GPIO_InitStructPrivate.Pull = GPIO_PULLUP;
		  HAL_GPIO_Init(DHT22_PORT, &GPIO_InitStructPrivate); // set the pin as input
		  microDelay (40);
		  if (!(HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN)))
		  {
		    microDelay (80);
		    if ((HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN))) Response = 1;
		  }
		  pMillis = HAL_GetTick();
		  cMillis = HAL_GetTick();
		  while ((HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN)) && pMillis + 2 > cMillis)
		  {
		    cMillis = HAL_GetTick();
		  }
		  return Response;
		}

		uint8_t DHT22_Read (void)
		{
		  uint8_t a,b;
		  for (a=0;a<8;a++)
		  {
		    pMillis = HAL_GetTick();
		    cMillis = HAL_GetTick();
		    while (!(HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN)) && pMillis + 2 > cMillis)
		    {  // wait for the pin to go high
		      cMillis = HAL_GetTick();
		    }
		    microDelay (40);   // wait for 40 us
		    if (!(HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN)))   // if the pin is low
		      b&= ~(1<<(7-a));
		    else
		      b|= (1<<(7-a));
		    pMillis = HAL_GetTick();
		    cMillis = HAL_GetTick();
		    while ((HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN)) && pMillis + 2 > cMillis)
		    {  // wait for the pin to go low
		      cMillis = HAL_GetTick();
		    }
		  }
		  return b;
		}
  /* Infinite loop */
  for(;;)
  {
//	  lastWakeTime1 = xTaskGetTickCount(); // Lấy tick hiện tại làm mốc tham chiếu
	  if(DHT22_Start())
  {
    RH1 = DHT22_Read(); // First 8bits of humidity
    RH2 = DHT22_Read(); // Second 8bits of Relative humidity
    TC1 = DHT22_Read(); // First 8bits of Celsius
    TC2 = DHT22_Read(); // Second 8bits of Celsius
    SUM = DHT22_Read(); // Check sum
    CHECK = RH1 + RH2 + TC1 + TC2;
    if (CHECK == SUM)
    {
      if (TC1>127) // If TC1=10000000, negative temperature
      {
        tCelsius = (float)TC2/10*(-1);
		T=tCelsius*10;
      }
      else
      {
        tCelsius = (float)((TC1<<8)|TC2)/10;
		T=tCelsius*10;
      }
      tFahrenheit = tCelsius * 9/5 + 32;
      RH = (float) ((RH1<<8)|RH2)/10;
      H=RH*10;
    }
  }
	  // Trì hoãn đến thời điểm tiếp theo
	   osDelay(5000);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void const * argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
  for(;;)
  {

//  lastWakeTime2 = xTaskGetTickCount(); // Lấy tick hiện tại làm mốc tham chiếu
	  // Gui nhiet do qua UART
	    sprintf(uart_buffer, "Temperature: %.1fC\r\n", tCelsius); // Gui nhiet do
		HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
		// Trì hoãn đến thời điểm tiếp theo
		 osDelay(5000);
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the myTask03 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void const * argument)
{
  /* USER CODE BEGIN StartTask03 */
  /* Infinite loop */
  for(;;)
  {

//  lastWakeTime3 = xTaskGetTickCount(); // Lấy tick hiện tại làm mốc tham chiếu
  for(int i=0;i<=200;i++){
	     DisplayNumber(T);
	  } HAL_GPIO_WritePin(GPIOB, digitPins[0] | digitPins[1] | digitPins[2], GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
	  osDelay(5000);
  }
  /* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_StartTask04 */
/**
* @brief Function implementing the myTask04 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask04 */
void StartTask04(void const * argument)
{
  /* USER CODE BEGIN StartTask04 */
  /* Infinite loop */
  for(;;)
  {

//  lastWakeTime4 = xTaskGetTickCount(); // Lấy tick hiện tại làm mốc tham chiếu
	  sprintf(uart_buffer, "Humidity: %.0f%%\r\n", RH); // Gui do am
	HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
	 osDelay(5000);
  }
  /* USER CODE END StartTask04 */
}

/* USER CODE BEGIN Header_StartTask05 */
/**
* @brief Function implementing the myTask05 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask05 */
void StartTask05(void const * argument)
{
  /* USER CODE BEGIN StartTask05 */
  /* Infinite loop */
  for(;;)
  {

//  lastWakeTime5 = xTaskGetTickCount(); // Lấy tick hiện tại làm mốc tham chiếu
	  for(int i=0;i<=200;i++){
	  DisplayNumber(H);
	  }
	  HAL_GPIO_WritePin(GPIOB, digitPins[0] | digitPins[1] | digitPins[2], GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
	  osDelay(5000);
  }
  /* USER CODE END StartTask05 */
}

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
