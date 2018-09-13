#include "main.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"


/* USER CODE BEGIN Includes */
#include <Adafruit_GFX.h>
#include <Max72xxPanel.h>
#include "LiquidCrystal_I2C.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

I2C_HandleTypeDef hi2c1;

osThreadId defaultTaskHandle;
osThreadId TaskLCDHandle;
osThreadId TaskDisplayHandle;
osThreadId TaskEntradasHandle;
osThreadId TaskSalidasHandle;
osMessageQId colaHandle;

int numberOfHorizontalDisplays = 3;
int numberOfVerticalDisplays = 2;

String tape = " Power Force";
int wait = 50; // In milliseconds
int spacer = 1;
int width = 5 + spacer; // The font width is 5 pixels
int y = 0; // center the text vertically

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
Max72xxPanel matrix = Max72xxPanel(&hspi1, numberOfHorizontalDisplays, numberOfVerticalDisplays);
LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27, &hi2c1 ,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C1_Init(void);
void matrixInit(void);  //Inicia los paneles en el sentido correcto
void StartDefaultTask(void const * argument);
void lcdTask(void const * argument);
void displayTask(void const * argument);
void entradasTask(void const * argument);
void salidasTask(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_I2C1_Init();

  /* USER CODE BEGIN 2 */

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

   /* Create the thread(s) */
   /* definition and creation of defaultTask */
   osThreadDef(defaultTask, StartDefaultTask, osPriorityRealtime, 0, 128);
   defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

   /* definition and creation of TaskLCD */
   osThreadDef(TaskLCD, lcdTask, osPriorityIdle, 0, 128);
   TaskLCDHandle = osThreadCreate(osThread(TaskLCD), NULL);

   /* definition and creation of TaskDisplay */
   osThreadDef(TaskDisplay, displayTask, osPriorityIdle, 0, 128);
   TaskDisplayHandle = osThreadCreate(osThread(TaskDisplay), NULL);

   /* definition and creation of TaskEntradas */
   osThreadDef(TaskEntradas, entradasTask, osPriorityIdle, 0, 128);
   TaskEntradasHandle = osThreadCreate(osThread(TaskEntradas), NULL);

   /* definition and creation of TaskSalidas */
   osThreadDef(TaskSalidas, salidasTask, osPriorityIdle, 0, 128);
   TaskSalidasHandle = osThreadCreate(osThread(TaskSalidas), NULL);

   /* USER CODE BEGIN RTOS_THREADS */
   /* add threads, ... */
   /* USER CODE END RTOS_THREADS */

   /* Create the queue(s) */
   /* definition and creation of cola */
 /* what about the sizeof here??? cd native code */
   osMessageQDef(cola, 16, uint16_t);
   colaHandle = osMessageCreate(osMessageQ(cola), NULL);

   /* USER CODE BEGIN RTOS_QUEUES */
   /* add queues, ... */
   /* USER CODE END RTOS_QUEUES */
   matrixInit();
   lcd.init();
   lcd.backlight();
   //lcd.autoscroll();
   lcd.setCursor(0,0);
   lcd.print("POWER FORCE");
   lcd.setCursor(0,1);
   lcd.print("Division maquinas");

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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks
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
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

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
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 78;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void matrixInit (void) {
	matrix.init();
	matrix.setPosition(0, 0, 0); // The first display is at <0, 0>
	matrix.setPosition(1, 1, 0); // The second display is at <1, 0>
	matrix.setPosition(2, 2, 0); // The third display is at <2, 0>

	matrix.setPosition(3, 5, 1);
	matrix.setPosition(4, 4, 1);
	matrix.setPosition(5, 3, 1);

	//  ...
	matrix.setRotation(0, 1);    // The first display is position upside down
	matrix.setRotation(1, 1);
	matrix.setRotation(2, 1);    // The same hold for the last display
	matrix.setRotation(3, 1);
	matrix.setRotation(4, 1);
	matrix.setRotation(5, 1);
}
/* USER CODE END 4 */



/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
     HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

   HAL_Delay(500);

   uint32_t m = HAL_GetTick();
   tape.concat(m);
   for ( uint16_t i = 0 ; i < width * tape.length(); i++ ) {

     //matrix.fillScreen(LOW);
	   uint16_t letter = i / width;
	   uint16_t x = (matrix.width() - 1) - i % width;

     //matrix.drawChar(0, 4, tape[0], HIGH, LOW, 1);
     while ( x + width - spacer >= 0 && letter >= 1 ) {
       if ( letter < tape.length() ) {
         matrix.drawChar(x, y, tape[letter], HIGH, LOW, 1);
       }

       letter--;
       x -= width;
     }

     matrix.write(); // Send bitmap to display

     HAL_Delay(wait);
   }
   HAL_Delay(200);
   matrix.fillScreen(LOW);
   if(y>=8){ y = -1;}
   y+=3;




    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* lcdTask function */
void lcdTask(void const * argument)
{
  /* USER CODE BEGIN lcdTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END lcdTask */
}

/* displayTask function */
void displayTask(void const * argument)
{
  /* USER CODE BEGIN displayTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END displayTask */
}

/* entradasTask function */
void entradasTask(void const * argument)
{
  /* USER CODE BEGIN entradasTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END entradasTask */
}

/* salidasTask function */
void salidasTask(void const * argument)
{
  /* USER CODE BEGIN salidasTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END salidasTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
