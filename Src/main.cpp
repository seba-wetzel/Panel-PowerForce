#include "main.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <Adafruit_GFX.h>
#include <Max72xxPanel.h>
#include "LiquidCrystal_I2C.h"
extern "C"{
#include "stm32_tm1637.h"
#include "hal_uart_print.h"
}
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
osThreadId TaskTimmerHandle;
osMessageQId colaHandle;

//Variables del display led
int numberOfHorizontalDisplays = 3;
int numberOfVerticalDisplays = 2;
String tape = " Power Force";
int wait = 50; // In milliseconds
int spacer = 1;
int width = 5 + spacer; // The font width is 5 pixels
int y = 5; // center the text vertically


uint16_t initTime =0;

//Lista de programas
const uint8_t programas [4][24] = {
		{ 1, 1, 2, 2, 5, 5, 2, 2, 5, 5, 2, 2, 5, 5, 2, 2, 5, 5, 2, 2, 5, 5, 2, 1 },
		{ 1, 1, 2, 2, 5, 5, 2, 2, 8, 8, 4, 4, 10, 10, 2, 2, 5, 5, 2, 2,	5, 5, 2, 1 },
		{ 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 10, 10, 10, 10, 9, 7, 7, 4, 4, 3, 3, 2, 2, 1 },
		{ 1, 1, 2, 2, 5, 5, 2, 2, 8, 8, 4, 4, 10, 10, 2, 2, 5, 5, 2, 2, 5, 5, 2, 1 }};

//Buffer donde guardar y modificar el programa seleccionado
uint8_t programaSeleccionado [24];

maquina_s maquina = { NO_INIT, M, 0, minTimmer, A0 , 0};

uint16_t pinEntradas[] = { VEL_UP_Pin, VEL_DOWN_Pin, ENTER_Pin, PROGRAMA_Pin,
PENDIENTE_UP_Pin, PENDIENTE_DOWN_Pin, TIMMER_Pin, VIENTO_Pin, START_Pin,
STOP_Pin };

GPIO_TypeDef* puertoEntradas[] = { VEL_UP_GPIO_Port, VEL_DOWN_GPIO_Port,
ENTER_GPIO_Port, PROGRAMA_GPIO_Port, PENDIENTE_UP_GPIO_Port,
PENDIENTE_DOWN_GPIO_Port, TIMMER_GPIO_Port, VIENTO_GPIO_Port,
START_GPIO_Port, STOP_GPIO_Port };

uint16_t pinSalida[] = {ON_OFF_Pin, VEL_U_Pin, VEL_D_Pin, LED_Pin };
GPIO_TypeDef* puertoSalidas[] = {ON_OFF_GPIO_Port, VEL_U_GPIO_Port, VEL_D_GPIO_Port, LED_GPIO_Port};

enum {
	POWER,
	UP,
	DOWN,
	LED
};


#define digitalWrite(P, X) HAL_GPIO_WritePin((GPIO_TypeDef*)puertoSalidas[P],pinSalida[P], (GPIO_PinState)X);



float porcentaje = 0;
uint8_t partes = 0;
uint8_t paso = 0;

uint32_t speed = 0;
float frecuencia = 0;
uint16_t flag = 0;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
Max72xxPanel matrix = Max72xxPanel(&hspi1, numberOfHorizontalDisplays,
		numberOfVerticalDisplays);
LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27, &hi2c1, 16, 2); // set the LCD address to 0x27 for a 16 chars and 2 line display
		/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C1_Init(void);
extern void initialise_monitor_handless (void);
void matrixInit(void);  //Inicia los paneles en el sentido correcto
void StartDefaultTask(void const * argument);
void lcdTask(void const * argument);
void displayTask(void const * argument);
void entradasTask(void const * argument);
void salidasTask(void const * argument);
void timmerTask (void const * args);
extern boton_e botonRead(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void printMenssage(void);
void drawProgram(uint8_t *);
void turnOffAllScreen(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 *
 * @retval None
 */
int main(void) {
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
	osThreadDef(defaultTask, StartDefaultTask, osPriorityIdle, 0, 128);
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

	/* definition and creation of TimmerSalidas */
	osThreadDef(TaskTimmer, timmerTask, osPriorityIdle, 0, 128);
	TaskTimmerHandle = osThreadCreate(osThread(TaskTimmer), NULL);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* Create the queue(s) */
	/* definition and creation of cola */
	/* what about the sizeof here??? cd native code */
	osMessageQDef(cola, 16, uint8_t);
	colaHandle = osMessageCreate(osMessageQ(cola), NULL);

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */
	matrixInit();
	lcd.init();
	tm1637Init();
	tm1637SetBrightness(0);
	printMenssage();
	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

	}
	/* USER CODE END 3 */

}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {

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
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure the Systick interrupt time
	 */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	/**Configure the Systick
	 */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

	  /* SysTick_IRQn interrupt configuration */
	  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);

}

/* SPI1 init function */
static void MX_SPI1_Init(void) {
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
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* USART1 init function */
static void MX_USART1_UART_Init(void) {

	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
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
static void MX_I2C1_Init(void) {

	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 78;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

static void MX_GPIO_Init(void) {

	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, VEL_U_Pin | VEL_D_Pin | ON_OFF_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : LED_Pin */
	GPIO_InitStruct.Pin = LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : VEL_U_Pin VEL_D_Pin ON_OFF_Pin */
	GPIO_InitStruct.Pin = VEL_U_Pin | VEL_D_Pin | ON_OFF_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : CS_Pin */
	GPIO_InitStruct.Pin = CS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(CS_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : VEL_UP_Pin VEL_DOWN_Pin ENTER_Pin PROGRAMA_Pin START_Pin STOP_Pin */

	GPIO_InitStruct.Pin = VEL_UP_Pin | VEL_DOWN_Pin | ENTER_Pin | PROGRAMA_Pin 	| START_Pin | STOP_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : PENDIENTE_UP_Pin PENDIENTE_DOWN_Pin TIMMER_Pin VIENTO_Pin */
	GPIO_InitStruct.Pin = PENDIENTE_UP_Pin | PENDIENTE_DOWN_Pin | TIMMER_Pin | VIENTO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : encoder_Pin */
	GPIO_InitStruct.Pin = encoder_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(encoder_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */

void matrixInit(void) {
	matrix.init();
	matrix.setPosition(0, 0, 0);
	matrix.setPosition(1, 1, 0);
	matrix.setPosition(2, 2, 0);
	matrix.setPosition(3, 5, 1);
	matrix.setPosition(4, 4, 1);
	matrix.setPosition(5, 3, 1);
	matrix.setRotation(0, 1);
	matrix.setRotation(1, 1);
	matrix.setRotation(2, 1);
	matrix.setRotation(3, 1);
	matrix.setRotation(4, 1);
	matrix.setRotation(5, 1);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

	volatile static bool capturaIndex = false;
	volatile static uint32_t captura1, captura2,periodo;
	flag++;
	if(!capturaIndex){
		captura1 = HAL_GetTick();
		capturaIndex = true;
	}
	else if (capturaIndex){
		captura2 = HAL_GetTick();
		periodo = captura2 - captura1;
		frecuencia = 1000.f/periodo;
		capturaIndex = false;
	}
}

/* USER CODE END 4 */

/* StartDefaultTask function */
void StartDefaultTask(void const * argument) {

	/* USER CODE BEGIN 5 */
	/* Infinite loop */
	boton_e boton = NONE_BOTON;
	for (;;) {
		osEvent evt = osMessageGet(colaHandle, osWaitForever);
		if (evt.status == osEventMessage) {
			boton = (boton_e) evt.value.signals;
		}
		switch (boton) {
				case START_BOTON:
					if ((maquina.run == PAUSE) | (maquina.run == NO_INIT)) {
						digitalWrite(LED, HIGH);
					}
		            if ( (maquina.run != START)) {
									maquina.run = START;
								    porcentaje = (maquina.timmer * 1000) /23;
								    initTime = maquina.timmer;
								}
					break;
		        case PAUSA_BOTON:
		            digitalWrite(LED, LOW);

		            break;

				case VEL_UP_BOTON:
					if ( (maquina.run == START)	&& (maquina.programa == M) && (maquina.velocidad < maxSpeed)) {
						maquina.velocidad++;
					}
					break;

				case VEL_DOWN_BOTON:
					if ( (maquina.run == START) && (maquina.programa == M) && (maquina.velocidad > 0)) {
						maquina.velocidad--;
					}
					break;

				case PROGRAMA_BOTON:
					if ( ((maquina.run == NO_INIT) | (maquina.run == FINISH))) {
						if (maquina.programa == LAST_PROGRAM) {
							maquina.programa = M;
						} else {
							maquina.programa = (program_e) (((uint8_t) maquina.programa) + 1);
							}
						if(maquina.programa != M){memcpy(programaSeleccionado, programas[(uint8_t) maquina.programa],24); }
						maquina.run = NO_INIT;
						maquina.timmer = minTimmer;
						paso=0;
					}
					break;

				case PENDIENTE_UP_BOTON:
					if ((maquina.programa == M) /*&& (maquina.run == START) && (maquina.programa == M)*/) {
						if (maquina.inclinacion == A60) {

						} else {
							maquina.inclinacion = (angle_e) (((uint8_t) maquina.inclinacion) + 1);
						}
					}
					break;

				case PENDIENTE_DOWN_BOTON:
					if ((maquina.programa == M) /*&& (maquina.run == START) && (maquina.programa == M)*/) {
						if (maquina.inclinacion == A0) {

						} else {
							maquina.inclinacion = (angle_e) (((uint8_t) maquina.inclinacion) - 1);
						}
					}
					break;

				case TIMMER_BOTON:
					if ( ((maquina.run == NO_INIT) | (maquina.run == FINISH))) {
						if(maquina.run == FINISH){
							maquina.run = NO_INIT;
						}
						 maquina.timmer += timmerStep;
						if (maquina.timmer == maxTimmer) {
							maquina.timmer = minTimmer;
						}
					}
					break;

				case VIENTO_BOTON: //Aca solo debe hacer un toggle del pin de salida (no definido aun)
		            ;
					break;

				case STOP_BOTON:
					if ( (maquina.run == START) | (maquina.run == PAUSE)) {
									maquina.run = STOP;
								}
					break;

				default:
					break;
				}

	}
	/* USER CODE END 5 */
}

/* lcdTask function */
void lcdTask(void const * argument) {
	/* USER CODE BEGIN lcdTask */
	/* Infinite loop */
	for (;;) {
		lcd.setCursor(0, 0);
		lcd.print (speed );
		lcd.print("hola");

		lcd.setCursor(0,1);
		lcd.print(maquina.distancia);
		osDelay(500);
	}
	/* USER CODE END lcdTask */
}

/* displayTask function */
void displayTask(void const * argument) {
	/* USER CODE BEGIN displayTask */
	uint8_t segundos = 0;
	uint8_t minutos  = 0;
	/* Infinite loop */
	for (;;) {
		if ((maquina.run != FINISH)){
			tm1637SetBrightness(7);
			segundos = maquina.timmer % 60;
			minutos = (maquina.timmer / 60) % 60;
			tm1637DisplayDecimal(((minutos * 100) + (segundos)), 1);
			if(maquina.timmer == 0){
				tm1637SetBrightness(0);
			}
		}


		   //Si hay un programa dibujado, se borran las barras pasadas
			if ((maquina.run == START) && (maquina.programa != M)) {

				if (((initTime * 1000) - (maquina.timmer * 1000)) >= (porcentaje)) {

					initTime = maquina.timmer;
					//Aca hay que ir borrando la barra del programa
					programaSeleccionado[paso] = 0;
					paso++;
					maquina.velocidad = programaSeleccionado[paso];
				}
			}
			matrix.fillScreen(LOW);

			//Aca se dibuja el programa cuando se esta seleccionando
			if ( (maquina.programa != M) && (maquina.run != FINISH) ) {

				drawProgram(programaSeleccionado);

			} else if ( (maquina.programa == M) && (maquina.run != FINISH) ){
				matrix.drawChar(7, 1, 'M', HIGH, LOW, 2);
				matrix.write();
			}

			else if ( (maquina.run == FINISH)){
				matrix.drawChar(7, 1, 'T', HIGH, LOW, 2);
				matrix.write();
			}
			osDelay(250);
		}

	/* USER CODE END displayTask */
}

/* entradasTask function */
void entradasTask(void const * argument) {
	/* USER CODE BEGIN entradasTask */
	/* Infinite loop */
/*
	uint8_t suma = 0;
	volatile uint8_t activated = 0;
	volatile uint8_t previusActivated = 0;
	volatile uint8_t step = 0;
	GPIO_PinState pines[10]; //Buffer para guardar el estado de las entradas
*/
	volatile boton_e activated;
	volatile boton_e previusActivated;
	volatile uint8_t step = 0;

	for (;;) {
		activated = botonRead();

			if (activated != previusActivated) {
				previusActivated = activated;
				step = 0;
			} else {
				if (step < 5) {
					step++;
				}
			}
			while (activated == botonRead()) {
				if ( activated == START_BOTON | activated == PAUSA_BOTON) {
					do {
						osDelay(500);
					} while (activated == botonRead());
				} else {
					osDelay(150u - (2u * step));
				}
				break;
			}
			//Envio del boton a la cola
			osMessagePut(colaHandle,  activated, osWaitForever);


		osDelay(1);
	}
	/* USER CODE END entradasTask */
}

/* salidasTask function */
void salidasTask(void const * argument) {
	/* USER CODE BEGIN salidasTask */
	/* Infinite loop */

	for (;;) {
		if((maquina.run == START)){
			if(flag != 0){
			speed = (uint32_t)frecuencieToSpeed(frecuencia);
			flag = 0;
			}
			else if (flag == 0){
				speed = 0;
			}
		}
		printNumberLn(speed, 10);
		osDelay(500);
	}
	/* USER CODE END salidasTask */
}
/* timmerTask function */
void timmerTask (void const * args){
	/* USER CODE BEGIN timmerTask */
	/* Infinite loop */
	for (;;) {

            osDelay(500);
			if((maquina.run == START) && (maquina.timmer >0) ){
				maquina.timmer -= 1;
				maquina.distancia += (maquina.velocidad*1000)/3600;
			}
			if ((maquina.run == START) &&(maquina.timmer == 0)){
				maquina.velocidad = 0;
				tm1637SetBrightness(0);
				maquina.run = FINISH;
				paso = 0;
			}

			osDelay(1000);
	}
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM1 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
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
void _Error_Handler(char *file, int line) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

void drawProgram(uint8_t* barras) {

	for (uint8_t i = 0; i <= 24; i++) {
		matrix.writeLine(i, (16 - barras[i]), i, 16, HIGH);

	}
	matrix.write();
}

void turnOffAllScreen(void){
	matrix.fillScreen(LOW);
	matrix.write();
	tm1637SetBrightness(0);
	lcd.clear();
}

void printMenssage(void) {

	for (uint16_t i = 0; i < width * tape.length(); i++) {

		uint16_t letter = i / width;
		uint16_t x = (matrix.width() - 1) - i % width;

		while (x + width - spacer >= 0 && letter >= 1) {
			if (letter < tape.length()) {
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
	matrix.write();

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
