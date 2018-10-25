/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */
#define millis() HAL_GetTick()
#define diametroRodillo 12
#define pi 3.14159265359
#define frecuencieToSpeed(x) ((pi*diametroRodillo*x)/100)

/* USER CODE END Includes */
/* Private define ------------------------------------------------------------*/
//SPI
#define SCK_Pin GPIO_PIN_5
#define SCK_GPIO_Port GPIOA
#define MOSI_Pin GPIO_PIN_7
#define MOSI_GPIO_Port GPIOA
#define CS_Pin GPIO_PIN_1
#define CS_GPIO_Port GPIOB

//UART
#define Tx_Pin GPIO_PIN_9
#define Tx_GPIO_Port GPIOA
#define Rx_Pin GPIO_PIN_10
#define Rx_GPIO_Port GPIOA

//I2C
#define SCL_Pin GPIO_PIN_6
#define SCL_GPIO_Port GPIOB
#define SDA_Pin GPIO_PIN_7
#define SDA_GPIO_Port GPIOB

//Salidas
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define VEL_U_Pin GPIO_PIN_0
#define VEL_U_GPIO_Port GPIOA
#define VEL_D_Pin GPIO_PIN_1
#define VEL_D_GPIO_Port GPIOA
#define ON_OFF_Pin GPIO_PIN_2
#define ON_OFF_GPIO_Port GPIOA

//Entradas
#define encoder_Pin GPIO_PIN_0
#define encoder_GPIO_Port GPIOB
#define encoder_EXTI_IRQn EXTI0_IRQn
#define VEL_UP_Pin GPIO_PIN_12
#define VEL_UP_GPIO_Port GPIOB
#define VEL_DOWN_Pin GPIO_PIN_13
#define VEL_DOWN_GPIO_Port GPIOB
#define ENTER_Pin GPIO_PIN_14
#define ENTER_GPIO_Port GPIOB
#define PROGRAMA_Pin GPIO_PIN_15
#define PROGRAMA_GPIO_Port GPIOB
#define PENDIENTE_UP_Pin GPIO_PIN_8
#define PENDIENTE_UP_GPIO_Port GPIOA
#define PENDIENTE_DOWN_Pin GPIO_PIN_11
#define PENDIENTE_DOWN_GPIO_Port GPIOA
#define TIMMER_Pin GPIO_PIN_12
#define TIMMER_GPIO_Port GPIOA
#define VIENTO_Pin GPIO_PIN_15
#define VIENTO_GPIO_Port GPIOA
#define START_Pin GPIO_PIN_3
#define START_GPIO_Port GPIOB
#define STOP_Pin GPIO_PIN_4
#define STOP_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <Print.hpp>
#include "binary.h"
#include "itoa.h"
#include "utils.h"
#include "wiring_constants.h"

#ifdef __cplusplus
//#include "HardwareSerial.h"
//#include "Tone.h"
#include "WMath.h"
#include "WCharacter.h"
#include "WString.h"
#endif // __cplusplus

#define timmerStep 30
#define maxSpeed 160
#define maxTimmer 3600
#define minTimmer 300

typedef enum {
	OFF,
	ON
}powerState_e;

typedef enum{
	NO_INIT,
	START,
	STOP,
	FINISH
}runState_e;

typedef enum {
	M,
	P0,
	P1,
	P2,
	LAST_PROGRAM
}program_e;



typedef enum{
	A0,
	A15,
	A30,
	A45,
	A60
}angle_e;

typedef struct{
	//powerState_e power; No se necesita mas, siempre va estar prendida
	runState_e run;
	program_e programa;
	uint16_t velocidad;
	uint16_t timmer;
	angle_e inclinacion;
	float distancia;
}maquina_s;

/* USER CODE END Private defines */

void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

typedef enum {
	START_BOTON,
	PAUSA_BOTON,
	VEL_UP_BOTON,
	VEL_DOWN_BOTON,
	PROGRAMA_BOTON,
	PENDIENTE_UP_BOTON,
	PENDIENTE_DOWN_BOTON,
	TIMMER_BOTON,
	VIENTO_BOTON,
	STOP_BOTON,
	NONE_BOTON
}boton_e;
//Funcion que lee los botones

extern boton_e botonRead(void);

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
