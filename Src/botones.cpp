/*
 * botones.cpp
 *
 *  Created on: 24 oct. 2018
 *      Author: seba
 */
#include "main.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
extern uint16_t pinEntradas[];
extern GPIO_TypeDef* puertoEntradas[];
uint8_t suma = 0;
volatile uint8_t activated = 0;
volatile uint8_t previusActivated = 0;
volatile uint8_t step = 0;
GPIO_PinState pines[10]; //Buffer para guardar el estado de las entradas


boton_e botonRead(void) {

	//Bucle para guardar en el buffer el estado de las entradas
	for (uint8_t i = 0; i <= 9; i++) {
		pines[i] = HAL_GPIO_ReadPin(puertoEntradas[i], pinEntradas[i]);
	}
	suma = 0;
	//Bucle para sumar el estado de todos los pines
	for (uint8_t i = 0; i <= 9; i++) {
		suma += pines[i];
		//Guardamos que pin esta activado, en caso de que se encuentre otro, si la suma
		//da mas de 9, se descarta
		if (pines[i] == GPIO_PIN_RESET) {
			activated = i;
		}
	}

	//Si un solo pin esta activo la suma da 9, si es menor significa que se presionaron
	//mas botones, si da mas significa que no se presiono ningun boton
	if (suma == 9) {

		while (HAL_GPIO_ReadPin(puertoEntradas[activated],pinEntradas[activated]) == GPIO_PIN_RESET) {
			osDelay(10);
			break;
		}
		return (boton_e) activated;
	} else {
		return NONE_BOTON;
	}
}

detection_e encoderRead (detection_e *lastState){
	GPIO_PinState pinState = HAL_GPIO_ReadPin(encoder_incl_GPIO_Port, encoder_incl_Pin); //TODO This is a hardcode function
	if((*lastState == INACTIVE)&& (pinState == GPIO_PIN_SET)){ //El pin se activa con un bajo
		return INACTIVE;
	}

	if((*lastState == INACTIVE)&& (pinState == GPIO_PIN_RESET)){ //El pin se activa con un bajo
		*lastState = ACTIVE;
		return RISING;
	}

	if((*lastState == ACTIVE)&& (pinState == GPIO_PIN_RESET)){ //El pin se activa con un bajo
		return ACTIVE;
	}

	if((*lastState == ACTIVE)&& (pinState == GPIO_PIN_SET)){ //El pin se activa con un bajo
		*lastState = INACTIVE;
		return FALLING;
	}

	if((*lastState == INACTIVE)&& (pinState == GPIO_PIN_SET)){ //El pin se activa con un bajo
		return INACTIVE;
	}


 return FALLING;
}
