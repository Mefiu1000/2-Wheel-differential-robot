/*
 * Button.h
 *
 *  Created on: Nov 21, 2023
 *      Author: akun1
 */

#ifndef INC_BUTTON_H_
#define INC_BUTTON_H_

#include "main.h"
#include <stdbool.h>
typedef enum
{
	IDLE = 0,
	DEBOUNCE,
	PRESSED
}BUTTON_STATE;

typedef struct
{
	BUTTON_STATE 	State;
	GPIO_TypeDef*	Gpio_Port;
	uint16_t 		Gpio_Pin;

	uint32_t 		TimerDebounce; //debounce time
	uint32_t		LastTick;

	void(*ButtonPressed)(void);

}Button_t;

void Button_Init(Button_t* key, GPIO_TypeDef* Gpio_Port, uint16_t Gpio_Pin, uint32_t TimerDebounce);
void Button_Task(Button_t* key);

#endif /* INC_BUTTON_H_ */
