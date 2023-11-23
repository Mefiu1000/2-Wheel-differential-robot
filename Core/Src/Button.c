/*
 * Button.c
 *
 *  Created on: Nov 21, 2023
 *      Author: akun1
 */
#include "Button.h"

extern bool* ptrRobotEnable;
//Init

void Button_Init(Button_t* key, GPIO_TypeDef* Gpio_Port, uint16_t Gpio_Pin, uint32_t TimerDebounce)
{
	key->State = IDLE;

	key->Gpio_Port = Gpio_Port;
	key->Gpio_Pin = Gpio_Pin;

	key->TimerDebounce = TimerDebounce;
}

void Button_IdleRoutine(Button_t* key)
{
	if(HAL_GPIO_ReadPin(key->Gpio_Port, key->Gpio_Pin) == GPIO_PIN_RESET)
	{
		key->State = DEBOUNCE;
		key->LastTick = HAL_GetTick();
	}

}

void Button_DebounceRoutine(Button_t* key)
{
	if( (HAL_GetTick() - key->LastTick) > key->TimerDebounce)
	{

		if(HAL_GPIO_ReadPin(key->Gpio_Port, key->Gpio_Pin) == GPIO_PIN_RESET)
		{
			key->State = PRESSED;
//			if(key->ButtonPressed != NULL)
//			{
//				key->ButtonPressed();
//			}
		}
		else
		{
			key->State = IDLE;
		}
	}
}

void Button_PressedRoutine(Button_t* key)
{
	if(HAL_GPIO_ReadPin(key->Gpio_Port, key->Gpio_Pin) == GPIO_PIN_SET)
	{
		key->State = IDLE;
		*ptrRobotEnable = !(*ptrRobotEnable);

	}
}
void Button_Task(Button_t* key)
{
	switch(key->State)
	{
	case IDLE:
		Button_IdleRoutine(key);
		break;
	case DEBOUNCE:
		Button_DebounceRoutine(key);
		break;
	case PRESSED:
		Button_PressedRoutine(key);
		break;
	}
}
