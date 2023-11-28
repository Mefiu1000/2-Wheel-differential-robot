/*
 * Button.c
 *
 *  Created on: Nov 21, 2023
 *      Author: akun1
 */
#include "my_lib/Button.h"

void Button_Init(Button_t* key, GPIO_TypeDef* Gpio_Port, uint16_t Gpio_Pin, uint32_t TimerDebounce)
{
	key->State = IDLE;

	key->Gpio_Port = Gpio_Port;
	key->Gpio_Pin = Gpio_Pin;

	key->TimerDebounce = TimerDebounce;
}

static void Button_IdleRoutine(Button_t* key)
{
	if(HAL_GPIO_ReadPin(key->Gpio_Port, key->Gpio_Pin) == GPIO_PIN_RESET)
	{
		key->State = DEBOUNCE;
		key->LastTick = HAL_GetTick();
	}
}

static void Button_DebounceRoutine(Button_t* key)
{
	if( (HAL_GetTick() - key->LastTick) > key->TimerDebounce)
	{
		if(HAL_GPIO_ReadPin(key->Gpio_Port, key->Gpio_Pin) == GPIO_PIN_RESET)
		{
			key->State = PRESSED;
		}
		else
		{
			key->State = IDLE;
		}
	}
}

static void Button_PressedRoutine(Button_t* key, bool* RobotEnable)
{
	if(HAL_GPIO_ReadPin(key->Gpio_Port, key->Gpio_Pin) == GPIO_PIN_SET)
	{
		key->State = IDLE;
		*RobotEnable = !(*RobotEnable);
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	}
}
void Button_Task(Button_t* key, bool* RobotEnable)
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
		Button_PressedRoutine(key, RobotEnable);
		break;
	}
}
