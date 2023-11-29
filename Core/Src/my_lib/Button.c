/*
 * Button.c
 *
 *  Created on: Nov 21, 2023
 *      Author: Sebastian Sosnowski
 */
#include "my_lib/Button.h"

/** Button_Init
 * @brief Button configuration initialization.
 *
 * @param key pointer to structure that contains Button configuration.
 * @param GPIO_Port pointer to button port.
 * @param GPIO_Pin button port bit.
 * @param TimerDebounce button time tick value for debounce.
 *
 * @retval None.
 * */
void Button_Init(Button_t* key, GPIO_TypeDef* GPIO_Port, uint16_t GPIO_Pin, uint32_t TimerDebounce)
{
	key->State = IDLE;

	key->Gpio_Port = GPIO_Port;
	key->Gpio_Pin = GPIO_Pin;

	key->TimerDebounce = TimerDebounce;
}

/** Button_IdleRoutine
 * @brief Button idle routine state i.e. when it's not pressed.
 * After press detection state is changed to debounce.
 *
 * @param key pointer to structure that contains Button configuration.
 *
 * @retval None.
 * */
static void Button_IdleRoutine(Button_t* key)
{
	if(HAL_GPIO_ReadPin(key->Gpio_Port, key->Gpio_Pin) == GPIO_PIN_RESET)
	{
		key->State = DEBOUNCE;
		key->LastTick = HAL_GetTick();
	}
}

/** Button_DebounceRoutine
 * @brief Button debounce routine state to determine
 * whether button was pressed or it were vibrations.
 * If yes, state is changed to pressed, otherwise goes back to idle.
 *
 * @param key pointer to structure that contains Button configuration.
 *
 * @retval None.
 * */
static void Button_DebounceRoutine(Button_t* key)
{
	if((HAL_GetTick() - key->LastTick) > key->TimerDebounce)
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

//static void Button_DebounceRoutine(Button_t* key)
//{
//	if((HAL_GetTick() - key->LastTick) < key->TimerDebounce)
//	{
//		return;
//	}
//
//	if(HAL_GPIO_ReadPin(key->Gpio_Port, key->Gpio_Pin) == GPIO_PIN_RESET)
//	{
//		key->State = PRESSED;
//	}
//	else
//	{
//		key->State = IDLE;
//	}
//}

/** Button_PressedRoutine
 * @brief Button pressed routine state to enable or disable robot operation.
 * LED on nucleo board is turned on when robot is enabled.
 *
 * @param key pointer to structure that contains Button configuration.
 * @param RobotEnable pointer to variable which determines if robot is enabled.
 *
 * @retval None.
 * */
static void Button_PressedRoutine(Button_t* key, bool* RobotEnable)
{
	if(HAL_GPIO_ReadPin(key->Gpio_Port, key->Gpio_Pin) == GPIO_PIN_SET)
	{
		key->State = IDLE;
		*RobotEnable = !(*RobotEnable);
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	}
}

/** Button_Task
 * @brief Main task where button states functions are executed.
 *
 * @param key pointer to structure that contains Button configuration.
 * @param RobotEnable pointer to variable which determines if robot is enabled.
 *
 * @retval None.
 * */
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
