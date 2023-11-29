/*
 * utils.c
 *
 *  Created on: Nov 26, 2023
 *      Author: Sebastian Sosnowski
 */
#include "my_lib/utils.h"

/** UartLog
 * @brief Sends inputed text via UART.
 *
 * @param Message pointer to text message.
 *
 * @retval None.
 * */
void UartLog(char* Message)
{
	HAL_UART_Transmit(&huart1, (uint8_t*)Message, strlen(Message), 1000);
}


