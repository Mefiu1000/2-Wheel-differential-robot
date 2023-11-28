/*
 * utils.c
 *
 *  Created on: Nov 26, 2023
 *      Author: akun1
 */
#include "my_lib/utils.h"

void UartLog(char* Message)
{
	HAL_UART_Transmit(&huart1, (uint8_t*)Message, strlen(Message), 1000);
}


