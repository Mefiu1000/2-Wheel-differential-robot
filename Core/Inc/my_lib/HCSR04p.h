/*
 * HCSR04p.h
 *
 *  Created on: Nov 20, 2023
 *      Author: akun1
 */

#ifndef INC_MY_LIB_HCSR04P_H_
#define INC_MY_LIB_HCSR04P_H_

#include "main.h"
#include <stdbool.h>

typedef struct
{
	TIM_HandleTypeDef *htim_trigger;
	TIM_HandleTypeDef *htim_echo;

	uint32_t Trigger_TimChannel;

	uint32_t Echo_TimChannel_Start;
	uint32_t Echo_TimChannel_Stop;

	uint16_t Result_us;

}HCSR04p_t;

void HCSR04p_Init(HCSR04p_t *hcsr04p,  TIM_HandleTypeDef *timer_trigger, uint32_t Trigger_TimChannel, TIM_HandleTypeDef *timer_echo,
		uint32_t Echo_TimChannel_Start, uint32_t Echo_TimChannel_Stop);
void HCSR04p_ReadInteger(HCSR04p_t *hcsr04p, uint16_t *Read_distance);
void HCSR04p_ReadFloat(HCSR04p_t *hcsr04p, float *Read_distance);
void HCSR04p_InteruptHandler(HCSR04p_t *hcsr04p, bool* ReadDistanceEnable);
void HCSR04p_Read(uint16_t *Read_distance);

#endif /* INC_MY_LIB_HCSR04P_H_ */
