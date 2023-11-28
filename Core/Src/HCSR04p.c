/*
 * HCSR04p.c
 *
 *  Created on: Nov 20, 2023
 *      Author: akun1
 */

#include "HCSR04p.h"

#define HCSR04p_FLOAT_CONST 0.01715 //  sound speed C  in cm / us

volatile uint16_t Time_uc;

void HCSR04p_Init(HCSR04p_t *hcsr04p,  TIM_HandleTypeDef *timer_trigger, uint32_t Trigger_TimChannel, TIM_HandleTypeDef *timer_echo,
		uint32_t Echo_TimChannel_Start, uint32_t Echo_TimChannel_Stop)
{
	hcsr04p->htim_trigger = timer_trigger;
	hcsr04p->htim_echo = timer_echo;

	hcsr04p->Trigger_TimChannel = Trigger_TimChannel;

	hcsr04p->Echo_TimChannel_Start = Echo_TimChannel_Start;
	hcsr04p->Echo_TimChannel_Stop = Echo_TimChannel_Stop;

	HAL_TIM_Base_Start(hcsr04p->htim_echo);
	HAL_TIM_IC_Start(hcsr04p->htim_echo, hcsr04p->Echo_TimChannel_Start);
	HAL_TIM_IC_Start_IT(hcsr04p->htim_echo, hcsr04p->Echo_TimChannel_Stop);

	HAL_TIM_Base_Start(hcsr04p->htim_trigger);
	HAL_TIM_PWM_Start(hcsr04p->htim_trigger, hcsr04p->Trigger_TimChannel);
}

void HCSR04p_ReadInteger(HCSR04p_t *hcsr04p, uint16_t *Read_distance)
{
	*Read_distance = hcsr04p->Result_us / 58; //in cm
}
void HCSR04p_ReadFloat(HCSR04p_t *hcsr04p, float *Read_distance)
{
	*Read_distance = (float)hcsr04p->Result_us * HCSR04p_FLOAT_CONST; //in cm
}
void HCSR04p_InteruptHandler(HCSR04p_t *hcsr04p)
{
	hcsr04p->Result_us = (uint16_t)hcsr04p->htim_echo->Instance->CCR2 - (uint16_t)hcsr04p->htim_echo->Instance->CCR1; //pulse width
	*ptrReadDist = true;
	//HAL_TIM_IC_Start_IT(htim, HCSR04p_STOP_CHANNEL);
}

