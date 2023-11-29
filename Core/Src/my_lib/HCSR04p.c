/*
 * HCSR04p.c
 *
 *  Created on: Nov 20, 2023
 *      Author: akun1
 */

#include "my_lib/HCSR04p.h"

#define HCSR04p_FLOAT_CONST 0.01715 //  sound speed C  in cm / us

/** HCSR04p_Init
 * @brief HCSR04p distance sensor configuration initialization.
 *
 * @param hcsr04p pointer to structure that contains HCSR04p configuration.
 * @param timer_trigger pointer to measurement trigger timer base handle.
 * @param Trigger_TimChannel measurement trigger timer channel.
 * @param timer_echo pointer to measurement echo timer base handle.
 * @param Echo_TimChannel_Start measurement echo result beginning detection channel.
 * @param Echo_TimChannel_Stop measurement echo result end detection channel.
 *
 * @retval None.
 * */
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

/** HCSR04p_ReadInteger
 * @brief Reads and saves distance value from sensor as integer.
 *
 * @param hcsr04p pointer to structure that contains HCSR04p configuration.
 * @param ReadDistance pointer to variable that contains sensor read distance value
 *
 * @retval None.
 * */
void HCSR04p_ReadInteger(HCSR04p_t* hcsr04p, uint16_t* ReadDistance)
{
	*ReadDistance = hcsr04p->Result_us / 58; //in cm
}

/** HCSR04p_ReadFloat
 * @brief Reads and saves distance value from sensor as float.
 *
 * @param hcsr04p pointer to structure that contains HCSR04p configuration.
 * @param ReadDistance pointer to variable that contains sensor read distance value.
 *
 * @retval None.
 * */
void HCSR04p_ReadFloat(HCSR04p_t* hcsr04p, float *ReadDistance)
{
	*ReadDistance = (float)hcsr04p->Result_us * HCSR04p_FLOAT_CONST; //in cm
}

/** HCSR04p_InteruptHandler
 * @brief Handles sensor interrupt by saving pulse width value.
 *
 * @param hcsr04p pointer to structure that contains HCSR04p configuration.
 * @param ReadDistanceEnable pointer to variable which informs that new data from sensor was acquired and is ready to use.
 *
 * @retval None.
 * */
void HCSR04p_InteruptHandler(HCSR04p_t* hcsr04p, bool* ReadDistanceEnable)
{
	hcsr04p->Result_us = (uint16_t)hcsr04p->htim_echo->Instance->CCR2 - (uint16_t)hcsr04p->htim_echo->Instance->CCR1; //pulse width
	*ReadDistanceEnable = true;
}

