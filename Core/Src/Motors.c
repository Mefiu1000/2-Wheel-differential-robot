/*
 * Motors.c
 *
 *  Created on: Nov 21, 2023
 *      Author: akun1
 */
#include "Motors.h"
void Move_Forward()
{
	HAL_GPIO_WritePin(LeftMotor_FWD_GPIO_Port, LeftMotor_FWD_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LeftMotor_BWD_GPIO_Port, LeftMotor_BWD_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(RightMotor_FWD_GPIO_Port, RightMotor_FWD_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(RightMotor_BWD_GPIO_Port, RightMotor_BWD_Pin, GPIO_PIN_RESET);
}

void Move_Backward()
{
	HAL_GPIO_WritePin(LeftMotor_FWD_GPIO_Port, LeftMotor_FWD_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LeftMotor_BWD_GPIO_Port, LeftMotor_BWD_Pin, GPIO_PIN_SET);

	HAL_GPIO_WritePin(RightMotor_FWD_GPIO_Port, RightMotor_FWD_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RightMotor_BWD_GPIO_Port, RightMotor_BWD_Pin, GPIO_PIN_SET);
}

void Move_Left()
{
	HAL_GPIO_WritePin(LeftMotor_FWD_GPIO_Port, LeftMotor_FWD_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LeftMotor_BWD_GPIO_Port, LeftMotor_BWD_Pin, GPIO_PIN_SET);

	HAL_GPIO_WritePin(RightMotor_FWD_GPIO_Port, RightMotor_FWD_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(RightMotor_BWD_GPIO_Port, RightMotor_BWD_Pin, GPIO_PIN_RESET);
}

void Move_Right()
{
	HAL_GPIO_WritePin(LeftMotor_FWD_GPIO_Port, LeftMotor_FWD_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LeftMotor_BWD_GPIO_Port, LeftMotor_BWD_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(RightMotor_FWD_GPIO_Port, RightMotor_FWD_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RightMotor_BWD_GPIO_Port, RightMotor_BWD_Pin, GPIO_PIN_SET);
}

void Stay()
{
	HAL_GPIO_WritePin(LeftMotor_FWD_GPIO_Port, LeftMotor_FWD_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LeftMotor_BWD_GPIO_Port, LeftMotor_BWD_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(RightMotor_FWD_GPIO_Port, RightMotor_FWD_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RightMotor_BWD_GPIO_Port, RightMotor_BWD_Pin, GPIO_PIN_RESET);
}
