/*
 * Robot.c
 *
 *  Created on: Nov 28, 2023
 *      Author: akun1
 */
#include "my_lib/Robot.h"

void Robot_Init(Robot_t* Robot, bool Enable, bool ReadDistanceEnable, bool Motor_Action_Flag, float HoldDistance, float ReadDistance_f)
{
	Robot->Enable = Enable;
	Robot->ReadDistanceEnable = ReadDistanceEnable;

	Robot->Motor_Action_Flag = Motor_Action_Flag;

	Robot->HoldDistance = HoldDistance;
	Robot->ReadDistance_f = ReadDistance_f;
}
