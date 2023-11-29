/*
 * Robot.c
 *
 *  Created on: Nov 28, 2023
 *      Author: akun1
 */
#include "my_lib/Robot.h"

/** Robot_Init
 * @brief Robot configuration initialization.
 *
 * @param Robot pointer to structure that contains Robot configuration.
 * @param Enable informs that robot is enabled or disabled.
 * @param ReadDistanceEnable informs that new data from sensor was acquired and is ready to use.
 * @param MotorCommand informs that new command was sent via BT.
 * @param HoldDistance distance to maintain from the object in maintain distance mode.
 * @param ReadDistance_f distance value read by HCSR04p distance sensor.
 *
 * @retval None.
 * */
void Robot_Init(Robot_t* Robot, bool Enable, bool ReadDistanceEnable, bool MotorCommand, float HoldDistance, float ReadDistance_f)
{
	Robot->Enable = Enable;
	Robot->ReadDistanceEnable = ReadDistanceEnable;

	Robot->MotorCommand = MotorCommand;

	Robot->HoldDistance = HoldDistance;
	Robot->ReadDistance_f = ReadDistance_f;
}
