/*
 * Robot.h
 *
 *  Created on: Nov 28, 2023
 *      Author: akun1
 */

#ifndef INC_MY_LIB_ROBOT_H_
#define INC_MY_LIB_ROBOT_H_

#include "main.h"

#include <stdbool.h>

#include "my_lib/RingBuffer.h"

typedef struct
{
	bool Enable;
	bool ReadDistanceEnable;
	bool MotorCommand;

	float HoldDistance;
	float ReadDistance_f;

	uint8_t HC05_Command[RING_BUFFER_SIZE];
}Robot_t;

void Robot_Init(Robot_t* Robot, bool Enable, bool ReadDistanceEnable, bool MotorCommand, float HoldDistance, float ReadDistance_f);

#endif /* INC_MY_LIB_ROBOT_H_ */
