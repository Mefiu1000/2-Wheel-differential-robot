/*
 * Motors.c
 *
 *  Created on: Nov 21, 2023
 *      Author: akun1
 */
#include <L298N.h>


void L298N_MotorInit(Motor_t* motor, GPIO_TypeDef* MotorForward_Port, uint16_t MotorForward_Pin,
		GPIO_TypeDef* MotorBackward_Port, uint16_t MotorBackward_Pin)
{
	motor->State = OPERATION;

	motor->MotorForward_Port = MotorForward_Port;
	motor->MotorForward_Pin = MotorForward_Pin;

	motor->MotorBackward_Port = MotorBackward_Port;
	motor->MotorBackward_Pin = MotorBackward_Pin;
}

void L298N_MotorTask(Motor_t* Leftmotor, Motor_t* Rightmotor, PID_t* PID)
{
	switch(Leftmotor->State)
	{
	case OPERATION:
		L298N_MotorOperationRoutine(Leftmotor, Rightmotor);
		break;
	case CHANGE_OPERATION:
		L298N_MotorChangeOperationRoutine(Leftmotor, Rightmotor);
		break;
	case MAINTAIN_DISTANCE:
		L298N_MotorHoldDistanceRoutine(Leftmotor, Rightmotor, PID);
		break;
	}
}

void L298N_MotorOperationRoutine(Motor_t* Leftmotor, Motor_t* Rightmotor)
{
	if(*ptrMotor_Action_Flag == true)
	{
		Leftmotor->LastState = Leftmotor->State;
		Rightmotor->LastState = Leftmotor->State;

		Leftmotor->State = CHANGE_OPERATION;
		Rightmotor->State = CHANGE_OPERATION;
	}
}

void L298N_MotorChangeOperationRoutine(Motor_t* Leftmotor, Motor_t* Rightmotor)
{
	uint8_t State = OPERATION;

	switch(HC05_Command[0])
	{
	case MOVE_FORWARD:
		Move_Forward(Leftmotor, Rightmotor);
		break;
	case MOVE_BACKWARD:
		Move_Backward(Leftmotor, Rightmotor);
		break;
	case MOVE_LEFT:
		Move_Left(Leftmotor, Rightmotor);
		break;
	case MOVE_RIGHT:
		Move_Right(Leftmotor, Rightmotor);
		break;
	case STOP:
		Move_Stop(Leftmotor, Rightmotor);
		break;
	case CHANGE_SPEED:
		uint16_t Speed;
		Speed = Motor_CalculateSpeed(HC05_Command[1],HC05_Command[2], HC05_Command[3]);
		Motor_SetSpeed(Speed, Speed);
		State = Leftmotor->LastState;
		break;
	case HOLD_DISTANCE:
		State = MAINTAIN_DISTANCE;
		break;
	default:
		RB_Flush(&RX_Buffer);
		break;
	}

	Leftmotor->State = State;
	Rightmotor->State = State;
	*ptrMotor_Action_Flag = false;
}

uint16_t Motor_CalculateSpeed(uint8_t num_100,uint8_t num_10, uint8_t num_1)
{

	uint16_t Speed;
	Speed = (num_1 - '0') + (num_10- '0') * 10 + (num_100- '0') * 100;

	return Speed;
}


static int8_t CheckDirection(int8_t prev_dir, int8_t dir)
{
	int8_t result = dir - prev_dir;

	if(result == 0) return 0; //same dir

	if(dir != 0) //not stay
	{
		if(result > 0) return 1; // change direction to forward
		else if(result < 0) return -1;	// change direction to backward
	}
	else
	{
		if(result > 0) return -1;  // S - BWD = 1
		else if(result < 0) return 1; // S - FWD = -1

	}

	return -2; //error
	// FWD - BWD = 1 - (-1) = 2
	// FWD - S = 1 - 0 = 1
	// BWD - FWD = -1 - 1 = -2
	//BWD - S = -1 - 0 = -1
	// S - FWD = -1
}

void L298N_MotorHoldDistanceRoutine(Motor_t* Leftmotor, Motor_t* Rightmotor, PID_t* PID)
{
	if(*ptrMotor_Action_Flag == true)
	{
		Leftmotor->LastState = Leftmotor->State;
		Rightmotor->LastState = Leftmotor->State;

		Leftmotor->State = CHANGE_OPERATION;
		Rightmotor->State = CHANGE_OPERATION;
	}

	if(*ptrReadDist == false) return;

	float error;
	int16_t MotorSpeed;
	int8_t direction = 0;

	error = Distance_f - *ptrHoldDistance_value ;
	PID->error_integral += error;
	PID->error_derivative = (PID->previous_error - error);
	PID->previous_error = error;

	MotorSpeed = round(PID->P * error + PID->I * PID->error_integral + PID->D * PID->error_derivative);

	if(MotorSpeed > 0) direction = 1;
	else if(MotorSpeed < 0) direction = -1;

	MotorSpeed = abs(MotorSpeed); //set to positive value cuz its value for TIM->CCR

	if(MotorSpeed > 999) MotorSpeed = 999;
	else if(MotorSpeed < 500) MotorSpeed = 500; //set 500 bcs thats the minimal speed value for used motors to actually rev with load

	switch(CheckDirection(PID->previous_direction, direction))
	{
	case 0:
		Motor_SetSpeed(MotorSpeed, MotorSpeed);
		break;
	case 1:
		Motor_SetSpeed(MotorSpeed, MotorSpeed);
		Move_Forward(Leftmotor, Rightmotor);
		break;
	case -1:
		Motor_SetSpeed(MotorSpeed, MotorSpeed);
		Move_Backward(Leftmotor, Rightmotor);
		break;
	default:
		UartLog("Bug in hold distance function \n\r");
	}

	PID->previous_direction = direction;
	*ptrReadDist = false;
}

void Move_Forward(Motor_t* Leftmotor, Motor_t* Rightmotor)
{
	uint16_t left_speed, right_speed;

	//Read set speed and change it to 100% for startup
	Motor_Startup(&left_speed, &right_speed);

	HAL_GPIO_WritePin(Leftmotor->MotorForward_Port, Leftmotor->MotorForward_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Leftmotor->MotorBackward_Port, Leftmotor->MotorBackward_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(Rightmotor->MotorForward_Port, Rightmotor->MotorForward_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Rightmotor->MotorBackward_Port, Rightmotor->MotorBackward_Pin, GPIO_PIN_RESET);

	//time needed to overcome static friction before changing to set speed
	HAL_Delay(STARTUP_TIME);

	Motor_SetSpeed(left_speed, right_speed);
}

void Move_Backward(Motor_t* Leftmotor, Motor_t* Rightmotor)
{
	uint16_t left_speed, right_speed;

	//Read set speed and change it to 100% for startup
	Motor_Startup(&left_speed, &right_speed);

	HAL_GPIO_WritePin(Leftmotor->MotorForward_Port, Leftmotor->MotorForward_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Leftmotor->MotorBackward_Port, Leftmotor->MotorBackward_Pin, GPIO_PIN_SET);

	HAL_GPIO_WritePin(Rightmotor->MotorForward_Port, Rightmotor->MotorForward_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Rightmotor->MotorBackward_Port, Rightmotor->MotorBackward_Pin, GPIO_PIN_SET);

	//time needed to overcome static friction before changing to set speed
	HAL_Delay(STARTUP_TIME);

	Motor_SetSpeed(left_speed, right_speed);
}

void Move_Left(Motor_t* Leftmotor, Motor_t* Rightmotor)
{
	uint16_t left_speed, right_speed;

	//Read set speed and change it to 100% for startup
	Motor_Startup(&left_speed, &right_speed);

	HAL_GPIO_WritePin(Leftmotor->MotorForward_Port, Leftmotor->MotorForward_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Leftmotor->MotorBackward_Port, Leftmotor->MotorBackward_Pin, GPIO_PIN_SET);

	HAL_GPIO_WritePin(Rightmotor->MotorForward_Port, Rightmotor->MotorForward_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Rightmotor->MotorBackward_Port, Rightmotor->MotorBackward_Pin, GPIO_PIN_RESET);

	//time needed to overcome static friction before changing to set speed
	HAL_Delay(STARTUP_TIME);

	Motor_SetSpeed(left_speed, right_speed);

}

void Move_Right(Motor_t* Leftmotor, Motor_t* Rightmotor)
{
	uint16_t left_speed, right_speed;

	//Read set speed and change it to 100% for startup
	Motor_Startup(&left_speed, &right_speed);

	HAL_GPIO_WritePin(Leftmotor->MotorForward_Port, Leftmotor->MotorForward_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Leftmotor->MotorBackward_Port, Leftmotor->MotorBackward_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(Rightmotor->MotorForward_Port, Rightmotor->MotorForward_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Rightmotor->MotorBackward_Port, Rightmotor->MotorBackward_Pin, GPIO_PIN_SET);

	//time needed to overcome static friction before changing to set speed
	HAL_Delay(STARTUP_TIME);

	Motor_SetSpeed(left_speed, right_speed);
}

void Move_Stop(Motor_t* Leftmotor, Motor_t* Rightmotor)
{
	HAL_GPIO_WritePin(Leftmotor->MotorForward_Port, Leftmotor->MotorForward_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Leftmotor->MotorBackward_Port, Leftmotor->MotorBackward_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(Rightmotor->MotorForward_Port, Rightmotor->MotorForward_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Rightmotor->MotorBackward_Port, Rightmotor->MotorBackward_Pin, GPIO_PIN_RESET);
}

void Motor_SetSpeed(uint16_t left_speed, uint16_t right_speed)
{
	//write set speed
	TIM_LEFTMOTOR->TIM_LEFTMOTOR_CHANNEL = left_speed;
	TIM_RIGHTMOTOR->TIM_RIGHTMOTOR_CHANNEL = right_speed;
}

//Read set speed and change it to 100% for startup
void Motor_Startup(uint16_t* left_speed, uint16_t* right_speed)
{
	//Read set speed
	*left_speed = TIM_LEFTMOTOR->TIM_LEFTMOTOR_CHANNEL;
	*right_speed = TIM_RIGHTMOTOR->TIM_RIGHTMOTOR_CHANNEL;
	//Change speed to 100% for startup to overcome static friction (cheap motors)
	TIM_LEFTMOTOR->TIM_LEFTMOTOR_CHANNEL = FULL_SPEED;
	TIM_RIGHTMOTOR->TIM_RIGHTMOTOR_CHANNEL = FULL_SPEED;
}

void PID_Init(PID_t* PID, float P, float I, float D)
{
	PID->previous_direction = 0;

	PID->previous_error = 0.0;
	PID->error_integral = 0.0;
	PID->error_derivative = 0.0;

	PID->P = P;  //#0.005
	PID->I = I; //#0.0005  0.0001
	PID->D = D; //# 0.0002
}
