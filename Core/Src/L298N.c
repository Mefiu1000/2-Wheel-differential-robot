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

void L298N_MotorTask(Motor_t* Leftmotor, Motor_t* Rightmotor)
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
		L298N_MotorHoldDistanceRoutine(Leftmotor, Rightmotor);
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
		//Wrong_Data();
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

void L298N_MotorHoldDistanceRoutine(Motor_t* Leftmotor, Motor_t* Rightmotor)
{
	if(*ptrMotor_Action_Flag == true)
	{
		Leftmotor->LastState = Leftmotor->State;
		Rightmotor->LastState = Leftmotor->State;

		Leftmotor->State = CHANGE_OPERATION;
		Rightmotor->State = CHANGE_OPERATION;
	}
	//Hold_Distance(&Distance_f);
	  if(Distance_f > (*ptrHoldDistance_value + 1.0))
	  {
		  Move_Forward(Leftmotor, Rightmotor);
	  }
	  else if(Distance_f < (*ptrHoldDistance_value - 3.0))
	  {
		 Move_Backward(Leftmotor, Rightmotor);
	  }
	  else
	  {
		  Move_Stop(Leftmotor, Rightmotor);
	  }
}

//void Wrong_Data()
//{
//	uint8_t msg[28], Length;
//
//	Length = sprintf((char*)msg, "Wrong command, try again\n\r");
//	HAL_UART_Transmit(&huart1, msg, Length, 2000);
//}

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
