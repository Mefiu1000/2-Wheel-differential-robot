/*
 * Motors.c
 *
 *  Created on: Nov 21, 2023
 *      Author: Sebastian Sosnowski
 */
#include "my_lib/L298N.h"

//Set accordingly to project
#define MAX_SPEED 				999 //PWM max value, change accordingly to PWM timer
#define MIN_SPEED 				500 //PWM min value, bcs used motor are trash and this is the minimum speed for them to revolve
#define STARTUP_TIME 			5 // time needed to overcome static friction for (cheap)motors used in project
#define TIM_LEFTMOTOR 			TIM3
#define TIM_LEFTMOTOR_CHANNEL 	CCR1
#define TIM_RIGHTMOTOR 			TIM3
#define TIM_RIGHTMOTOR_CHANNEL 	CCR2
//lib constants
#define CHANGE_TO_FORWARD 1
#define CHANGE_TO_BACKWARD -1
#define DONT_CHANGE 0
#define STAY 0
#define ERROR 255

/** L298N_MotorInit
 * @brief Motor combined with L298N driver configuration initialization.
 *
 * @param motor pointer to structure that contains Motor configuration.
 * @param MotorForward_Port pointer to motor forward signal port.
 * @param MotorForward_Pin motor forward signal port bit.
 * @param MotorBackward_Port pointer to motor backward signal port.
 * @param MotorBackward_Pin motor backward signal port bit.
 *
 * @retval None.
 * */
void L298N_MotorInit(Motor_t* motor, GPIO_TypeDef* MotorForward_Port, uint16_t MotorForward_Pin,
		GPIO_TypeDef* MotorBackward_Port, uint16_t MotorBackward_Pin)
{
	motor->State = OPERATION;

	motor->MotorForward_Port = MotorForward_Port;
	motor->MotorForward_Pin = MotorForward_Pin;

	motor->MotorBackward_Port = MotorBackward_Port;
	motor->MotorBackward_Pin = MotorBackward_Pin;
}

/** L298N_MotorOperationRoutine
 * @brief Motor operation routine state i.e. when no command is handled.
 * If new command comes, state is changed to CHANGE_OPERATION
 * and last state is saved in motors configurations.
 *
 * @param MotorCommand pointer to variable which informs that new command was send via BT.
 * @param Leftmotor pointer to structure that contains left motor configuration.
 * @param Rightmotor pointer to structure that contains right motor configuration.
 *
 * @retval None.
 * */
static void L298N_MotorOperationRoutine(bool* MotorCommand, Motor_t* Leftmotor, Motor_t* Rightmotor)
{
	if(*MotorCommand == true)
	{
		Leftmotor->LastState = Leftmotor->State;
		Rightmotor->LastState = Leftmotor->State;

		Leftmotor->State = CHANGE_OPERATION;
		Rightmotor->State = CHANGE_OPERATION;
	}
}

/** Motor_CalculateSpeed
 * @brief Converts speed digits sent in char to integer value and concatenate them.
 *
 * @param num_100 hundredths digit.
 * @param num_10 tenths digit.
 * @param num_1 ones digit.
 *
 * @retval Speed value written to TIM_PWM modulation register.
 * */
static uint16_t Motor_CalculateSpeed(uint8_t num_100,uint8_t num_10, uint8_t num_1)
{
	uint16_t Speed;

	Speed = (num_1 - '0') + (num_10- '0') * 10 + (num_100- '0') * 100;

	return Speed;
}

/** L298N_MotorChangeOperationRoutine
 * @brief Motor change operation routine state i.e. when command was sent
 * and needs to be handled by proper functionality or other state.
 *
 * @param HC05_Command pointer to table that contains received command.
 * @param MotorCommand pointer to variable which informs that new command was sent via BT.
 * @param Leftmotor pointer to structure that contains left motor configuration.
 * @param Rightmotor pointer to structure that contains right motor configuration.
 * @param RX_Buff pointer to structure that contains receive ring buffer configuration.
 *
 * @retval None.
 * */
static void L298N_MotorChangeOperationRoutine(uint8_t* HC05_Command, bool* MotorCommand, Motor_t* Leftmotor, Motor_t* Rightmotor, RingBuffer_t* RX_Buff)
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
		RB_Flush(RX_Buff);
		break;
	}

	Leftmotor->State = State;
	Rightmotor->State = State;
	*MotorCommand = false;
}

/** CheckDirection
 * @brief Function that compare actual moving direction
 * with new direction based on new motor speed.
 *
 * @param prev_dir value with actual moving direction.
 * @param dir value with new moving direction.
 *
 * @retval Move direction change.
 * */
static int8_t CheckDirection(int8_t prev_dir, int8_t dir)
{
	int8_t result = dir - prev_dir;

	if(result == 0) return DONT_CHANGE; //same dir

	if(dir != STAY) //not stay
	{
		if(result > 0) return CHANGE_TO_FORWARD; // change direction to forward
		else if(result < 0) return CHANGE_TO_BACKWARD;	// change direction to backward
	}
	else
	{
		if(result > 0) return CHANGE_TO_BACKWARD;  // S - BWD = 1
		else if(result < 0) return CHANGE_TO_FORWARD; // S - FWD = -1
	}

	return ERROR; //error
}

/** L298N_MotorHoldDistanceRoutine
 * @brief Motor hold distance routine state i.e. robot tries
 * to maintain set distance from object in front of it. PID regulator is used to determine speed changes.
 * New speed and/or direction is applied only after sensor distance is updated.
 *
 * @param Robot pointer to structure that contains Robot configuration.
 * @param Leftmotor pointer to structure that contains left motor configuration.
 * @param Rightmotor pointer to structure that contains right motor configuration.
 * @param PID pointer to structure that contains PID regulator configuration.
 *
 * @retval None.
 * */
static void L298N_MotorHoldDistanceRoutine(Robot_t* Robot, Motor_t* Leftmotor, Motor_t* Rightmotor, PID_t* PID)
{
	if(Robot->MotorCommand == true)
	{
		Leftmotor->LastState = Leftmotor->State;
		Rightmotor->LastState = Leftmotor->State;

		Leftmotor->State = CHANGE_OPERATION;
		Rightmotor->State = CHANGE_OPERATION;
	}

	if(Robot->ReadDistanceEnable == false) return;

	float error;
	int16_t MotorSpeed;
	int8_t direction = DONT_CHANGE;
	error = Robot->ReadDistance_f - Robot->HoldDistance;
	PID->error_integral += error;
	PID->error_derivative = (PID->previous_error - error);
	PID->previous_error = error;

	MotorSpeed = round(PID->P * error + PID->I * PID->error_integral + PID->D * PID->error_derivative);

	if(MotorSpeed > 0) direction = CHANGE_TO_FORWARD;
	else if(MotorSpeed < 0) direction = CHANGE_TO_BACKWARD;

	MotorSpeed = abs(MotorSpeed); //set to positive value cuz its value for TIM->CCR

	if(MotorSpeed > MAX_SPEED) MotorSpeed = MAX_SPEED;
	else if(MotorSpeed < MIN_SPEED) MotorSpeed = MIN_SPEED; //set 500 bcs thats the minimal speed value for used motors to actually rev with load

	switch(CheckDirection(PID->previous_direction, direction))
	{
	case DONT_CHANGE:
		Motor_SetSpeed(MotorSpeed, MotorSpeed);
		break;
	case CHANGE_TO_FORWARD:
		Motor_SetSpeed(MotorSpeed, MotorSpeed);
		Move_Forward(Leftmotor, Rightmotor);
		break;
	case CHANGE_TO_BACKWARD:
		Motor_SetSpeed(MotorSpeed, MotorSpeed);
		Move_Backward(Leftmotor, Rightmotor);
		break;
	default:
		UartLog("Bug in hold distance function \n\r");
	}

	PID->previous_direction = direction;
	Robot->ReadDistanceEnable = false;
}

/** L298N_MotorTask
 * @brief Main task where motor states functions are executed.
 *
 * @param Robot pointer to structure that contains Robot configuration.
 * @param Leftmotor pointer to structure that contains left motor configuration.
 * @param Rightmotor pointer to structure that contains right motor configuration.
 * @param PID pointer to structure that contains PID regulator configuration.
 * @param RX_Buff pointer to structure that contains receive ring buffer configuration.
 *
 * @retval None.
 * */
void L298N_MotorTask(Robot_t* Robot, Motor_t* Leftmotor, Motor_t* Rightmotor, PID_t* PID, RingBuffer_t* RX_Buff)
{
	switch(Leftmotor->State)
	{
	case OPERATION:
		L298N_MotorOperationRoutine(&Robot->MotorCommand, Leftmotor, Rightmotor);
		break;
	case CHANGE_OPERATION:
		L298N_MotorChangeOperationRoutine(Robot->HC05_Command ,&Robot->MotorCommand,Leftmotor, Rightmotor, RX_Buff);
		break;
	case MAINTAIN_DISTANCE:
		L298N_MotorHoldDistanceRoutine(Robot, Leftmotor, Rightmotor, PID);
		break;
	}
}

/** Move_Forward
 * @brief Set motors to move forward with set speed.
 *
 * @param Leftmotor pointer to structure that contains left motor configuration.
 * @param Rightmotor pointer to structure that contains right motor configuration.
 *
 * @retval None.
 * */
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

/** Move_Backward
 * @brief Set motors to move backward with set speed.
 *
 * @param Leftmotor pointer to structure that contains left motor configuration.
 * @param Rightmotor pointer to structure that contains right motor configuration.
 *
 * @retval None.
 * */
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

/** Move_Left
 * @brief Set motors to turn left with set speed.
 *
 * @param Leftmotor pointer to structure that contains left motor configuration.
 * @param Rightmotor pointer to structure that contains right motor configuration.
 *
 * @retval None.
 * */
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

/** Move_Right
 * @brief Set motors to turn right with set speed.
 *
 * @param Leftmotor pointer to structure that contains left motor configuration.
 * @param Rightmotor pointer to structure that contains right motor configuration.
 *
 * @retval None.
 * */
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

/** Move_Stop
 * @brief Set motors to stop moving.
 *
 * @param Leftmotor pointer to structure that contains left motor configuration.
 * @param Rightmotor pointer to structure that contains right motor configuration.
 *
 * @retval None.
 * */
void Move_Stop(Motor_t* Leftmotor, Motor_t* Rightmotor)
{
	HAL_GPIO_WritePin(Leftmotor->MotorForward_Port, Leftmotor->MotorForward_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Leftmotor->MotorBackward_Port, Leftmotor->MotorBackward_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(Rightmotor->MotorForward_Port, Rightmotor->MotorForward_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Rightmotor->MotorBackward_Port, Rightmotor->MotorBackward_Pin, GPIO_PIN_RESET);
}

/** Motor_SetSpeed
 * @brief Set motor speed value to timers PWM modulation.
 *
 * @param left_speed value for left motor.
 * @param right_speed value for right motor.
 *
 * @retval None.
 * */
void Motor_SetSpeed(uint16_t left_speed, uint16_t right_speed)
{
	TIM_LEFTMOTOR->TIM_LEFTMOTOR_CHANNEL = left_speed;
	TIM_RIGHTMOTOR->TIM_RIGHTMOTOR_CHANNEL = right_speed;
}

/** Motor_Startup
 * @brief Read current speed and change it to 100% for startup.
 * Function needed because of cheap motors used in project ;(.
 *
 * @param left_speed pointer to variable with speed value for left motor.
 * @param right_speed pointer to variable with speed value for right motor.
 *
 * @retval None.
 * */
void Motor_Startup(uint16_t* left_speed, uint16_t* right_speed)
{
	//Read set speed
	*left_speed = TIM_LEFTMOTOR->TIM_LEFTMOTOR_CHANNEL;
	*right_speed = TIM_RIGHTMOTOR->TIM_RIGHTMOTOR_CHANNEL;
	//Change speed to 100% for startup to overcome static friction (cheap motors)
	TIM_LEFTMOTOR->TIM_LEFTMOTOR_CHANNEL = MAX_SPEED;
	TIM_RIGHTMOTOR->TIM_RIGHTMOTOR_CHANNEL = MAX_SPEED;
}

/** PID_Init
 * @brief Hold distance PID regulator configuration initialization.
 *
 * @param PID pointer to structure that contains PID regulator configuration.
 * @param P value of proportional coefficient.
 * @param I value of integral coefficient.
 * @param D value of derivative coefficient.
 *
 * @retval None.
 * */
void PID_Init(PID_t* PID, float P, float I, float D)
{
	PID->previous_direction = STAY;

	PID->previous_error = 0.0;
	PID->error_integral = 0.0;
	PID->error_derivative = 0.0;

	PID->P = P;  //#0.005
	PID->I = I; //#0.0005  0.0001
	PID->D = D; //# 0.0002
}
