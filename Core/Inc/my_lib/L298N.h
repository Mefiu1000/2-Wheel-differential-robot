/*
 * Motors.h
 *
 *  Created on: Nov 21, 2023
 *      Author: akun1
 */

#ifndef INC_MOTORS_H_
#define INC_MOTORS_H_

#include "main.h"
#include "usart.h"

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "my_lib/RingBuffer.h"
#include "my_lib/utils.h"
#include "my_lib/Robot.h"

//commands for HC05 COMMUNICATION via USART
#define MOVE_FORWARD 	0x46 //F
#define MOVE_BACKWARD 	0x42 //B
#define MOVE_LEFT 		0x4C //L
#define MOVE_RIGHT 		0x52 //R
#define STOP 			0x53 //S
#define CHANGE_SPEED 	0x56 //V
#define HOLD_DISTANCE 	0x48 //H
#define WRONG_DATA 		0x00


typedef enum
{
	OPERATION = 0,
	CHANGE_OPERATION,
	MAINTAIN_DISTANCE
}MOTOR_STATE;

typedef struct
{
	MOTOR_STATE 	State;
	MOTOR_STATE		LastState;

	GPIO_TypeDef*	MotorForward_Port;
	uint16_t 		MotorForward_Pin;

	GPIO_TypeDef*	MotorBackward_Port;
	uint16_t 		MotorBackward_Pin;
}Motor_t;

typedef struct
{
	int8_t previous_direction;//for L298N driver to change motor rev direction

	float previous_error;
	float error_integral;
	float error_derivative;

	float P;
	float I;
	float D;

}PID_t;


void L298N_MotorInit(Motor_t* motor, GPIO_TypeDef* MotorLeftForward_Port, uint16_t MotorLeftForward_Pin,
		GPIO_TypeDef* MotorLeftBackward_Port, uint16_t MotorLeftBackward_Pin);
void L298N_MotorTask(Robot_t* Robot, Motor_t* Leftmotor, Motor_t* Rightmotor, PID_t* PID, RingBuffer_t* RX_Buff);
void Move_Forward(Motor_t* Leftmotor, Motor_t* Rightmotor);
void Move_Backward(Motor_t* Leftmotor, Motor_t* Rightmotor);
void Move_Left(Motor_t* Leftmotor, Motor_t* Rightmotor);
void Move_Right(Motor_t* Leftmotor, Motor_t* Rightmotor);
void Move_Stop(Motor_t* Leftmotor, Motor_t* Rightmotor);
void Motor_SetSpeed(uint16_t left_speed, uint16_t right_speed);
void Motor_Startup(uint16_t* left_speed, uint16_t* right_speed);
//uint16_t Motor_CalculateSpeed(uint8_t num_100,uint8_t num_10, uint8_t num_1);
void PID_Init(PID_t* PID, float P, float I, float D);


#endif /* INC_MOTORS_H_ */
