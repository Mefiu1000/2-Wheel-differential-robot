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
#include "stdlib.h"
#include "HCSR04p.h"
#include "RingBuffer.h"

//Set accordingly to project
#define FULL_SPEED 				999 //PWM max value, change accordingly to PWM timer
#define STARTUP_TIME 			5 // time needed to overcome static friction for (cheap)motors used in project
#define TIM_LEFTMOTOR 			TIM3
#define TIM_LEFTMOTOR_CHANNEL 	CCR1
#define TIM_RIGHTMOTOR 			TIM3
#define TIM_RIGHTMOTOR_CHANNEL 	CCR2

//commands for HC05 COMMUNICATION via USART
#define MOVE_FORWARD 	0x46 //F
#define MOVE_BACKWARD 	0x42 //B
#define MOVE_LEFT 		0x4C //L
#define MOVE_RIGHT 		0x52 //R
#define STOP 			0x53 //S
#define CHANGE_SPEED 	0x56 //V
#define HOLD_DISTANCE 	0x48 //H
#define WRONG_DATA 		0x00


extern bool *ptrMotor_Action_Flag;
extern uint8_t HC05_Command[RING_BUFFER_SIZE];
extern RingBuffer_t RX_Buffer;
extern float Distance_f;
extern float* ptrHoldDistance_value;

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


void L298N_MotorInit(Motor_t* motor, GPIO_TypeDef* MotorLeftForward_Port, uint16_t MotorLeftForward_Pin,
		GPIO_TypeDef* MotorLeftBackward_Port, uint16_t MotorLeftBackward_Pin);
void L298N_MotorTask(Motor_t* Leftmotor, Motor_t* Rightmotor);
void L298N_MotorOperationRoutine(Motor_t* Leftmotor, Motor_t* Rightmotor);
void L298N_MotorChangeOperationRoutine(Motor_t* Leftmotor, Motor_t* Rightmotor);
void L298N_MotorHoldDistanceRoutine(Motor_t* Leftmotor, Motor_t* Rightmotor);
//void Wrong_Data();
void Move_Forward(Motor_t* Leftmotor, Motor_t* Rightmotor);
void Move_Backward(Motor_t* Leftmotor, Motor_t* Rightmotor);
void Move_Left(Motor_t* Leftmotor, Motor_t* Rightmotor);
void Move_Right(Motor_t* Leftmotor, Motor_t* Rightmotor);
void Move_Stop(Motor_t* Leftmotor, Motor_t* Rightmotor);
void Motor_SetSpeed(uint16_t left_speed, uint16_t right_speed);
void Motor_Startup(uint16_t* left_speed, uint16_t* right_speed);
uint16_t Motor_CalculateSpeed(uint8_t num_100,uint8_t num_10, uint8_t num_1);

#endif /* INC_MOTORS_H_ */
