/*
 * simple_parser.h
 *
 *  Created on: Nov 25, 2023
 *      Author: akun1
 */

#ifndef INC_COMPLEX_PARSER_H_
#define INC_COMPLEX_PARSER_H_

#include "main.h"
#include "RingBuffer.h"
#include "L298N.h"
#include "string.h"
#include "utils.h"


#define END_LINE '\n'

extern uint8_t HC05_Command[RING_BUFFER_SIZE];
extern float* ptrHoldDistance_value;
extern bool* ptrRobotEnable;
extern bool* ptrMotor_Action_Flag;


void Parser_TakeLine(RingBuffer_t* Buff, uint8_t* Destination);
void Parser_Parse(uint8_t* DataToParse);

#endif /* INC_COMPLEX_PARSER_H_ */
