/*
 * simple_parser.h
 *
 *  Created on: Nov 25, 2023
 *      Author: akun1
 */

#ifndef INC_MY_LIB_COMPLEX_PARSER_H_
#define INC_MY_LIB_COMPLEX_PARSER_H_

#include "main.h"

#include <string.h>
#include <stdbool.h>

#include "my_lib/L298N.h"
#include "my_lib/RingBuffer.h"
#include "my_lib/utils.h"
#include "my_lib/Robot.h"

#define END_LINE '\n'

void Parser_TakeLine(RingBuffer_t* Buff, uint8_t* Destination);
void Parser_Parse(Robot_t* Robot, uint8_t* DataToParse);

#endif /* INC_MY_LIB_COMPLEX_PARSER_H_ */
