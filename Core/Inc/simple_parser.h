/*
 * simple_parser.h
 *
 *  Created on: Nov 25, 2023
 *      Author: akun1
 */

#ifndef INC_SIMPLE_PARSER_H_
#define INC_SIMPLE_PARSER_H_

#include "main.h"
#include "RingBuffer.h"
#include "L298N.h"
#include "string.h"

#define END_LINE '\n'

void Parser_TakeLine(RingBuffer_t* Buff, uint8_t* Destination);
uint8_t Parser_Parse(uint8_t* DataToParse);

#endif /* INC_SIMPLE_PARSER_H_ */
