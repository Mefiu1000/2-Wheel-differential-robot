/*
 * RingBuffer.h
 *
 *  Created on: Nov 23, 2023
 *      Author: akun1
 */

#ifndef INC_MY_LIB_RINGBUFFER_H_
#define INC_MY_LIB_RINGBUFFER_H_

#include "main.h"

#define RING_BUFFER_SIZE 32

typedef enum
{
	RB_OK = 0,
	RB_ERROR
}RB_Status;


typedef struct
{
	uint8_t Buffer[RING_BUFFER_SIZE];
	uint16_t Head;
	uint16_t Tail;
}RingBuffer_t;


RB_Status RB_Write(RingBuffer_t* Buff, uint8_t Value);
RB_Status RB_Read(RingBuffer_t* Buff, uint8_t* Value);
void RB_Flush(RingBuffer_t* Buff);


#endif /* INC_MY_LIB_RINGBUFFER_H_ */
