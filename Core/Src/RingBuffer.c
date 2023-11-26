/*
 * RingBuffer.c
 *
 *  Created on: Nov 23, 2023
 *      Author: akun1
 */


#include "RingBuffer.h"

RB_Status RB_Write(RingBuffer_t* Buff, uint8_t Value)
{
	uint8_t HeadTmp = (Buff->Head + 1) % (RING_BUFFER_SIZE); //modulo to avoid accessing element outside of table

	if(HeadTmp == Buff->Tail)
	{
		return RB_ERROR;
	}

	Buff->Buffer[Buff->Head] = Value;
	Buff->Head = HeadTmp;

	return RB_OK;
}

RB_Status RB_Read(RingBuffer_t* Buff, uint8_t* Value)
{
	if(Buff->Head == Buff->Tail)
	{
		return RB_ERROR;
	}

	*Value = Buff->Buffer[Buff->Tail];
	Buff->Tail = (Buff->Tail + 1) % RING_BUFFER_SIZE; //modulo to avoid accessing element outside of table

	return RB_OK;
}

void RB_Flush(RingBuffer_t* Buff)
{
	Buff->Head = 0;
	Buff->Tail = 0;
}
