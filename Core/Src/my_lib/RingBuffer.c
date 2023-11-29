/*
 * RingBuffer.c
 *
 *  Created on: Nov 23, 2023
 *      Author: Sebastian Sosnowski
 */


#include "my_lib/RingBuffer.h"

/** RB_Write
 * @brief Write data received from e.g. UART to ring buffer table.
 *
 * @param Buff pointer to structure that contains ring buffer configuration.
 * @param ReceivedValue data to write to ring buffer.
 *
 * @retval RB_Status returns writing result.
 * */
RB_Status RB_Write(RingBuffer_t* Buff, uint8_t ReceivedValue)
{
	uint8_t HeadTmp = (Buff->Head + 1) % (RING_BUFFER_SIZE); //modulo to avoid accessing element outside of table

	if(HeadTmp == Buff->Tail)
	{
		return RB_ERROR;
	}

	Buff->Buffer[Buff->Head] = ReceivedValue;
	Buff->Head = HeadTmp;

	return RB_OK;
}

/** RB_Read
 * @brief Read data saved in ring buffer table.
 *
 * @param ReadBuff pointer to structure that contains ring buffer configuration.
 * @param WriteToValue pointer to table to write data from Ring buffer.
 *
 * @retval RB_Status returns reading result.
 * */
RB_Status RB_Read(RingBuffer_t* ReadBuff, uint8_t* WriteToValue)
{
	if(ReadBuff->Head == ReadBuff->Tail)
	{
		return RB_ERROR;
	}

	*WriteToValue = ReadBuff->Buffer[ReadBuff->Tail];
	ReadBuff->Tail = (ReadBuff->Tail + 1) % RING_BUFFER_SIZE; //modulo to avoid accessing element outside of table

	return RB_OK;
}

/** RB_Flush
 * @brief Flush data contained in ring buffer.
 *
 * @param Buff pointer to structure that contains ring buffer configuration.
 *
 * @retval None.
 * */
void RB_Flush(RingBuffer_t* Buff)
{
	Buff->Head = 0;
	Buff->Tail = 0;
}
