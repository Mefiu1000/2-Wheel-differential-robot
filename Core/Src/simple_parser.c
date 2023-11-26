/*
 * simple_parser.c
 *
 *  Created on: Nov 25, 2023
 *      Author: akun1
 */

#include "simple_parser.h"


void Parser_TakeLine(RingBuffer_t* Buff, uint8_t* Destination)
{
	  uint8_t i = 0;
	  uint8_t Tmp;

	  do
	  {
		  RB_Read(Buff, &Tmp);

		  if(Tmp == END_LINE)
		  {
			  Destination[i] = 0;
		  }
		  else
		  {
			  Destination[i] = Tmp;
		  }

		  i++;

	  }while(Tmp != END_LINE);
}

uint8_t Parser_Parse(uint8_t* DataToParse)
{
	uint8_t Command;

	  if(strcmp("F", (char*)DataToParse) == 0)
	  {
		 Command = MOVE_FORWARD;
	  }
	  else if(strcmp("B", (char*)DataToParse) == 0)
	  {
		  Command = MOVE_BACKWARD;
	  }
	  else if(strcmp("L", (char*)DataToParse) == 0)
	  {
		  Command = MOVE_LEFT;
	  }
	  else if(strcmp("R", (char*)DataToParse) == 0)
	  {
		  Command = MOVE_RIGHT;
	  }
	  else if(strcmp("S", (char*)DataToParse) == 0)
	  {
		  Command = STOP;
	  }
	  else if(strcmp("V", (char*)DataToParse[0]) == 0)
	  {
		  Command = CHANGE_SPEED;
	  }
	  else if(strcmp("H", (char*)DataToParse) == 0)
	  {
		  Command = HOLD_DISTANCE;
	  }
	  else
	  {
		  Command = 0;
	  }
	  return Command;
}
