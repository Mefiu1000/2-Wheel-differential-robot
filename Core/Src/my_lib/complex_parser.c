/*
 * simple_parser.c
 *
 *  Created on: Nov 25, 2023
 *      Author: akun1
 */

#include "my_lib/complex_parser.h"


/** Parser_TakeLine
 * @brief This function writes data from receive buffer to destined table
 * until it detects new line char i.e. writes one line to destined table.
 *
 * @param RX_Buff pointer to structure that contains receive ring buffer configuration.
 * @param Destination pointer to table to store data from buffer.
 *
 * @retval None.
 * */
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

/** Parser_ParseMOVE
 * @brief Parses MOVE command value and writes according command to destined table.
 * If wrong data is sent, error log is transmitted.
 *
 * @param MotorCommand pointer to variable which informs that new command was sent via BT.
 * @param HC05_Command pointer to table that contains received command.
 *
 * @retval None.
 * */
static void Parser_ParseMOVE(bool* MotorCommand, uint8_t* HC05_Command)
{
	uint8_t Command;

	char *ParsePointer = strtok(NULL, ",");

	if(strlen(ParsePointer) > 0) // not null
	{
		if(ParsePointer[0] == 'F')
		{
			Command = MOVE_FORWARD;
		}
		else if(ParsePointer[0] == 'B')
		{
			Command = MOVE_BACKWARD;
		}
		else if(ParsePointer[0] == 'L')
		{
			Command = MOVE_LEFT;
		}
		else if(ParsePointer[0] == 'R')
		{
			Command = MOVE_RIGHT;
		}
		else if(ParsePointer[0] == 'S')
		{
			Command = STOP;
		}
		else
		{
			UartLog("Wrong movement command. Available: F, B, L, R or S\r\n");
			Command = WRONG_DATA;
		}
	}
	else
	{
		Command = WRONG_DATA;
	}

	*MotorCommand = true;
	HC05_Command[0] = Command;
}

/** Parser_ParseSPEED
 * @brief Parses SPEED command value and writes it to destined table.
 * If wrong data is sent, error log is transmitted.
 *
 * @param MotorCommand pointer to variable which informs that new command was sent via BT.
 * @param HC05_Command pointer to table that contains received command.
 *
 * @retval None.
 * */
static void Parser_ParseSPEED(bool* MotorCommand, uint8_t* HC05_Command)
{
	uint8_t Command, len;

	char *ParsePointer = strtok(NULL, ",");

	len = strlen(ParsePointer);

	if(len > 0) // not null
	{
		for(uint8_t i = 0; ParsePointer[i] != 0; i++) //strok puts 0 when the string ends
		{
			if(ParsePointer[i] < '0' || ParsePointer[i] > '9')
			{
				UartLog("Wrong distance value. Type numerical value e.g. 1.0, 2.34\r\n");
				*MotorCommand = true;
				HC05_Command[0] = WRONG_DATA;
				return;
			}
		}
		if(len > 3) //speed not between 0-999
		{
			UartLog("Wrong speed value. Type value between 0-999\r\n");
			Command = WRONG_DATA;
		}
		else
		{
			if(len == 1)
			{
				HC05_Command[1] = '0';
				HC05_Command[2] = '0';
				HC05_Command[3] = ParsePointer[0];
			}
			else if(len == 2)
			{
				HC05_Command[1] = '0';
				HC05_Command[2] = ParsePointer[0];
				HC05_Command[3] = ParsePointer[1];
			}
			else
			{
				HC05_Command[1] = ParsePointer[0];
				HC05_Command[2] = ParsePointer[1];
				HC05_Command[3] = ParsePointer[2];
			}
			Command = CHANGE_SPEED;
		}
	}
	else
	{
		UartLog("Wrong speed value. Type value between 0-999\r\n");
		Command = WRONG_DATA;
	}

	*MotorCommand = true;
	HC05_Command[0] = Command;
}

/** Parser_ParseHOLD
 * @brief Parses HOLD command value. Writes command to destined table and set hold distance to destined variable.
 * If wrong data is sent, error log is transmitted.
 *
 * @param MotorCommand pointer to variable which informs that new command was sent via BT.
 * @param HoldDistance_value distance to maintain from the object in maintain distance mode.
 * @param HC05_Command pointer to table that contains received command.
 *
 * @retval None.
 * */
static void Parser_ParseHOLD(bool* MotorCommand, float* HoldDistance_value, uint8_t* HC05_Command)
{
	uint8_t Command, len, i;
	char *ParsePointer = strtok(NULL, ",");

	len = strlen(ParsePointer);

	if(len > 0)
	{
		for(i = 0; ParsePointer[i] != 0; i++) //strok puts 0 when the string ends
		{
			if((ParsePointer[i] < '0' || ParsePointer[i] > '9') && ParsePointer[i] != '.')
			{
				UartLog("Wrong distance value. Type numerical value e.g. 1.0, 2.34\r\n");
				*MotorCommand = true;
				HC05_Command[0] = WRONG_DATA;
				return;
			}
		}
		Command = HOLD_DISTANCE;
		*HoldDistance_value = atof(ParsePointer);
	}
	else
	{
		UartLog("Wrong distance value. Type numerical value e.g. 1.0, 2.34\r\n");
		Command = WRONG_DATA;
	}

	*MotorCommand = true;
	HC05_Command[0] = Command;
}

/** Parser_ParseENABLE
 * @brief Parses ENABLE command value and enable or disable robot operation.
 * If wrong data is sent, error log is transmitted.
 *
 * @param RobotEnable informs that robot is enabled or disabled.
 *
 * @retval None.
 * */
static void Parser_ParseENABLE(bool* RobotEnable)
{
	uint8_t len;

	char *ParsePointer = strtok(NULL, ",");

	len = strlen(ParsePointer);

	if(len > 0) // not null
	{
		if(ParsePointer[0] < '0' || ParsePointer[0] > '1')
		{
			UartLog("Wrong value. Type 1 to enable robot or 0 to disable.\r\n");
			return;
		}

		if(ParsePointer[0] == '1')
		{
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
			*RobotEnable = true;
		}
		else if(ParsePointer[0] == '0')
		{
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
			*RobotEnable = false;
		}
	}
	else
	{
		UartLog("Wrong value. Type 1 to enable robot or 0 to disable.\r\n");
	}
}

/** Parser_Parse
 * @brief Parses data to determine command type and execute according parse function.
 * If wrong data is sent, error log is transmitted.
 *
 * @param Robot pointer to structure that contains Robot configuration.
 * @param DataToParse pointer to data to parse.
 *
 * @retval None.
 * */
void Parser_Parse(Robot_t* Robot, uint8_t* DataToParse)
{
	char *ParsePointer = strtok((char*)DataToParse, "=");

	  if(strcmp("MOVE", ParsePointer) == 0)
	  {
		  Parser_ParseMOVE(&Robot->MotorCommand, Robot->HC05_Command);
	  }
	  else if(strcmp("SPEED", ParsePointer) == 0)
	  {
		  Parser_ParseSPEED(&Robot->MotorCommand, Robot->HC05_Command);
	  }
	  else if(strcmp("HOLD", (char*)DataToParse) == 0)
	  {
		  Parser_ParseHOLD(&Robot->MotorCommand, &Robot->HoldDistance, Robot->HC05_Command);
	  }
	  else if(strcmp("ENABLE", (char*)DataToParse) == 0)
	  {
		  Parser_ParseENABLE(&Robot->Enable);
	  }
	  else
	  {
		  UartLog("Wrong command value. Available: MOVE=x, SPEED=xxx, HOLD=x.xx, ENABLE=x.\r\n");
	  }
}
