/*
 * LSM6DS33.c
 *
 *  Created on: Dec 29, 2023
 *      Author: akun1
 */

#include "my_lib/LSM6DS33.h"

#define LSM6DS33_I2C_TIMEOUT 1000
#define LSM6DS33_I2C_ERROR 1

static uint8_t Read8Bit(LSM6DS33_t *LSM6DS33, uint8_t Register)
{
	uint8_t Value;

	HAL_I2C_Mem_Read(LSM6DS33->LSM6DS33_I2C, LSM6DS33->Address, Register, 1, &Value, 1, LSM6DS33_I2C_TIMEOUT);

	return Value;
}

static uint16_t Read16Bit(LSM6DS33_t *LSM6DS33, uint8_t Register)
{
	uint8_t Value[2];

	HAL_I2C_Mem_Read(LSM6DS33->LSM6DS33_I2C, LSM6DS33->Address, Register, 1, Value, 2, LSM6DS33_I2C_TIMEOUT);

	return (Value[0] | (Value[1] << 8));
}

static void Write8bit(LSM6DS33_t *LSM6DS33, uint8_t Register, uint8_t Value)
{

	HAL_I2C_Mem_Write(LSM6DS33->LSM6DS33_I2C, LSM6DS33->Address, Register, 1, &Value, 1, 1000);
}

static void LSM6DS33_GyroEnableAxes(LSM6DS33_t *LSM6DS33, uint8_t Axes)
{
	uint8_t CTRL;

	if(Axes < 0 || Axes > 7) //if wrong value, activate all axes
	{
		Axes = 7;
	}

	CTRL = Read8Bit(LSM6DS33, LSM6DS33_CTRL10_C);

	CTRL = CTRL & 0xC7; 	//1100 0111 mask to zero axes settings
	CTRL |= (Axes << 3);

	Write8bit(LSM6DS33, LSM6DS33_CTRL10_C, CTRL);
}

static void LSM6DS33_GyroDataRate(LSM6DS33_t *LSM6DS33, uint8_t ODR)
{
	uint8_t CTRL;

	if(ODR < 0 || ODR > 8) //if wrong value, ODR = 208 Hz
	{
		ODR = 5;
	}

	CTRL = Read8Bit(LSM6DS33, LSM6DS33_CTRL2_G);

	CTRL = CTRL & 0x0F; 	//0000 1111 mask
	CTRL |= (ODR << 4);

	Write8bit(LSM6DS33, LSM6DS33_CTRL2_G, CTRL);
}

void LSM6DS33_ReadGyroData(LSM6DS33_t *LSM6DS33)
{
	LSM6DS33->Gyro_X =  Read16Bit(LSM6DS33, LSM6DS33_OUTX_L_G);
	LSM6DS33->Gyro_Y =  Read16Bit(LSM6DS33, LSM6DS33_OUTY_L_G);
	LSM6DS33->Gyro_Z =  Read16Bit(LSM6DS33, LSM6DS33_OUTZ_L_G);
}

static void LSM6DS33_AccelerometerEnableAxes(LSM6DS33_t *LSM6DS33, uint8_t Axes)
{
	if(Axes < 0 || Axes > 7) //if wrong value, activate all axes
	{
		Axes = 7;
	}

	Write8bit(LSM6DS33, LSM6DS33_CTRL9_XL, (Axes << 3));
}

static void LSM6DS33_AccelerometerDataRate(LSM6DS33_t *LSM6DS33, uint8_t ODR)
{
	uint8_t CTRL;

	if(ODR < 0 || ODR > 10) //if wrong value, ODR = 208 Hz
	{
		ODR = 5;
	}

	CTRL = Read8Bit(LSM6DS33, LSM6DS33_CTRL1_XL);

	CTRL = CTRL & 0x0F; 	//0000 1111 mask
	CTRL |= (ODR << 4);

	Write8bit(LSM6DS33, LSM6DS33_CTRL1_XL, CTRL);
}

void LSM6DS33_ReadAccelerometerData(LSM6DS33_t *LSM6DS33)
{
	LSM6DS33->Acc_X =  Read16Bit(LSM6DS33, LSM6DS33_OUTX_L_XL);
	LSM6DS33->Acc_Y =  Read16Bit(LSM6DS33, LSM6DS33_OUTY_L_XL);
	LSM6DS33->Acc_Z =  Read16Bit(LSM6DS33, LSM6DS33_OUTZ_L_XL);
}

void LSM6DS33_ReadAccAndGyroData(LSM6DS33_t *LSM6DS33)
{
	LSM6DS33_ReadAccelerometerData(LSM6DS33);
	LSM6DS33_ReadGyroData(LSM6DS33);
}

uint8_t LSM6DS33_DataStatus(LSM6DS33_t *LSM6DS33)
{
	uint8_t STATUS;

	STATUS = Read8Bit(LSM6DS33, LSM6DS33_STATUS_REG);

	return (STATUS & 0x03);
}

uint8_t LSM6DS33_Init(LSM6DS33_t *LSM6DS33, I2C_HandleTypeDef* I2C, uint8_t Address)
{
	uint8_t WhoAmI;

	LSM6DS33->LSM6DS33_I2C = I2C;
	LSM6DS33->Address = Address;

	WhoAmI = Read8Bit(LSM6DS33, LSM6DS33_WHO_AM_I);

	if(WhoAmI != 0x69) //Chip ID - read datasheet
	{
		return LSM6DS33_I2C_ERROR;
	}

	//Acc settings
	LSM6DS33_AccelerometerEnableAxes(LSM6DS33, AXIS_X_EN | AXIS_Y_EN | AXIS_Z_EN);
	LSM6DS33_AccelerometerDataRate(LSM6DS33, LSM6DS33_ODR_416HZ);

	//Gyro settings
	LSM6DS33_GyroEnableAxes(LSM6DS33, AXIS_X_EN | AXIS_Y_EN | AXIS_Z_EN);
	LSM6DS33_GyroDataRate(LSM6DS33, LSM6DS33_ODR_416HZ);

	return 0;
}

