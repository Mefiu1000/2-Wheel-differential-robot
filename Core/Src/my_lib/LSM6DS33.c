/*
 * LSM6DS33.c
 *
 *  Created on: Dec 29, 2023
 *      Author: akun1
 */

#include "my_lib/LSM6DS33.h"

#define LSM6DS33_I2C_TIMEOUT 1000
#define LSM6DS33_I2C_ERROR 1


/** Read8Bit
 * @brief Reads 8 bits register
 *
 * @param LSM6DS33 Pointer to structure that contains sensor configuration.
 * @param Register Register address to read.
 *
 * @retval Read register.
 * */
static uint8_t Read8Bit(LSM6DS33_t *LSM6DS33, uint8_t Register)
{
	uint8_t Value;

	HAL_I2C_Mem_Read(LSM6DS33->LSM6DS33_I2C, LSM6DS33->Address, Register, 1, &Value, 1, LSM6DS33_I2C_TIMEOUT);

	return Value;
}

/** Read16Bit
 * @brief Reads 16 bits register
 *
 * @param LSM6DS33 Pointer to structure that contains sensor configuration.
 * @param Register Register address to read.
 *
 * @retval Read register.
 * */
static uint16_t Read16Bit(LSM6DS33_t *LSM6DS33, uint8_t Register)
{
	uint8_t Value[2];

	HAL_I2C_Mem_Read(LSM6DS33->LSM6DS33_I2C, LSM6DS33->Address, Register, 1, Value, 2, LSM6DS33_I2C_TIMEOUT);

	return (Value[0] | (Value[1] << 8));
}

/** Write8bit
 * @brief Writes 8 bits register
 *
 * @param LSM6DS33 Pointer to structure that contains sensor configuration.
 * @param Register Register address to write.
 * @param Value Value to write to the register.
 *
 * @retval None.
 * */
static void Write8bit(LSM6DS33_t *LSM6DS33, uint8_t Register, uint8_t Value)
{

	HAL_I2C_Mem_Write(LSM6DS33->LSM6DS33_I2C, LSM6DS33->Address, Register, 1, &Value, 1, 1000);
}

/** LSM6DS33_GyroEnableAxes
 * @brief Enables Gyro axes measurement operation.
 *
 * @param LSM6DS33 Pointer to structure that contains sensor configuration.
 * @param Axes Axes to enable.
 *
 * @retval None.
 * */
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

/** LSM6DS33_GyroDataRate
 * @brief Sets Gyro output data rate frequency and/or operation mode.
 *
 * @param LSM6DS33 Pointer to structure that contains sensor configuration.
 * @param ODR Output data rate mode.
 *
 * @retval None.
 * */
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

/** LSM6DS33_ReadGyroData
 * @brief Reads Gyro measurements from all axes.
 *
 * @param LSM6DS33 Pointer to structure that contains sensor configuration.
 *
 * @retval None.
 * */
void LSM6DS33_ReadGyroData(LSM6DS33_t *LSM6DS33)
{
	LSM6DS33->Gyro_X =  Read16Bit(LSM6DS33, LSM6DS33_OUTX_L_G);
	LSM6DS33->Gyro_Y =  Read16Bit(LSM6DS33, LSM6DS33_OUTY_L_G);
	LSM6DS33->Gyro_Z =  Read16Bit(LSM6DS33, LSM6DS33_OUTZ_L_G);
}

/** LSM6DS33_AccelerometerEnableAxes
 * @brief Enables Accelerometer axes measurement operation.
 *
 * @param LSM6DS33 Pointer to structure that contains sensor configuration.
 * @param Axes Axes to enable.
 *
 * @retval None.
 * */
static void LSM6DS33_AccelerometerEnableAxes(LSM6DS33_t *LSM6DS33, uint8_t Axes)
{
	if(Axes < 0 || Axes > 7) //if wrong value, activate all axes
	{
		Axes = 7;
	}

	Write8bit(LSM6DS33, LSM6DS33_CTRL9_XL, (Axes << 3));
}

/** LSM6DS33_AccelerometerDataRate
 * @brief Sets Accelerometer output data rate frequency and/or operation mode.
 *
 * @param LSM6DS33 Pointer to structure that contains sensor configuration.
 * @param ODR Output data rate mode.
 *
 * @retval None.
 * */
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

/** LSM6DS33_ReadAccelerometerData
 * @brief Reads Accelerometer measurements from all axes.
 *
 * @param LSM6DS33 Pointer to structure that contains sensor configuration.
 *
 * @retval None.
 * */
void LSM6DS33_ReadAccelerometerData(LSM6DS33_t *LSM6DS33)
{
	LSM6DS33->Acc_X =  Read16Bit(LSM6DS33, LSM6DS33_OUTX_L_XL);
	LSM6DS33->Acc_Y =  Read16Bit(LSM6DS33, LSM6DS33_OUTY_L_XL);
	LSM6DS33->Acc_Z =  Read16Bit(LSM6DS33, LSM6DS33_OUTZ_L_XL);
}

/** LSM6DS33_ReadAccAndGyroData
 * @brief Reads Accelerometer and Gyro measurements from all axes.
 *
 * @param LSM6DS33 Pointer to structure that contains sensor configuration.
 *
 * @retval None.
 * */
void LSM6DS33_ReadAccAndGyroData(LSM6DS33_t *LSM6DS33)
{
	LSM6DS33_ReadAccelerometerData(LSM6DS33);
	LSM6DS33_ReadGyroData(LSM6DS33);
}

/** LSM6DS33_DataStatus
 * @brief Reads status register data to check if
 * new data from Gyro and Accelerometer is available.
 *
 * @param LSM6DS33 Pointer to structure that contains sensor configuration.
 *
 * @retval Status register value.
 * */
uint8_t LSM6DS33_DataStatus(LSM6DS33_t *LSM6DS33)
{
	uint8_t STATUS;

	STATUS = Read8Bit(LSM6DS33, LSM6DS33_STATUS_REG);

	return (STATUS & 0x03);
}


/** LSM6DS33_Init
 * @brief LSM6DS33 configuration initialization.
 *
 * @param LSM6DS33 Pointer to structure that contains sensor configuration.
 * @param I2C Pointer to a I2C_HandleTypeDef structure that contains
 * the configuration information for the LSM6DS33 I2C.
 * @param Address Contains info about sensor address.
 *
 * @retval Initialization success confirmation.
 * */
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

