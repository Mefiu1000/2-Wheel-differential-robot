/*
 * LSM6DS33.h
 *
 *  Created on: Dec 29, 2023
 *      Author: akun1
 */

#ifndef INC_MY_LIB_LSM6DS33_H_
#define INC_MY_LIB_LSM6DS33_H_

#include "main.h"


#define LSM6DS33_SAD (0x6B << 1) 	//acc and gyro address, shifted 1 bit bcs HAL uses value shifted by 1 bit

#define LSM6DS33_WHO_AM_I 0x0F		//acc and gyro ID register

//Accelerometer and Gyro axes output enable
#define AXIS_X_EN 0x01				//X-axis output enable
#define AXIS_Y_EN 0x02				//Y-axis output enable
#define AXIS_Z_EN 0x04				//Z-axis output enable

//Accelerometer and Gyro output data rate (and power mode if XL_HM_MODE = 1 (by default 0)
#define LSM6DS33_POWER_DOWN   0x00 	//Power-down
#define LSM6DS33_ODR_13HZ 	  0x01 	//Low power if XL_HM_MODE = 1
#define LSM6DS33_ODR_26HZ 	  0x02 	//Low power if XL_HM_MODE = 1
#define LSM6DS33_ODR_52HZ 	  0x03 	//Low power if XL_HM_MODE = 1
#define LSM6DS33_ODR_104HZ    0x04  //Normal mode if XL_HM_MODE = 1
#define LSM6DS33_ODR_208HZ    0x05  //Normal mode if XL_HM_MODE = 1
#define LSM6DS33_ODR_416HZ    0x06  //High performance
#define LSM6DS33_ODR_833HZ    0x07  //High performance
#define LSM6DS33_ODR_1_66KHZ  0x08  //High performance
#define LSM6DS33_ODR_3_33KHZ  0x09  //High performance, only Accelerometer
#define LSM6DS33_ODR_6_66KHZ  0x0A  //High performance, only Accelerometer

//control registers
#define LSM6DS33_CTRL1_XL 0x10		//Linear acceleration sensor control register 1 (r/w).
#define LSM6DS33_CTRL2_G  0x11		//Angular rate sensor control register 2 (r/w).
#define LSM6DS33_CTRL3_C  0x12		//Control register 3 (r/w).
#define LSM6DS33_CTRL4_C  0x13		//Control register 4 (r/w).
#define LSM6DS33_CTRL5_C  0x14		//Control register 5 (r/w).
#define LSM6DS33_CTRL6_C  0x15		//Angular rate sensor control register 6 (r/w).
#define LSM6DS33_CTRL7_G  0x16		//Angular rate sensor control register 6 (r/w).
#define LSM6DS33_CTRL8_XL 0x17		//Linear acceleration sensor control register 8 (r/w).
#define LSM6DS33_CTRL9_XL 0x18		//Linear acceleration sensor control register 9 (r/w).
#define LSM6DS33_CTRL10_C 0x19		//Control register 10 (r/w).

#define LSM6DS33_STATUS_REG 0x1E	//Status register

//output Gyro registers
#define LSM6DS33_OUTX_L_G 	0x22	//Pitch axis (X) angular rate value (LSbyte)
#define LSM6DS33_OUTX_H_G 	0x23	//Pitch axis (X) angular rate value (MSbyte)
#define LSM6DS33_OUTY_L_G 	0x24	//Roll axis (Y) angular rate value (LSbyte)
#define LSM6DS33_OUTY_H_G 	0x25	//Roll axis (Y) angular rate value (MSbyte)
#define LSM6DS33_OUTZ_L_G 	0x26	//Yaw axis (Z) angular rate value (LSbyte)
#define LSM6DS33_OUTZ_H_G 	0x27	//Yaw axis (Z) angular rate value (MSbyte)

//output Accelerometer registers
#define LSM6DS33_OUTX_L_XL 	0x28	//X-axis linear acceleration value (LSbyte)
#define LSM6DS33_OUTX_H_XL 	0x29	//X-axis linear acceleration value (MSbyte)
#define LSM6DS33_OUTY_L_XL 	0x2A	//Y-axis linear acceleration value (LSbyte)
#define LSM6DS33_OUTY_H_XL 	0x2B	//Y-axis linear acceleration value (MSbyte)
#define LSM6DS33_OUTZ_L_XL 	0x2C	//Z-axis linear acceleration value (LSbyte)
#define LSM6DS33_OUTZ_H_XL 	0x2D	//Z-axis linear acceleration value (MSbyte)

typedef struct
{
	I2C_HandleTypeDef *LSM6DS33_I2C;
	uint8_t Address;

	int16_t Acc_X;
	int16_t Acc_Y;
	int16_t Acc_Z;

	int16_t Gyro_X;
	int16_t Gyro_Y;
	int16_t Gyro_Z;

}LSM6DS33_t;



void LSM6DS33_ReadGyroData(LSM6DS33_t *LSM6DS33);
void LSM6DS33_ReadAccelerometerData(LSM6DS33_t *LSM6DS33);
void LSM6DS33_ReadAccAndGyroData(LSM6DS33_t *LSM6DS33);
uint8_t LSM6DS33_DataStatus(LSM6DS33_t *LSM6DS33);
uint8_t LSM6DS33_Init(LSM6DS33_t *LSM6DS33, I2C_HandleTypeDef* I2C, uint8_t Address);



#endif /* INC_MY_LIB_LSM6DS33_H_ */
