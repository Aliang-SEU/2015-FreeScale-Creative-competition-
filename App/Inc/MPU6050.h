#ifndef _MPU6050_H_
#define _MPU6050_H_

#include "include.h"

//陀螺仪内部地址定义
#define MPU6050_I2C_ADDR 0x68
#define SMPLRT_DIV      0X19
#define CONFIG          0X1A
#define GYRO_CONFIG     0X1B
#define ACCEL_CONFIG    0x1C

#define ACCEL_XOUT_H    0X3B
#define ACCEL_XOUT_L    0X3C
#define ACCEL_YOUT_H    0X3D
#define ACCEL_YOUT_L    0X3E
#define ACCEL_ZOUT_H    0X3F
#define ACCEL_ZOUT_L    0X40

#define TEMP_OUT_H      0X41
#define TEMP_OUT_L      0X42

#define GYRO_XOUT_H     0X43
#define GYRO_XOUT_L     0X44
#define GYRO_YOUT_H     0X45
#define GYRO_YOUT_L     0X46
#define GYRO_ZOUT_H     0X47
#define GYRO_ZOUT_L     0X48

#define USER_CTRL       0X6A
#define PWR_MGNT_1      0X6B
#define PWR_MGNT_2      0X6C
#define WHO_AM_I        0X75

#define BYPASS_EN_CTRL  0x37

#define HMC5883L_SLAVE_ADDR  0x1E

#define HMC5883L_CONFIG_A 0x00
#define HMC5883L_CONFIG_B 0x01
#define HMC5883L_MODE   0x02
#define HMC5883L_XOUT_H 0x03
#define HMC5883L_XOUT_L 0x04
#define HMC5883L_YOUT_H 0x05
#define HMC5883L_YOUT_L 0x06
#define HMC5883L_ZOUT_H 0x07
#define HMC5883L_ZOUT_L 0x08
#define HMC5883L_STATE  0x09
#define HMC5883L_REC_A  0x0A
#define HMC5883L_REC_B  0x0B
#define HMC5883L_REC_C  0x0C

extern uint8 MPU6050_Init(void);
extern void MPU6050_ReadData(void);
extern void MPU6050_Calc_Offect(void);
extern void HMC5883L_ReadData();
#endif 