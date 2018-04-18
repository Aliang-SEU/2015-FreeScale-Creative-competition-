#ifndef _VARIABLE_H
#define _VARIABLE_H

#include "include.h"

typedef struct s_int16_xyz
{
  int16 X;
  int16 Y;
  int16 Z;
}S_INT16_XYZ;


typedef struct s_int32_xyz
{
  int32 X;
  int32 Y;
  int32 Z;
}S_INT32_XYZ;

typedef struct float_xyz
{
  float X;
  float Y;
  float Z;
}Data_Float_XYZ;

typedef struct moto_pid
{
  float P;
  float I;
  float D;
  float POUT;
  float IOUT;
  float DOUT;
  float OUT;
}MOTO_PID;
//步进电机数据
extern uint8 Stop_Flag;
extern const uint16 STEP_FRE;
//传感器数据
extern float MPU6050_GYRO_LAST_X;
extern S_INT16_XYZ MPU6050_GYRO,MPU6050_ACCEL;
extern S_INT16_XYZ HMC5883L;
extern S_INT16_XYZ GYRO_OFFECT,ACCEL_OFFECT;
extern S_INT16_XYZ ACC_AVG;
extern S_INT32_XYZ ACC_BUF[10];
//姿态数据
extern Data_Float_XYZ Q_ANGLE,LAST_Q_ANGLE;
extern float ANGLE_ERR_INTEGER;
extern float ANGLE_SPEED_INTEGER;
extern float Start_yaw;
extern uint32 Start_time;
//电机输出

extern uint32 MOTO_Left_PWM,MOTO_Right_PWM;
extern MOTO_PID MOTO_OUTER,MOTO_INNER;
extern uint16 ARMED;
extern uint8 step_motor_arr[4][4];

#endif