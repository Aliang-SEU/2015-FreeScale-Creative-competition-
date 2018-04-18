#ifndef _IMU_H_
#define _IMU_H_

#include "variable.h"

#define RtA 	57.324841f      //弧度到角度
#define AtR    	0.0174533f	//度到角度		
#define Acc_G 	0.0011963f   	//加速度变为g

extern void Get_Attitude(void);
extern void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az);
extern void Data_To_Deal(void);
extern S_INT16_XYZ Low_Pass(void);
extern void AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
#endif