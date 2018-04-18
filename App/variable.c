#include "variable.h"

//所有的变量

S_INT16_XYZ MPU6050_GYRO={0,0,0},MPU6050_ACCEL={0,0,0}; //陀螺仪、加速度计数据
S_INT16_XYZ HMC5883L={0,0,0};
float MPU6050_GYRO_LAST_X=0;
S_INT16_XYZ GYRO_OFFECT={344,-5,6},ACCEL_OFFECT={230,110,1350};   //陀螺仪、加速度计偏移
S_INT16_XYZ ACC_AVG={0,0,0};                            //滑动平均滤波后的加速度计
S_INT32_XYZ ACC_BUF[10]={0,0,0};                        //加速度计缓存

//姿态角数据
Data_Float_XYZ Q_ANGLE={0,0,0}, LAST_Q_ANGLE={0,0,0}; ; 
float ANGLE_ERR_INTEGER=0;       //姿态角度积分
float ANGLE_SPEED_INTEGER=0;
float Start_yaw=0;
uint32 Start_time=0;
//步进电机相序 A- A+ B- B+
uint8 Stop_Flag=0;
const uint16 STEP_FRE=100;    //步进电机的频率
uint8 step_motor_arr[4][4]=
{
  1,0,0,0
  ,0,0,0,1
  ,0,1,0,0
  ,0,0,1,0
//    
//   0,0,1,0
//  ,0,1,0,0
//  ,0,0,0,1
//  ,1,0,0,0
//   1,0,0,0
//  ,1,0,0,1
//  ,0,0,0,1
//  ,0,1,0,1
//  ,0,1,0,0
//  ,0,1,1,0
//  ,0,0,1,0
//  ,1,0,1,0
};
//无刷电机数据
uint32 MOTO_Left_PWM=0,MOTO_Right_PWM=0;

MOTO_PID MOTO_OUTER=
{
   0.14,   //0.1   //P
   0.02,   //1.50  //I
   0.02,  //0.2   //D
   0,     //POUT
   0,     //IUT
   0,     //DOUT
   0,     //OUT
},
MOTO_INNER=
{ 
   8.2,   //8.0,   //9.2   //P
   0.042,  //0.032,//0.032, //0.04, //I
   9.0,   //8.0,   //0.05  //D
   0,   //POUT
   0,   //IUT
   0,   //DOUT
   0,   //OUT
};

uint16 ARMED=0;