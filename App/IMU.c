#include "IMU.h"
#include "math.h"

#define Gyro_G 	0.0610351f	  		//陀螺仪变为度
#define Gyro_Gr	0.0010653f		  	//陀螺仪 2000 灵敏度
#define FILTER_NUM 10                           //滑动滤波处理的点数
  
void Data_To_Deal()
{
  static uint8 i=0,cnt=0;
  int32 temp_x=0,temp_y=0,temp_z=0;
   
  //加速度进行滑动平均滤波
  ACC_BUF[cnt].X =  MPU6050_ACCEL.X-ACCEL_OFFECT.X;
  ACC_BUF[cnt].Y =  MPU6050_ACCEL.Y-ACCEL_OFFECT.Y;
  ACC_BUF[cnt].Z =  MPU6050_ACCEL.Z-ACCEL_OFFECT.Z;
  
  for(i=0;i<FILTER_NUM;i++)
  {
      temp_x += ACC_BUF[i].X;
      temp_y += ACC_BUF[i].Y;
      temp_z += ACC_BUF[i].Z;
  }
  
  ACC_AVG.X = temp_x/FILTER_NUM;
  ACC_AVG.Y = temp_y/FILTER_NUM;
  ACC_AVG.Z = temp_z/FILTER_NUM;
  
  cnt++;
  if(cnt==FILTER_NUM) cnt=0;
  //陀螺仪数据进行低通滤波
  MPU6050_GYRO.X -= GYRO_OFFECT.X;
  MPU6050_GYRO.Y -= GYRO_OFFECT.Y;
  MPU6050_GYRO.Z -= GYRO_OFFECT.Z;
  //MPU6050_GYRO = Low_Pass();
}

void Get_Attitude()
{
  AHRSupdate(MPU6050_GYRO.X*Gyro_Gr,MPU6050_GYRO.Y*Gyro_Gr,MPU6050_GYRO.Z*Gyro_Gr,
            ACC_AVG.X,ACC_AVG.Y,ACC_AVG.Z,HMC5883L.X , HMC5883L.Y ,HMC5883L.Z);
}

S_INT16_XYZ Low_Pass(void)
{
  S_INT16_XYZ Result={0,0,0};
  static S_INT16_XYZ Last_Result={0,0,0};
  static float a=0.8;
  
  Result.X = (int16)((1-a)*MPU6050_GYRO.X+a*Last_Result.X);
  Result.Y = (int16)((1-a)*MPU6050_GYRO.Y+a*Last_Result.Y);
  Result.Z = (int16)((1-a)*MPU6050_GYRO.Z+a*Last_Result.Z);
 
  Last_Result = Result;
  return Result;
}
//姿态融合的参数	

#define Kp  20.0f		//比例增益控制的收敛速度
#define Ki  0.002f	        //积分增益控制的收敛速度
#define halfT  0.001f		//一半的样本周期

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;     //四元数初始值（需要调整）
float exInt = 0, eyInt = 0, ezInt = 0;     

void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az)
{
  float norm;
  float vx, vy, vz;
  float ex, ey, ez;

  float q0q0 = q0*q0;
  float q0q1 = q0*q1;
  float q0q2 = q0*q2;
  float q1q1 = q1*q1;
  float q1q3 = q1*q3;
  float q2q2 = q2*q2;
  float q2q3 = q2*q3;
  float q3q3 = q3*q3;
	
  if(ax*ay*az==0)
 	return;
	
  norm = sqrt(ax*ax + ay*ay + az*az);       
  ax = ax /norm;
  ay = ay / norm;
  az = az / norm;

  vx = 2*(q1q3 - q0q2);												
  vy = 2*(q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3 ;

  ex = (ay*vz - az*vy) ;                           					
  ey = (az*vx - ax*vz) ;
  ez = (ax*vy - ay*vx) ;

  exInt = exInt + ex * Ki;								
  eyInt = eyInt + ey * Ki;
  ezInt = ezInt + ez * Ki;

  gx = gx + Kp*ex + exInt;					   						
  gy = gy + Kp*ey + eyInt;
  gz = gz + Kp*ez + ezInt;				   							
					   
  q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
  q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
  q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
  q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;

  norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 = q0 / norm;
  q1 = q1 / norm;
  q2 = q2 / norm;
  q3 = q3 / norm;

 // Q_ANGLE.Y = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3; // pitch
 // Q_ANGLE.Z = -MPU6050_GYRO.Z*0.0005*Gyro_G;// yaw
 // if(Q_ANGLE.Y<90 && Q_ANGLE.Y>-90)		
  
  Q_ANGLE.X = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3; // roll
}


void AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
  float norm;
  float hx, hy, hz, bx, bz;
  float vx, vy, vz, wx, wy, wz;
  float ex, ey, ez;

  float q0q0 = q0*q0;
  float q0q1 = q0*q1;
  float q0q2 = q0*q2;
  float q0q3 = q0*q3;
  float q1q1 = q1*q1;
  float q1q2 = q1*q2;
  float q1q3 = q1*q3;
  float q2q2 = q2*q2;   
  float q2q3 = q2*q3;
  float q3q3 = q3*q3;          
  
  //矩阵元素
  //   [   1-2*q3*q3-2*q1*q1                      2*q1*q2-2*q0*q3      ]
  //   [   2*q2*q3-2*q0*q1     1-2*q2*q2-2*q1*q1  2*q1*q3+2*q0*q2      ]
  //   [   2*q1*q2+2*q0*q3                        1-2*q2*q2-2*q3*q3    ]
  
  norm = sqrt(ax*ax + ay*ay + az*az);       
  ax = ax/ norm;
  ay = ay/ norm;
  az = az/ norm;
  norm = sqrt(mx*mx + my*my + mz*mz);          
  mx = mx/ norm;
  my = my/ norm;
  mz = mz/ norm;         
        
  hx = 2*mx*(0.5 - q2q2 - q3q3) + 2*my*(q1q2 - q0q3) + 2*mz*(q1q3 + q0q2);
  hy = 2*mx*(q1q2 + q0q3) + 2*my*(0.5 - q1q1 - q3q3) + 2*mz*(q2q3 - q0q1);
  hz = 2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.5 - q1q1 - q2q2);         
  bx = sqrt((hx*hx) + (hy*hy));
  bz = hz;        
            
  vx = 2*(q1q3 - q0q2);
  vy = 2*(q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3;
  wx = 2*bx*(0.5 - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);
  wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);
  wz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5 - q1q1 - q2q2);  
        
  ex = (ay*vz - az*vy) + (my*wz - mz*wy);
  ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
  ez = (ax*vy - ay*vx) + (mx*wy - my*wx);
        
  exInt = exInt + ex*Ki;
  eyInt = eyInt + ey*Ki;
  ezInt = ezInt + ez*Ki;
        
  gx = gx + Kp*ex + exInt;
  gy = gy + Kp*ey + eyInt;
  gz = gz + Kp*ez + ezInt;
        
  q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
  q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
  q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
  q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  
        
  norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 = q0 / norm;
  q1 = q1 / norm;
  q2 = q2 / norm;
  q3 = q3 / norm;
  
  Q_ANGLE.Z = atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2*q2 - 2 * q3* q3 + 1)* 57.3; // yaw 
  Q_ANGLE.Y = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3; // pitch
  Q_ANGLE.X = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3; // roll

}
