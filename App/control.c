#include "control.h"
#include "variable.h"
#include "oled.h"
#define MinPWM 1000     //最小油门行程 对应1ms--100HZ
#define MaxPWM 1800     //最大油门行程 对应2ms--100HZ
#define Gyro_G 0.0610351f
#define INTEHER_MAX 20
#define INTEHER_MIN -20

//无刷电机控制
void Moto_Out(int32 moto1_pwm,int32 moto2_pwm)
{
  if(moto1_pwm > MaxPWM) moto1_pwm = MaxPWM;
  if(moto2_pwm > MaxPWM) moto2_pwm = MaxPWM;
  if(moto1_pwm < MinPWM) moto1_pwm = MinPWM;
  if(moto2_pwm < MinPWM) moto2_pwm = MinPWM;
  
  MOTO_Right_PWM = moto1_pwm-1000;   //发送给上位机
  MOTO_Left_PWM = moto2_pwm-1000;
  
  ftm_pwm_duty(FTM0,FTM_CH4,moto1_pwm);
  ftm_pwm_duty(FTM0,FTM_CH5,moto2_pwm); 
}
void control()
{
  static int32  OUT_Left_Outer=0,OUT_Left_Inner=0,OUT_Right_Outer=0,OUT_Right_Inner=0;
  static uint8 cnt=0;
  static uint8 start_flag=0;
  
  if((Q_ANGLE.X<5 && Q_ANGLE.X>-5)  && Start_time<1500)
  {
    start_flag=1;
    Start_time++;
  }
  if(start_flag==0) return;
  //车子已经启动
  if(start_flag==1 && Start_time==1500) 
  {
      Start_yaw=Q_ANGLE.Z;
      OLED_ShowString_1206(0,0,"Get Ready!!!!!",12);
      OLED_Refresh_Gram();
      ftm_pwm_duty(FTM2,FTM_CH0,50);
      Start_time++;             //501
  }
  if(start_flag==1 && Start_time>=1501 && Stop_Flag==0)
  {
     if(Start_time<=5000)
     {
       Start_time++;
     }
     else
     {
        if((Q_ANGLE.Z-Start_yaw<4) &&(Q_ANGLE.Z-Start_yaw>0))
        {
          Stop_Flag=1;
          OLED_Clear();   
          OLED_ShowString_1206(0,0,"Car stop!!!!!",12);
          OLED_Refresh_Gram();
          ftm_pwm_duty(FTM2, FTM_CH0,0);
        }
     }
  }
  if(Q_ANGLE.X >24 || Q_ANGLE.X<-24)    //角度过大停止运行
  {
    Moto_Out(1000,1000);
    return;
  }
 
  else
  {
    if(cnt>=4)
    {
      //外环PID(角度PID)
      ANGLE_ERR_INTEGER -= Q_ANGLE.X;
      
      if(ANGLE_ERR_INTEGER>INTEHER_MAX)
        ANGLE_ERR_INTEGER=INTEHER_MAX;
      if(ANGLE_ERR_INTEGER<INTEHER_MIN)
        ANGLE_ERR_INTEGER=INTEHER_MIN;
      
      float p=0;
      p=MOTO_OUTER.P ;
      MOTO_OUTER.POUT = (float)(p*(-Q_ANGLE.X));
     
      //限幅
      //if(MOTO_OUTER.POUT > 0.5) MOTO_OUTER.POUT = 0.5;
      //else if(MOTO_OUTER.POUT < -0.5) MOTO_OUTER.POUT = -0.5;
      
      MOTO_OUTER.IOUT = (float)(MOTO_OUTER.I*ANGLE_ERR_INTEGER);
      MOTO_OUTER.DOUT = (float)(MOTO_OUTER.D*(-MPU6050_GYRO.X*Gyro_G));
      
      MOTO_OUTER.OUT =  MOTO_OUTER.POUT  + MOTO_OUTER.IOUT  + MOTO_OUTER.DOUT ;
  
      cnt=0;
    }
    cnt++;
    //内环PID(角速度PID)
      
      MOTO_INNER.POUT = (float)(MOTO_INNER.P*(MOTO_OUTER.OUT-MPU6050_GYRO.X*Gyro_G));
      MOTO_INNER.DOUT = (float)(MOTO_INNER.D*(MOTO_OUTER.OUT-MPU6050_GYRO.X*Gyro_G-MPU6050_GYRO_LAST_X));
      
      ANGLE_SPEED_INTEGER += (MOTO_OUTER.OUT-MPU6050_GYRO.X*Gyro_G);
     
      if(ANGLE_SPEED_INTEGER>1900)
        ANGLE_SPEED_INTEGER = 1900;
      else if(ANGLE_SPEED_INTEGER<-1900)
        ANGLE_SPEED_INTEGER = -1900;
      

      MOTO_INNER.IOUT = (float)(MOTO_INNER.I*ANGLE_SPEED_INTEGER);
      
      MPU6050_GYRO_LAST_X=MOTO_OUTER.OUT-MPU6050_GYRO.X*Gyro_G;
      
      MOTO_INNER.OUT = MOTO_INNER.POUT + MOTO_INNER.DOUT + MOTO_INNER.IOUT ;
      
      if(MOTO_INNER.OUT>400) MOTO_INNER.OUT=400;
      if(MOTO_INNER.OUT<-400) MOTO_INNER.OUT=-400;
         
      
      OUT_Right_Inner = (int32) (1400   + MOTO_INNER.OUT);
      OUT_Left_Inner = (int32) (1400  - MOTO_INNER.OUT);
      
      Moto_Out(OUT_Right_Inner,OUT_Left_Inner);
  }
}

void Moto_Init()
{
    ftm_pwm_init(FTM0, FTM_CH4,50, 1000); //左边 100hz
    ftm_pwm_init(FTM0, FTM_CH5,50, 1000); //右边 100hz
    ftm_pwm_init(FTM0, FTM_CH0,50, 2100); //1456正中心 1800正中心
    ftm_pwm_init(FTM2, FTM_CH0,2500,0);
   // StepMoto_Init();    //步进电机初始化
}

//步进电机控制
void StepMoto_Init()
{
  gpio_init(PTC8,GPO,1);//8+,10- 逆时针；8-,10+ 顺时针 B组线圈
  gpio_init(PTC10,GPO,0);
  gpio_init(PTC12,GPO,0);//12+,14- 逆时针；12-,14+ 顺时针 A组线圈
  gpio_init(PTC14,GPO,0);
}
void StepMoto_Control()
{
  static uint8 step=0;
  gpio_set(PTC8,step_motor_arr[step][0]);
  gpio_set(PTC10,step_motor_arr[step][1]);
  gpio_set(PTC12,step_motor_arr[step][2]);
  gpio_set(PTC14,step_motor_arr[step][3]);
  step++;
  if(step>=4)
    step=0;
}