#include "usart_communication.h"
#include "variable.h"

uint8 Data_to_Receive[32];    //接收的数据
uint8 Data_to_Send[32];       //发送的数据//////串口接收中断

//航向角的转换
void Data_Trans()
{
  float Start_yaw_temp,Q_ANGLE_temp;
  float yaw_error=0;
  float fp[3];
  float dir;
  float x,y;
  float data;
  if(Start_yaw<0)       //副半圈
    Start_yaw_temp = 360 + Start_yaw;
  else
    Start_yaw_temp = Start_yaw;
  if(Q_ANGLE.Z<0)
    Q_ANGLE_temp = 360.0f + Q_ANGLE.Z;
  else
    Q_ANGLE_temp = Q_ANGLE.Z;
  
  yaw_error = Start_yaw_temp - Q_ANGLE_temp ;
  
  if(yaw_error<0) yaw_error +=360.0f;
  
  if(yaw_error <90)
    dir=0;
  else if(yaw_error >=90 && yaw_error <180)
    dir=1;
  else if(yaw_error >=180 && yaw_error <270)
    dir=2;
  else if(yaw_error >=270 )
    dir=3;
  
  if(dir==0)
  {
    y = yaw_error/90 *201.0f;
    x = 0.0f;
    dir = 0.0f;
  }
  else if(dir==1)
  {
    x = (yaw_error-90)/90 *201.0f;
    y = 201.0f;
    dir = -90.0f;
  }
  else if(dir==2)
  {
    x = 201.0f;
    y = 201-(yaw_error-180)/90 *201.0f;
    dir = -180.0f;
  }
  else if(dir==3)
  {
    x = 201-(yaw_error-270)/90 *201.0f; 
    y = 0;
    dir = -270.f;
  }
     
  data = Q_ANGLE.Z;
  UART_Send_Information(10,0x11,(uint8*)&data,sizeof(float));
  
  fp[0] =x;
  fp[1] =y;
  fp[2] =dir;
  UART_Send_Information(10,0x10,(uint8*)fp,sizeof(float)*3);

}
//发送给SEU上位机
void UART_Send_Information(uint8 port,uint8 Instrument,uint8 data[],uint8 data_len)
{
  uint8 sum=0,i;
  Data_to_Send[0]=0xFF;
  Data_to_Send[1]=0xFF;
  Data_to_Send[2] =data_len + 3;
  Data_to_Send[3] =port;
  Data_to_Send[4] =Instrument;
  sum +=   Data_to_Send[2] + Data_to_Send[3] + Data_to_Send[4] ;
  for(i=0;i<data_len;i++)
  {
    Data_to_Send[5+i] = data[i];
    sum += data[i];
  }
  Data_to_Send[5+data_len] =sum;
  uart_putbuff(UART0,Data_to_Send,Data_to_Send[2]+3);
}

void uart0_handler()
{
    uint8 receive_data=0;
    
    uart_querychar (UART0,&receive_data);  //等待接收一个数据  
    ANO_DT_Data_Receive_Prepare(receive_data);
    
}
static void ANO_DT_Send_Check(uint8 head, uint8 check_sum)
{
	Data_to_Send[0]=0xAA;
	Data_to_Send[1]=0xAA;
	Data_to_Send[2]=0xEF;
	Data_to_Send[3]=2;
	Data_to_Send[4]=head;
	Data_to_Send[5]=check_sum;
	
	uint8 sum = 0,i=0;
	for(i=0;i<6;i++)
		sum += Data_to_Send[i];
	Data_to_Send[6]=sum;

	uart_putbuff(UART0,Data_to_Send, 7);
}

void ANO_DT_Data_Receive_Prepare(uint8 data)
{
	static uint8 RxBuffer[50];
	static uint8 _data_len = 0,_data_cnt = 0;
	static uint8 state = 0;
	
	if(state==0&&data==0xAA)
	{
		state=1;
		RxBuffer[0]=data;
	}
	else if(state==1&&data==0xAF)
	{
		state=2;
		RxBuffer[1]=data;
	}
	else if(state==2&&data<0XF1)
	{
		state=3;
		RxBuffer[2]=data;
	}
	else if(state==3&&data<50)
	{
		state = 4;
		RxBuffer[3]=data;
		_data_len = data;
		_data_cnt = 0;
	}
	else if(state==4&&_data_len>0)
	{
		_data_len--;
		RxBuffer[4+_data_cnt++]=data;
		if(_data_len==0)
			state = 5;
	}
	else if(state==5)
	{
		state = 0;
		RxBuffer[4+_data_cnt]=data;
		Data_Receive_Anl(RxBuffer,_data_cnt+5);
	}
	else
		state = 0;
}

//发送传感器输出值
void UART_Send_Sensor(void)
{
	uint8 _cnt=0;
	uint8 i=0;
	uint8 sum = 0;
        
	Data_to_Send[_cnt++]=0xaa;
	Data_to_Send[_cnt++]=0xaa;
	Data_to_Send[_cnt++]=0x02;
	Data_to_Send[_cnt++]=0x12;
	
        Data_to_Send[_cnt++]=BYTE1(ACC_AVG.X);
	Data_to_Send[_cnt++]=BYTE0(ACC_AVG.X);
	Data_to_Send[_cnt++]=BYTE1(ACC_AVG.Y);
	Data_to_Send[_cnt++]=BYTE0(ACC_AVG.Y);
	Data_to_Send[_cnt++]=BYTE1(ACC_AVG.Z);
	Data_to_Send[_cnt++]=BYTE0(ACC_AVG.Z);
	Data_to_Send[_cnt++]=BYTE1(MPU6050_GYRO.X);
	Data_to_Send[_cnt++]=BYTE0(MPU6050_GYRO.X);
	Data_to_Send[_cnt++]=BYTE1(MPU6050_GYRO.Y);
	Data_to_Send[_cnt++]=BYTE0(MPU6050_GYRO.Y);
	Data_to_Send[_cnt++]=BYTE1(MPU6050_GYRO.Z);
	Data_to_Send[_cnt++]=BYTE0(MPU6050_GYRO.Z);
	Data_to_Send[_cnt++]=BYTE1(HMC5883L.X);
	Data_to_Send[_cnt++]=BYTE0(HMC5883L.X);
	Data_to_Send[_cnt++]=BYTE1(HMC5883L.Y);
	Data_to_Send[_cnt++]=BYTE0(HMC5883L.Y);
	Data_to_Send[_cnt++]=BYTE1(HMC5883L.Z);
	Data_to_Send[_cnt++]=BYTE0(HMC5883L.Z);
	Data_to_Send[3]=_cnt-4;
        
	for(i=0;i<_cnt;i++)
		sum += Data_to_Send[i];
	Data_to_Send[_cnt++]=sum;
	uart_putbuff(UART0,Data_to_Send,_cnt);
}

//发送姿态信息
void UART_Send_Status(void)
{
	uint8 _cnt=0;
	uint8 i=0;
	uint8 sum = 0;
	int16 temp=0;
        
	Data_to_Send[_cnt++]=0xaa;
	Data_to_Send[_cnt++]=0xaa;
	Data_to_Send[_cnt++]=0x01;
	Data_to_Send[_cnt++]=0x00;
        
        temp=(int16)(Q_ANGLE.X*100);//(Start_yaw*100);
	Data_to_Send[_cnt++]=BYTE1(temp);
	Data_to_Send[_cnt++]=BYTE0(temp);
        temp=(int16)(Q_ANGLE.Y*100);
	Data_to_Send[_cnt++]=BYTE1(temp);
	Data_to_Send[_cnt++]=BYTE0(temp);
        temp=(int16)(Q_ANGLE.Z*100);
	Data_to_Send[_cnt++]=BYTE1(temp);
	Data_to_Send[_cnt++]=BYTE0(temp);
        Data_to_Send[_cnt++]=0;
	Data_to_Send[_cnt++]=0;
        Data_to_Send[_cnt++]=0;
	Data_to_Send[_cnt++]=0;   
        Data_to_Send[_cnt++]=0;
	Data_to_Send[_cnt++]=0;
	Data_to_Send[3]=_cnt-4;
        
	for(i=0;i<_cnt;i++)
		sum += Data_to_Send[i];
	Data_to_Send[_cnt++]=sum;
	uart_putbuff(UART0,Data_to_Send,_cnt);
}

//发送电机输出值
void Data_Send_MotoPWM(void)
{
	uint8 _cnt=0;
        uint8 i=0;
        
	Data_to_Send[_cnt++]=0xAA;
	Data_to_Send[_cnt++]=0xAA;
	Data_to_Send[_cnt++]=0x06;
	Data_to_Send[_cnt++]=0;
	Data_to_Send[_cnt++]=BYTE1(MOTO_Right_PWM);
	Data_to_Send[_cnt++]=BYTE0(MOTO_Right_PWM);
	Data_to_Send[_cnt++]=BYTE1(MOTO_Left_PWM);
	Data_to_Send[_cnt++]=BYTE0(MOTO_Left_PWM);
        Data_to_Send[_cnt++]=0;
	Data_to_Send[_cnt++]=0;
	Data_to_Send[_cnt++]=0;
	Data_to_Send[_cnt++]=0;
	
	Data_to_Send[3] = _cnt-4;
	
	uint8 sum = 0;
	for(i=0;i<_cnt;i++)
		sum += Data_to_Send[i];
	
	Data_to_Send[_cnt++]=sum;
	
	uart_putbuff(UART0,Data_to_Send,_cnt);
}
//发送PID数据
void Data_Send_PID1(void)
{
	uint8 _cnt=0;
	Data_to_Send[_cnt++]=0xAA;
	Data_to_Send[_cnt++]=0xAA;
	Data_to_Send[_cnt++]=0x10;
	Data_to_Send[_cnt++]=0;
	
	int16 _temp;
	_temp = (int16) (MOTO_OUTER.P * 1000);
	Data_to_Send[_cnt++]=BYTE1(_temp);
	Data_to_Send[_cnt++]=BYTE0(_temp);
	_temp = (int16) (MOTO_OUTER.I * 1000);
	Data_to_Send[_cnt++]=BYTE1(_temp);
	Data_to_Send[_cnt++]=BYTE0(_temp);
	_temp = (int16) (MOTO_OUTER.D * 1000);
	Data_to_Send[_cnt++]=BYTE1(_temp);
	Data_to_Send[_cnt++]=BYTE0(_temp);
	_temp = 0;
	Data_to_Send[_cnt++]=BYTE1(_temp);
	Data_to_Send[_cnt++]=BYTE0(_temp);
	_temp = 0;
	Data_to_Send[_cnt++]=BYTE1(_temp);
	Data_to_Send[_cnt++]=BYTE0(_temp);
	_temp = 0;
	Data_to_Send[_cnt++]=BYTE1(_temp);
	Data_to_Send[_cnt++]=BYTE0(_temp);
	_temp = 0;
	Data_to_Send[_cnt++]=BYTE1(_temp);
	Data_to_Send[_cnt++]=BYTE0(_temp);
	_temp = 0;
	Data_to_Send[_cnt++]=BYTE1(_temp);
	Data_to_Send[_cnt++]=BYTE0(_temp);
	_temp = 0;
	Data_to_Send[_cnt++]=BYTE1(_temp);
	Data_to_Send[_cnt++]=BYTE0(_temp);
	
	Data_to_Send[3] = _cnt-4;
	
	uint8 sum = 0,i=0;
	for(i=0;i<_cnt;i++)
		sum += Data_to_Send[i];
	
	Data_to_Send[_cnt++]=sum;
	
	uart_putbuff(UART0,Data_to_Send,_cnt);
}



//下位机解析数据
void Data_Receive_Anl(uint8 *data_buf,uint8 num)
{
	uint8 sum = 0,i = 0;
	
	for(i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;         //检验校验和
        if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;	//判断帧头

	if(*(data_buf+2)==0X02)
	{
            Data_Send_PID1();     
        }

	else if(*(data_buf+2)==0X10)	//PID1
	{
                MOTO_OUTER.P = (float)((int16)(*(data_buf+4)<<8)|*(data_buf+5))*0.001;
		MOTO_OUTER.I = (float)((int16)(*(data_buf+6)<<8)|*(data_buf+7))*0.001;
		MOTO_OUTER.D = (float)((int16)(*(data_buf+8)<<8)|*(data_buf+9))*0.001;
               // ARMED = (uint16) ((int16)(*(data_buf+10)<<8)|*(data_buf+11))/100;
               //Data_Send_Check(sum);
                ANO_DT_Send_Check(*(data_buf+2),sum);
	}
}