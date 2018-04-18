#include "include.h"
#include "main.h"

static uint8 Uart_Send_Flag=0;


void main(void)
{
   DisableInterrupts;
   
   Init_All();
 
   EnableInterrupts;    

   while(1)
   {
     if(Uart_Send_Flag==1)
     {
   //  Data_Send_MotoPWM();       //串口数据总共需要4.43ms
    // UART_Send_Sensor();
        UART_Send_Status();
       //Data_Trans();
        Uart_Send_Flag=0;
     }
   }
}

void Init_All()
{
    uint16 i=0;
    
    Moto_Init();                //上电后进行电机的初始化
    lptmr_delay_ms(2000);
    led_init(LED_MAX);          //LED灯的初始化
    OLED_Init();                //OLED初始化
    
    uart_init(UART0,115200);    //串口以及串口接收中断的初始化

    i= MPU6050_Init();
    if(i==2)    {OLED_ShowString_1206(0,0,"MPU6050 Init Success!",12);
                 OLED_ShowString_1206(0,16,"HMC5883L Init Success!",12);}
    else        OLED_ShowString_1206(0,0,"MPU6050 Init Fail",12);
    
    OLED_Refresh_Gram();
    lptmr_delay_ms(1000);
    OLED_Clear();   
    MPU6050_Calc_Offect();//陀螺仪数据自矫正
    //矫正完成后亮灯指示状态
    led(LED2,LED_ON);lptmr_delay_ms(500);led(LED2,LED_OFF);lptmr_delay_ms(500);led(LED2,LED_ON);lptmr_delay_ms(500);led(LED2,LED_OFF);
   
    for(i=0;i<400;i++)
    {
       MPU6050_ReadData();  
       Data_To_Deal();       
       Get_Attitude();            //287.5us
       lptmr_delay_ms(2);
    }
   
    pit_init_ms(PIT0,1);                            //PIT定时器初始化 1ms中断            
    set_vector_handler(PIT0_VECTORn,pit_handler);   //设置中断服务函数到中断向量表里  服务程序在usart_communication内
    enable_irq(PIT0_IRQn);                          //使能PIT中断  
}

//定时中断服务程序
void pit_handler()
{
   static uint8 ms1=0,ms2=0,ms5=0,ms10=0;
   static uint16 ms1000=0;
   PIT_Flag_Clear(PIT0); 
   
   ms1++;ms2++;ms5++;ms10++,ms1000++;
   
   //1ms读取一次数据
   if(ms1==1)
   {
     ms1=0;                     //700us
     MPU6050_ReadData();  
     Data_To_Deal();       
   }
   //2ms解算一次姿态
   if(ms2==2)
   {
      ms2=0;
      Get_Attitude();            //287.5
   }
   //3ms控制一次
   if(ms10==3)
   {
     ms10=0;
     control();                 //19.48us
   }
   //7ms解算一次姿态
   if(ms5==8)                   
   {
     ms5=0;
     HMC5883L_ReadData();       //300us    
   }
   if(ms1000==50)
   {
     ms1000=0;
      if(Start_time>=1500)
        Uart_Send_Flag=1;   
   }
}


