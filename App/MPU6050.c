#include "MPU6050.h"
#include "variable.h"

uint8 MPU6050_Init()
{
    uint32 i2c_baud=0;
    
    i2c_baud=i2c_init(I2C0,400*1000);    //i2c接口初始化配置   引脚PTB2、PTB3
    
    asm("nop");asm("nop");asm("nop");
    if(i2c_baud<300000) return 0;      //确保波特率正确
    
    else
    {
        if( i2c_read_reg(I2C0,MPU6050_I2C_ADDR,WHO_AM_I)==0x68 )
          {
              i2c_write_reg(I2C0,MPU6050_I2C_ADDR,PWR_MGNT_1,0x00);   //解除休眠状态
              i2c_write_reg(I2C0,MPU6050_I2C_ADDR,SMPLRT_DIV,0x07);   //1khz输出速度
              i2c_write_reg(I2C0,MPU6050_I2C_ADDR,CONFIG,0x02);       //低通频率94hz
              i2c_write_reg(I2C0,MPU6050_I2C_ADDR,GYRO_CONFIG,0x18);  //±2000°/S
              i2c_write_reg(I2C0,MPU6050_I2C_ADDR,ACCEL_CONFIG,0x08); //±4g
              
              i2c_write_reg(I2C0,MPU6050_I2C_ADDR,USER_CTRL,0x00);    //将MPU6050配置成Bypass模式
              i2c_write_reg(I2C0,MPU6050_I2C_ADDR,BYPASS_EN_CTRL,0x02); //使能bypass模式
   
              //磁力计初始化 连续测量模式
              asm("nop");asm("nop");asm("nop");
              asm("nop");asm("nop");asm("nop");
              
              if(i2c_read_reg(I2C0,HMC5883L_SLAVE_ADDR,HMC5883L_REC_A) == 0x48)
              {
                 i2c_write_reg(I2C0,HMC5883L_SLAVE_ADDR,HMC5883L_MODE,0X01); 
                 HMC5883L_ReadData();       //300us
                  return 2;
              } 
              return 1;
          }
        return 0;
    }
}

void MPU6050_ReadData()
{
  uint8 MPU6050_buf[14]={0};    //注意数据类型
  
  MPU6050_read_reg(I2C0,MPU6050_I2C_ADDR,ACCEL_XOUT_H,MPU6050_buf);

  MPU6050_ACCEL.X = ((int16)(MPU6050_buf[0]<<8)) | MPU6050_buf[1];
  MPU6050_ACCEL.Y = ((int16)(MPU6050_buf[2]<<8)) | MPU6050_buf[3];
  MPU6050_ACCEL.Z = ((int16)(MPU6050_buf[4]<<8)) | MPU6050_buf[5];
  
  MPU6050_GYRO.X = ((int16)(MPU6050_buf[8]<<8)) | MPU6050_buf[9];
  MPU6050_GYRO.Y = ((int16)(MPU6050_buf[10]<<8)) | MPU6050_buf[11];
  MPU6050_GYRO.Z = ((int16)(MPU6050_buf[12]<<8)) | MPU6050_buf[13];
}
void HMC5883L_ReadData()
{
  uint8 HMC5883L_buf[6]={0};
 
  HMC5883L_read_reg(I2C0,HMC5883L_SLAVE_ADDR,HMC5883L_XOUT_H,HMC5883L_buf);
  i2c_write_reg(I2C0,HMC5883L_SLAVE_ADDR,HMC5883L_MODE,0X01); 
  
  HMC5883L.X = ((int16)(HMC5883L_buf[0]<<8)) |HMC5883L_buf[1];
  HMC5883L.Y = ((int16)(HMC5883L_buf[2]<<8)) |HMC5883L_buf[3];
  HMC5883L.Z = ((int16)(HMC5883L_buf[4]<<8)) |HMC5883L_buf[5];
  
}
void MPU6050_Calc_Offect()
{
  static uint16 cnt=0;
  static S_INT32_XYZ ACC_BUF={0,0,0},GYRO_BUF={0,0,0};
 
  if(cnt==0)
  {
     GYRO_BUF.X=0;
     GYRO_BUF.Y=0;
     GYRO_BUF.Z=0;
     ACC_BUF.X=0;
     ACC_BUF.Y=0;
     ACC_BUF.Z=0;
     cnt++;
  }
  while(cnt!=500)
  {
    MPU6050_ReadData();
    lptmr_delay_ms(1);
    GYRO_BUF.X += MPU6050_GYRO.X;
    GYRO_BUF.Y += MPU6050_GYRO.Y;
    GYRO_BUF.Z += MPU6050_GYRO.Z;
    ACC_BUF.X += MPU6050_ACCEL.X;
    ACC_BUF.Y += MPU6050_ACCEL.Y;
    ACC_BUF.Z += MPU6050_ACCEL.Z;
    cnt++;
  }
    //ACCEL_OFFECT.X = ACC_BUF.X / cnt;
    //ACCEL_OFFECT.Y = ACC_BUF.Y / cnt;
    //ACC_OFFECT.Z = ACC_BUF.Z / cnt;
    GYRO_OFFECT.X = GYRO_BUF.X / (cnt-1);
    GYRO_OFFECT.Y = GYRO_BUF.Y / (cnt-1);
    GYRO_OFFECT.Z = GYRO_BUF.Z / (cnt-1);
    cnt=0;
    return;
}
