#ifndef _USART_COMMUNICATION_H_
#define _USART_COMMUNICATION_H_

#include "include.h"

#define BYTE0(dwTemp)       (*(char *)(&dwTemp))        
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))

extern uint8 Data_to_Receive[32];    //接收的数据
extern uint8 Data_to_Send[32];       //发送的数据
extern void ANO_DT_Data_Receive_Prepare(uint8 data);
extern void UART_Send_Sensor(void);
extern void UART_Send_Status(void);
extern void Data_Send_MotoPWM(void);
extern void uart0_handler(void);
extern void Data_Receive_Anl(uint8 *data_buf,uint8 num);
extern void UART_Send_Information(uint8 port,uint8 Instrument,uint8 data[],uint8 data_len);
extern void Data_Trans(void);
#endif 