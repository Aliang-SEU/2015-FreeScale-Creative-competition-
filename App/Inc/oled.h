#ifndef OLED_H
#define OLED_H

#include "include.h"
//-------------------------------OLED--------------------------------------//

#define OLED_RST  PTA14_OUT
#define OLED_RS   PTA13_OUT
#define OLED_CS   PTA12_OUT  
#define OLED_SDIN PTA16_OUT
#define OLED_SCLK PTA15_OUT	       						  

		     
#define OLED_CMD  0	//写命令
#define OLED_DATA 1	//写数据
//OLED控制用函数
void OLED_WR_Byte(uint8 dat,uint8 cmd);	    
void OLED_Display_On(void);
void OLED_Display_Off(void);
void OLED_Refresh_Gram(void);		   
						   		    
void OLED_Init(void);
void OLED_Clear(void);
void OLED_DrawPoint(uint8 x,uint8 y,uint8 t);
void OLED_Fill(uint8 x1,uint8 y1,uint8 x2,uint8 y2,uint8 dot);
void OLED_ShowChar(uint8 x,uint8 y,uint8 chr,uint8 size,uint8 mode);
void OLED_ShowNum(uint8 x,uint8 y,uint32 num,uint8 len,uint8 size);
void OLED_ShowString_1608(uint8 x,uint8 y,const uint8 *p,uint8 mode);
void OLED_ShowString_1206(uint8 x,uint8 y,const uint8 *p,uint8 mode);	 
#endif  