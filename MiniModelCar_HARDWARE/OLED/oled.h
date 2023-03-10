#ifndef __OLED_H
#define __OLED_H			  	 
#include "sys.h"
  /**************************************************************************
项目：MiniModelCar
说明：标准两驱模型智能小车
**************************************************************************/
//-----------------OLED端口定义---------------- 

#define OLED_RST_Clr() PBout(14)=0   //RST
#define OLED_RST_Set() PBout(14)=1   //RST

#define OLED_RS_Clr() PBout(13)=0    //DC
#define OLED_RS_Set() PBout(13)=1    //DC

#define OLED_SCLK_Clr()  PBout(12)=0  //SCL
#define OLED_SCLK_Set()  PBout(12)=1   //SCL

#define OLED_SDIN_Clr()  PBout(15)=0   //SDA
#define OLED_SDIN_Set()  PBout(15)=1   //SDA

#define OLED_CMD  0	//写命令
#define OLED_DATA 1	//写数据
//OLED控制用函数
void OLED_WR_Byte(u8 dat,u8 cmd);	    
void OLED_Display_On(void);
void OLED_Display_Off(void);
void OLED_Refresh_Gram(void);		   				   		    
void OLED_Init(void);
void OLED_Clear(void);
void OLED_DrawPoint(u8 x,u8 y,u8 t);
void OLED_ShowChar(u8 x,u8 y,u8 chr,u8 size,u8 mode);
void OLED_ShowNumber(u8 x,u8 y,u32 num,u8 len,u8 size);
void OLED_ShowString(u8 x,u8 y,const u8 *p);

void OLED_ShowHexNumber(u8 x,u8 y,u32 num,u8 len,u8 size);
void OLED_ShowSignedNumber(u8 x,u8 y,float num,u8 len,u8 size);

#endif  

