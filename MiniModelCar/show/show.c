#include "show.h"

#include "stdio.h"
#include "stdlib.h"
  /**************************************************************************
项目：MiniModelCar
说明：标准两驱模型智能小车
**************************************************************************/
/**************************************************************************
函数功能：OLED显示
入口参数：无
返回  值：无
**************************************************************************/
extern float After_filter[M];




void oled_show(void)
{
					OLED_ShowString(00,0,"F");
//					OLED_ShowNumber(10,0,S_Input_Read,4,12);	
				//OLED_ShowNumber(10,0,After_filter[4],4,12);
//					OLED_ShowString(40,0,"B");
//					OLED_ShowNumber(50,0,S_D_Input_Read,4,12);
//					OLED_ShowNumber(50,0,Speed,4,12);
					OLED_ShowString(80,0,"R");
					//OLED_ShowNumber(90,0,UrxSt,4,12);
					OLED_ShowHexNumber(90,0,UrxSt,4,12);
					
					OLED_ShowString(0,10,"Y");
					OLED_ShowNumber(10,10,Move_Y,4,12);	
					OLED_ShowString(40,10,"Z");
					OLED_ShowNumber(50,10,Move_Z,4,12);
					OLED_ShowString(80,10,"S");
			//		OLED_ShowNumber(90,10,Speed_Num,4,12);	
						
					OLED_ShowString(0,20,"FT");
//					OLED_ShowNumber(20,20,Flag_Turn,3,12);	
					OLED_ShowString(40,20,"BA");
//					OLED_ShowNumber(60,20,Auto_Back_Flag,2,12);
					OLED_ShowString(80,20,"TF");
//					OLED_ShowNumber(100,20,TimeFlag,3,12);
														

					//=============第4行显示电机B的状态=======================//
						
						if( Target_B<0)		  OLED_ShowString(00,30,"TB-"),
																OLED_ShowNumber(15,30,-Target_B,5,12);
					else                 	OLED_ShowString(0,30,"TB+"),
																OLED_ShowNumber(15,30, Target_B,5,12); 					
					if( Encoder_B<0)		  OLED_ShowString(80,30,"EB-"),
																OLED_ShowNumber(95,30,-Encoder_B,4,12);
					else                 	OLED_ShowString(80,30,"EB+"),
																OLED_ShowNumber(95,30, Encoder_B,4,12);
										//=============第5行显示电机C的状态=======================//	
						if( Target_A<0)	  	OLED_ShowString(00,40,"TA-"),
																OLED_ShowNumber(15,40,-Target_A,5,12);
					else                 	OLED_ShowString(0,40,"TA+"),
																OLED_ShowNumber(15,40, Target_A,5,12); 				
					if( Encoder_A<0)		  OLED_ShowString(80,40,"EA-"),
																OLED_ShowNumber(95,40,-Encoder_A,4,12);
					else                 	OLED_ShowString(80,40,"EA+"),
																OLED_ShowNumber(95,40, Encoder_A,4,12);
											OLED_ShowString(48,50,".");
											OLED_ShowString(70,50,"V");
											OLED_ShowNumber(35,50,Voltage/100,2,12);
											OLED_ShowNumber(58,50,Voltage%100,2,12);
 if(Voltage%100<10) 	OLED_ShowNumber(52,50,0,2,12);
										
											if(Flag_Stop==0)
											OLED_ShowString(103,50,"O-N");
											if(Flag_Stop==1)
											OLED_ShowString(103,50,"OFF");
											OLED_ShowString(0,50,"ELE");
											OLED_Refresh_Gram();	//刷新

}


void OLED_DrawPoint_Shu(u8 x,u8 y,u8 t)
{ 
	 u8 i=0;
  OLED_DrawPoint(x,y,t);
	OLED_DrawPoint(x,y,t);
	  for(i = 0;i<8; i++)
  {
      OLED_DrawPoint(x,y+i,t);
  }
}



