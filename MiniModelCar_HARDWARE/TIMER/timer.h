#ifndef __TIMER_H
#define __TIMER_H
#include <sys.h>	 
  /**************************************************************************
��Ŀ��MiniModelCar
˵������׼����ģ������С��
**************************************************************************/
void TIM3_Cap_Init(u16 arr,u16 psc);
void Read_Distane(void);
int TIM3_IRQHandler(void);
void TIM8_Cap_Init(u16 arr, u16 psc);
void TIM8_CC_IRQHandler(void);
#endif
