#ifndef __TIMER_H
#define __TIMER_H
#include <sys.h>	 
  /**************************************************************************
项目：MiniModelCar
说明：标准两驱模型智能小车
**************************************************************************/
void TIM3_Cap_Init(u16 arr,u16 psc);
void Read_Distane(void);
int TIM3_IRQHandler(void);
void TIM8_Cap_Init(u16 arr, u16 psc);
void TIM8_CC_IRQHandler(void);
#endif
