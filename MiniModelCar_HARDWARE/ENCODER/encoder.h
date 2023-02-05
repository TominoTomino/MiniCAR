#ifndef __ENCODER_H
#define __ENCODER_H
#include <sys.h>	 
  /**************************************************************************
项目：MiniModelCar
说明：标准两驱模型智能小车
**************************************************************************/
#define ENCODER_TIM_PERIOD (u16)(65535)   //不可大于65535 因为F103的定时器是16位的。

void Encoder_Init_TIM2(void);
void Encoder_Init_TIM8(void);
int Read_Encoder(u8 TIMX);


void TIM2_IRQHandler(void);
void TIM8_IRQHandler(void);

#endif
