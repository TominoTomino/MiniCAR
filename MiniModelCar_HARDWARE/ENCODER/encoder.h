#ifndef __ENCODER_H
#define __ENCODER_H
#include <sys.h>	 
  /**************************************************************************
��Ŀ��MiniModelCar
˵������׼����ģ������С��
**************************************************************************/
#define ENCODER_TIM_PERIOD (u16)(65535)   //���ɴ���65535 ��ΪF103�Ķ�ʱ����16λ�ġ�

void Encoder_Init_TIM2(void);
void Encoder_Init_TIM8(void);
int Read_Encoder(u8 TIMX);


void TIM2_IRQHandler(void);
void TIM8_IRQHandler(void);

#endif
