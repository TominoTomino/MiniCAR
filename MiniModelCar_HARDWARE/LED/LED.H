#ifndef __LED_H
#define __LED_H
#include "sys.h"
  /**************************************************************************
��Ŀ��MiniModelCar
˵������׼����ģ������С��
**************************************************************************/
//LED �˿ڶ���
#define LED PBout(13)  
#define CHARGE PBout(15)  //������
void LED_Init(void);  //��ʼ��
void Led_Flash(u16 time);
#endif