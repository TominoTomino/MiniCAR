#ifndef __LED_H
#define __LED_H
#include "sys.h"
  /**************************************************************************
项目：MiniModelCar
说明：标准两驱模型智能小车
**************************************************************************/
//LED 端口定义
#define LED PBout(13)  
#define CHARGE PBout(15)  //充电控制
void LED_Init(void);  //初始化
void Led_Flash(u16 time);
#endif
