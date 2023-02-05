#ifndef __MOTOR_H
#define __MOTOR_H
#include <sys.h>	 
  /**************************************************************************
项目：MiniModelCar
说明：标准两驱模型智能小车
**************************************************************************/
#define PWMA1   TIM4->CCR2  
#define PWMA2   TIM4->CCR1 

#define PWMB1   TIM4->CCR4  
#define PWMB2   TIM4->CCR3

void MiniModelCar_PWM_Init(u16 arr,u16 psc);
#endif
