#include "timer.h"
  /**************************************************************************
项目：MiniModelCar
说明：标准两驱模型智能小车
**************************************************************************/
/**************************************************************************
函数功能：定时器3通道3输入捕获初始化
入口参数：入口参数：arr：自动重装值  psc：时钟预分频数 
返回  值：无
**************************************************************************/
void TIM3_Cap_Init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	
	TIM_TimeBaseInitStruct.TIM_Period = arr;     //重装载值
	TIM_TimeBaseInitStruct.TIM_Prescaler = psc;  //预分频系数
	TIM_TimeBaseInitStruct.TIM_ClockDivision =0; //时钟分割
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up; //TIM向上计数模式
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStruct);
	
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);  //使能定时器中断
	
	NVIC_InitStruct.NVIC_IRQChannel = TIM3_IRQn;   //使能外部中断通道
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;   //使能外部中断通道
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1; //抢占优先级1
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 3;    //响应优先级3
	NVIC_Init(&NVIC_InitStruct);

	TIM_Cmd(TIM3,ENABLE);	  //使能定时器3
							 
}


