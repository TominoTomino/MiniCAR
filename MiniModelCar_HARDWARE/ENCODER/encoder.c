#include "encoder.h"
#include "stm32f10x_gpio.h"
  /**************************************************************************
项目：MiniModelCar
说明：标准两驱模型智能小车
**************************************************************************/

/**************************************************************************
函数功能：把TIM2初始化为编码器接口模式
入口参数：无
返回  值：无
**************************************************************************/
void Encoder_Init_TIM2(void)
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;  
  TIM_ICInitTypeDef TIM_ICInitStructure;  
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);// 需要使能AFIO时钟
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);//使能定时器2的时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);//使能PA端口时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);//使能PB端口时钟 
  GPIO_PinRemapConfig(GPIO_FullRemap_TIM2, ENABLE);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;  //端口配置
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //浮空输入
  GPIO_Init(GPIOA, &GPIO_InitStructure);                //根据设定参数初始化GPIOA
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3; //端口配置
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //浮空输入
  GPIO_Init(GPIOB, &GPIO_InitStructure);                //根据设定参数初始化GPIOB

  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_TimeBaseStructure.TIM_Prescaler = 0x0; // 预分频器 
  TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD; //设定计数器自动重装值
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//选择时钟分频：不分频
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;////TIM向上计数  
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
  TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);//使用编码器模式3
  TIM_ICStructInit(&TIM_ICInitStructure);
  TIM_ICInitStructure.TIM_ICFilter = 10;
  TIM_ICInit(TIM2, &TIM_ICInitStructure);
  TIM_ClearFlag(TIM2, TIM_FLAG_Update);//清除TIM的更新标志位
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
  //Reset counter
//	TIM_SetCounter(TIM2,10000);//设置TIMx 计数器寄存器值
  TIM_SetCounter(TIM2,0);
  TIM_Cmd(TIM2, ENABLE); 
}


/**************************************************************************
函数功能：把TIM8初始化为编码器接口模式
入口参数：无
返回  值：无
**************************************************************************/
void Encoder_Init_TIM8(void)  //重引脚PC6、PC7，默认TIM8_CH1、TIM8_CH2
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;  
  TIM_ICInitTypeDef TIM_ICInitStructure;  
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);//使能定时器8的时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);//使能PC端口时钟	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;	//端口配置
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //浮空输入
  GPIO_Init(GPIOC, &GPIO_InitStructure);					      //根据设定参数初始化GPIOC
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_TimeBaseStructure.TIM_Prescaler = 0x0; // 预分频器 
  TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD; //设定计数器自动重装值
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//选择时钟分频：不分频
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;////TIM向上计数  
  TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);
  TIM_EncoderInterfaceConfig(TIM8, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);//使用编码器模式3
  TIM_ICStructInit(&TIM_ICInitStructure);
  TIM_ICInitStructure.TIM_ICFilter = 10;
  TIM_ICInit(TIM8, &TIM_ICInitStructure);
  TIM_ClearFlag(TIM8, TIM_FLAG_Update);//清除TIM的更新标志位
  TIM_ITConfig(TIM8, TIM_IT_Update, ENABLE);
  TIM_SetCounter(TIM8,0);
  TIM_Cmd(TIM8, ENABLE); 
}


/**************************************************************************
函数功能：单位时间读取编码器计数
入口参数：定时器
返回  值：速度值
**************************************************************************/
int Read_Encoder(u8 TIMX)
{
    int Encoder_TIM;    
   switch(TIMX)
	 {	
     case 2:  Encoder_TIM= (short)TIM2 -> CNT;  TIM2 -> CNT=0;break;   
		 case 8:  Encoder_TIM= (short)TIM8 -> CNT;  TIM8 -> CNT=0;break;		
		 default:  Encoder_TIM=0;
	 }
		return Encoder_TIM;
}


/**************************************************************************
函数功能：TIM2中断服务函数
入口参数：无
返回  值：无
**************************************************************************/
void TIM2_IRQHandler(void)
{ 		    		  			    
	if(TIM2->SR&0X0001)//溢出中断
	{    				   				     	    	
	}				   
	TIM2->SR&=~(1<<0);//清除中断标志位 	    
}

/**************************************************************************
函数功能：TIM8中断服务函数
入口参数：无
返回  值：无
**************************************************************************/
void TIM8_IRQHandler(void)
{ 		    		  			    
	if(TIM8->SR&0X0001)//溢出中断
	{    				   				     	    	
	}				   
	TIM8->SR&=~(1<<0);//清除中断标志位 	    
}
