#include "motor.h"
  /**************************************************************************
项目：MiniModelCar
说明：标准两驱模型智能小车
**************************************************************************/
//决定输出频率和占空比主要由这三个参数（arr、psc、TIM_Pulse）决定的
void MiniModelCar_PWM_Init(u16 arr,u16 psc)
{		 		
		    /* 初始化结构体定义 */
    GPIO_InitTypeDef GPIO_InitStructure;            //GPIO初始化结构体
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure; //TIM时间基数初始化结构体
    TIM_OCInitTypeDef  TIM_OCInitStructure;         //TIM输出比较功能结构体
    
    /* 时钟线(RCC)设置 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE);  //使能GPIOB外设时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);    //使能TIM4外设时钟

    
    /* 端口(GPIO)设置 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;//PB6、PB7、PB8、PB9
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //复用推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//速度2MHz
    GPIO_Init(GPIOB, &GPIO_InitStructure);          //初始化GPIOB
    
    /* TIM时间基数初始化 */
    TIM_TimeBaseStructure.TIM_Period = arr;     //计数器计数arr次则溢出  ， 定时器周期(自动从装载寄存器ARR的值)
    TIM_TimeBaseStructure.TIM_Prescaler =psc;   //预分频系数  ，预分频值
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;//设置时钟分割
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//TIM向上计数模式
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //初始化TIMx的时钟基数单位
    
    /* TIM外设初始化 */
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;//选择定时器模式：TIM脉冲宽度调制模式2
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;//比较输出使能
    TIM_OCInitStructure.TIM_Pulse = 0; //设置CCR为0  , 脉宽值
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;//输出极性:TIM输出比较极性高
    TIM_OC1Init(TIM4, &TIM_OCInitStructure);    //初始化CH1
    TIM_OC2Init(TIM4, &TIM_OCInitStructure);    //初始化CH2
    TIM_OC3Init(TIM4, &TIM_OCInitStructure);    //初始化CH3
    TIM_OC4Init(TIM4, &TIM_OCInitStructure);    //初始化CH4
    
    /* 使能TIM */
    TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);   //CH1预装载使能 
    TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);   //CH2预装载使能   
    TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);   //CH3预装载使能
    TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);   //CH4预装载使能
    TIM_ARRPreloadConfig(TIM4, ENABLE);                 //使能TIMx在ARR上的预装载寄存器
    TIM_Cmd(TIM4, ENABLE);                              //使能TIM4
 
} 

