#ifndef __DELAY_H
#define __DELAY_H 			   
#include "sys.h"  
  /**************************************************************************
项目：MiniModelCar
说明：标准两驱模型智能小车
**************************************************************************/

/*
1.正点原子初始化使用misc.c里的库函数SysTick_CLKSourceConfig(uint32_t SysTick_CLKSource)初始化systick时钟源为
9MHz，总的延时赋值给24位LOAD寄存器，延时时间受限制，最大0xFFFFFF/9=1864135us，SysTick-
>LOAD=nus*fac_us-1=1864134us，从0开始; 同理0xFFFFFF/9/1000=1864ms，SysTick->LOAD=nms*fac_ms-1=1863ms
*/
//24位LOAD寄存器最大延时1864ms，正点原子部分
void delay_init(void);
void delay_ms(u16 nms);
void delay_us(u32 nus);


/*
野火没有初始化函数，systick.h中直接调用了core_cm3.h（里面有库函数SysTick_Config，默认systick时钟源为72MHz）
，在us和ms延时函数中分别调用了SysTick_Config，设Load寄存器值分别为72(1us）和72000（1ms），通过循环实现时
间延时，值不受限制。
*/
//延时不受限制，野火部分
void SysTick_Delay_us(uint32_t us);
void SysTick_Delay_ms(uint32_t ms);


/*
小马四轴飞控init初始化函数通过直接配置寄存器ctrl和load，设置了系统时钟源72MHz，中断和LOAD寄存器值为72（
1us）,然后通过每中断一次，时间减1的方式实现延时us和ms，延时时间不受限制
*/
void SysTick_Init(void);
void fc_delay_us(u32 time);
void fc_delay_ms(u32 time);


#endif





























