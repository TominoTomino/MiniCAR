#ifndef __DELAY_H
#define __DELAY_H 			   
#include "sys.h"  
  /**************************************************************************
��Ŀ��MiniModelCar
˵������׼����ģ������С��
**************************************************************************/

/*
1.����ԭ�ӳ�ʼ��ʹ��misc.c��Ŀ⺯��SysTick_CLKSourceConfig(uint32_t SysTick_CLKSource)��ʼ��systickʱ��ԴΪ
9MHz���ܵ���ʱ��ֵ��24λLOAD�Ĵ�������ʱʱ�������ƣ����0xFFFFFF/9=1864135us��SysTick-
>LOAD=nus*fac_us-1=1864134us����0��ʼ; ͬ��0xFFFFFF/9/1000=1864ms��SysTick->LOAD=nms*fac_ms-1=1863ms
*/
//24λLOAD�Ĵ��������ʱ1864ms������ԭ�Ӳ���
void delay_init(void);
void delay_ms(u16 nms);
void delay_us(u32 nus);


/*
Ұ��û�г�ʼ��������systick.h��ֱ�ӵ�����core_cm3.h�������п⺯��SysTick_Config��Ĭ��systickʱ��ԴΪ72MHz��
����us��ms��ʱ�����зֱ������SysTick_Config����Load�Ĵ���ֵ�ֱ�Ϊ72(1us����72000��1ms����ͨ��ѭ��ʵ��ʱ
����ʱ��ֵ�������ơ�
*/
//��ʱ�������ƣ�Ұ�𲿷�
void SysTick_Delay_us(uint32_t us);
void SysTick_Delay_ms(uint32_t ms);


/*
С������ɿ�init��ʼ������ͨ��ֱ�����üĴ���ctrl��load��������ϵͳʱ��Դ72MHz���жϺ�LOAD�Ĵ���ֵΪ72��
1us��,Ȼ��ͨ��ÿ�ж�һ�Σ�ʱ���1�ķ�ʽʵ����ʱus��ms����ʱʱ�䲻������
*/
void SysTick_Init(void);
void fc_delay_us(u32 time);
void fc_delay_ms(u32 time);


#endif





























