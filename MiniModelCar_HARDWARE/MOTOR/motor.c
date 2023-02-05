#include "motor.h"
  /**************************************************************************
��Ŀ��MiniModelCar
˵������׼����ģ������С��
**************************************************************************/
//�������Ƶ�ʺ�ռ�ձ���Ҫ��������������arr��psc��TIM_Pulse��������
void MiniModelCar_PWM_Init(u16 arr,u16 psc)
{		 		
		    /* ��ʼ���ṹ�嶨�� */
    GPIO_InitTypeDef GPIO_InitStructure;            //GPIO��ʼ���ṹ��
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure; //TIMʱ�������ʼ���ṹ��
    TIM_OCInitTypeDef  TIM_OCInitStructure;         //TIM����ȽϹ��ܽṹ��
    
    /* ʱ����(RCC)���� */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE);  //ʹ��GPIOB����ʱ��
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);    //ʹ��TIM4����ʱ��

    
    /* �˿�(GPIO)���� */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;//PB6��PB7��PB8��PB9
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //�����������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//�ٶ�2MHz
    GPIO_Init(GPIOB, &GPIO_InitStructure);          //��ʼ��GPIOB
    
    /* TIMʱ�������ʼ�� */
    TIM_TimeBaseStructure.TIM_Period = arr;     //����������arr�������  �� ��ʱ������(�Զ���װ�ؼĴ���ARR��ֵ)
    TIM_TimeBaseStructure.TIM_Prescaler =psc;   //Ԥ��Ƶϵ��  ��Ԥ��Ƶֵ
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;//����ʱ�ӷָ�
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//TIM���ϼ���ģʽ
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //��ʼ��TIMx��ʱ�ӻ�����λ
    
    /* TIM�����ʼ�� */
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;//ѡ��ʱ��ģʽ��TIM�����ȵ���ģʽ2
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;//�Ƚ����ʹ��
    TIM_OCInitStructure.TIM_Pulse = 0; //����CCRΪ0  , ����ֵ
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;//�������:TIM����Ƚϼ��Ը�
    TIM_OC1Init(TIM4, &TIM_OCInitStructure);    //��ʼ��CH1
    TIM_OC2Init(TIM4, &TIM_OCInitStructure);    //��ʼ��CH2
    TIM_OC3Init(TIM4, &TIM_OCInitStructure);    //��ʼ��CH3
    TIM_OC4Init(TIM4, &TIM_OCInitStructure);    //��ʼ��CH4
    
    /* ʹ��TIM */
    TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);   //CH1Ԥװ��ʹ�� 
    TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);   //CH2Ԥװ��ʹ��   
    TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);   //CH3Ԥװ��ʹ��
    TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);   //CH4Ԥװ��ʹ��
    TIM_ARRPreloadConfig(TIM4, ENABLE);                 //ʹ��TIMx��ARR�ϵ�Ԥװ�ؼĴ���
    TIM_Cmd(TIM4, ENABLE);                              //ʹ��TIM4
 
} 

