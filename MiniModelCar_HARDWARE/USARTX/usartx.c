#include "usartx.h"
  /**************************************************************************
��Ŀ��MiniModelCar
˵������׼����ģ������С��
**************************************************************************/
u8 Usart3_Receive;
/**************************ʵ�ֺ���**********************************************
*��    ��:		usart����һ���ֽ�
*********************************************************************************/
void usart3_send(u8 data)
{
	USART3->DR = data;
	while((USART3->SR&0x40)==0);	
}

/**************************ʵ�ֺ���**********************************************
*��    ��:����ģ���ַ���ŵ�����
					����3ÿ��һ֡����ǰ�������ȵ���һ�θú���
*********************************************************************************/
void usart3_send_head(void)
{
	USART3->DR = 0x00;
	while((USART3->SR&0x40)==0);	
	USART3->DR = 0x00;
	while((USART3->SR&0x40)==0);	
	USART3->DR = 0x17;
	while((USART3->SR&0x40)==0);	
}


void UART_PutChar(USART_TypeDef* USARTx, uint8_t Data)  
{  	 
    USART_SendData(USARTx, Data);  
    while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET){}  
} 
 
 
void UART_PutStr (USART_TypeDef* USARTx, uint8_t *str)    
{    
    while (0 != *str)    
    {    
        UART_PutChar(USARTx, *str);    
        str++;    
    }    
}
#include "stdarg.h"

#define TX_BUF_LEN  512     /* ���ͻ�����������������Ҫ���е��� */
uint8_t TxBuf[TX_BUF_LEN];  /* ���ͻ�����                       */
void MyPrintf(const char *__format, ...)
{
	int i;
  int len = strlen((const char*)TxBuf);
  va_list ap;
  va_start(ap, __format);
    
  /* ��շ��ͻ����� */
  memset(TxBuf, 0x0, TX_BUF_LEN);
    
  /* ��䷢�ͻ����� */
  vsnprintf((char*)TxBuf, TX_BUF_LEN, (const char *)__format, ap);
  va_end(ap);

  
  /* �����ڷ������� */
  for ( i = 0; i < len; i++)
  {
   while(USART_GetFlagStatus(USART3, USART_FLAG_TC)==RESET);    
   USART_SendData(USART3, TxBuf[i]);
  }
}


/**************************************************************************
�������ܣ�����3��ʼ��
��ڲ����� bound:������
����  ֵ����
**************************************************************************/
void uart3_init(u32 bound)
{  	 
	  //GPIO�˿�����
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);// ��Ҫʹ��AFIOʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);	//ʹ��GPIOʱ��
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);	//ʹ��USARTʱ��
		GPIO_PinRemapConfig(GPIO_PartialRemap_USART3, ENABLE);
	//USART_TX  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; //Pc10
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
  GPIO_Init(GPIOC, &GPIO_InitStructure);
   
  //USART_RX	  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;//Pc11
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  //UsartNVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//�����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
   //USART ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
  USART_Init(USART3, &USART_InitStructure);     //��ʼ������3
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//�������ڽ����ж�
  USART_Cmd(USART3, ENABLE);                    //ʹ�ܴ���3 
}

/**************************************************************************
�������ܣ�����3�����ж�
��ڲ�������
����  ֵ����
**************************************************************************/
void USART3_IRQHandler(void)
{	
	uint8_t ucTemp;
	if(USART_GetITStatus(USART3,USART_IT_RXNE)!=RESET)
	{		
		ucTemp = USART_ReceiveData(USART3);
    UrxSt = ucTemp; 
		
		//usart3_send_head();	
		usart3_send(0x00);
		usart3_send(0x00);
		usart3_send(0x17);
		usart3_send(UrxSt);
		if(UrxSt == 0x88)
		{
			
		}


		//if(UrxSt==0x05){ Auto_Back_Flag=1;}else if(UrxSt==0xff){Auto_Back_Flag=0;}

		
		USART_ClearITPendingBit(USART3 , USART_IT_RXNE);
	}	 							 
} 


