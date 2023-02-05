#include "usartx.h"
  /**************************************************************************
项目：MiniModelCar
说明：标准两驱模型智能小车
**************************************************************************/
u8 Usart3_Receive;
/**************************实现函数**********************************************
*功    能:		usart发送一个字节
*********************************************************************************/
void usart3_send(u8 data)
{
	USART3->DR = data;
	while((USART3->SR&0x40)==0);	
}

/**************************实现函数**********************************************
*功    能:无线模块地址、信道参数
					串口3每发一帧数据前都必须先调用一次该函数
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

#define TX_BUF_LEN  512     /* 发送缓冲区容量，根据需要进行调整 */
uint8_t TxBuf[TX_BUF_LEN];  /* 发送缓冲区                       */
void MyPrintf(const char *__format, ...)
{
	int i;
  int len = strlen((const char*)TxBuf);
  va_list ap;
  va_start(ap, __format);
    
  /* 清空发送缓冲区 */
  memset(TxBuf, 0x0, TX_BUF_LEN);
    
  /* 填充发送缓冲区 */
  vsnprintf((char*)TxBuf, TX_BUF_LEN, (const char *)__format, ap);
  va_end(ap);

  
  /* 往串口发送数据 */
  for ( i = 0; i < len; i++)
  {
   while(USART_GetFlagStatus(USART3, USART_FLAG_TC)==RESET);    
   USART_SendData(USART3, TxBuf[i]);
  }
}


/**************************************************************************
函数功能：串口3初始化
入口参数： bound:波特率
返回  值：无
**************************************************************************/
void uart3_init(u32 bound)
{  	 
	  //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);// 需要使能AFIO时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);	//使能GPIO时钟
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);	//使能USART时钟
		GPIO_PinRemapConfig(GPIO_PartialRemap_USART3, ENABLE);
	//USART_TX  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; //Pc10
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
  GPIO_Init(GPIOC, &GPIO_InitStructure);
   
  //USART_RX	  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;//Pc11
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  //UsartNVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//子优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
   //USART 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART3, &USART_InitStructure);     //初始化串口3
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//开启串口接受中断
  USART_Cmd(USART3, ENABLE);                    //使能串口3 
}

/**************************************************************************
函数功能：串口3接收中断
入口参数：无
返回  值：无
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


