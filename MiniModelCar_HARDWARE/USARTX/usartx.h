#ifndef __USRAT3_H
#define __USRAT3_H 
#include "sys.h"	  	
  /**************************************************************************
��Ŀ��MiniModelCar
˵������׼����ģ������С��
**************************************************************************/
void usart3_send(u8 data);
void uart3_init(u32 bound);
void USART3_IRQHandler(void);

void usart3_send_head(void);
void UART_PutChar(USART_TypeDef* USARTx, uint8_t Data);  
void UART_PutStr (USART_TypeDef* USARTx, uint8_t *str);
void MyPrintf(const char *__format, ...);
#endif

