#ifndef __USART_H
#define __USART_H

#include <stdio.h>
#include <stdarg.h>

#include "stm32f10x.h"

#define USART1_REC_MAXLEN 256	//最大接收数据长度

void USART1_Init_Config(u32 bound);
int  UART1_Printf(const char *fmt, ...);
void UART1_SendString(char* s);
void USART2_Init_Config(u32 bound);
void UART2_SendString(char* s);
void UART2_SendU8Array(u8* s,int len);
void UART1_SendLR(void);
void UART2_SendLR(void);
void UART2_Send_Command(char* s);
u8 UART2_Send_AT_Command(char *b,char *a,u8 wait_time,u32 s);
void UART2_Send_Command_END(char* s); 
u8 UART2_Send_AT_Command_End(char *b,char *a,u8 wait_time,u32 s);

//串口1发送回车换行
#define UART1_SendLR() UART1_SendString("\r\n")

//串口2发送回车换行
#define UART2_SendLR() UART2_SendString("\r\n")
//											
#endif


