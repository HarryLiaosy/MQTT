/**********************************************************************************
 * 文件名  ：usart.c
 * 描述    ：usart1 应用函数库          
 * 实验平台：NiRen_TwoHeart系统板
 * 硬件连接：TXD(PB9)  -> 外部串口RXD     
 *           RXD(PB10) -> 外部串口TXD      
 *           GND	   -> 外部串口GND 
 * 库版本  ：ST_v3.5
**********************************************************************************/

#include "usart.h"
#include "string.h"
#include "SysTick.h"
	
vu8 Usart1_R_Buff[USART1_REC_MAXLEN];	//串口1数据接收缓冲区 
vu8 Usart1_R_State=0;					//串口1接收状态
vu16 Usart1_R_Count=0;					//当前接收数据的字节数 	  
extern void clear_uart2_recv_buf(void);
/*******************************************************************************
* 函数名  : USART1_Init_Config
* 描述    : USART1初始化配置
* 输入    : bound：波特率(常用：2400、4800、9600、19200、38400、115200等)
* 输出    : 无
* 返回    : 无 
* 说明    : 无
*******************************************************************************/
void USART1_Init_Config(u32 bound)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;

	/*使能USART1和GPIOA外设时钟*/  
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);	

	/*复位串口1*/
	USART_DeInit(USART1);  

	/*USART1_GPIO初始化设置*/ 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;			//USART1_TXD(PA.9)     
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		//复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//设置引脚输出最大速率为50MHz
	GPIO_Init(GPIOA, &GPIO_InitStructure);				//调用库函数中的GPIO初始化函数，初始化USART1_TXD(PA.9)  


	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;				//USART1_RXD(PA.10)
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	//浮空输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);					//调用库函数中的GPIO初始化函数，初始化USART1_RXD(PA.10)


	/*USART1 初始化设置*/
	USART_InitStructure.USART_BaudRate = bound;										//设置波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;						//8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;							//1个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;								//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					//工作模式设置为收发模式
	USART_Init(USART1, &USART_InitStructure);										//初始化串口1

	/*Usart1 NVIC配置*/
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;	//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//从优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);							//根据指定的参数初始化VIC寄存器 

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);			//使能串口1接收中断

	USART_Cmd(USART1, ENABLE);                    			//使能串口 
	USART_ClearFlag(USART1, USART_FLAG_TC);					//清除发送完成标志
}


/*******************************************************************************
* 函数名  : UART1_Printf
* 描述    : 模拟实现printf函数，方便调试
* 输入    : 
* 输出    : 无
* 返回    : 无 
* 说明    : 无
*******************************************************************************/
int UART1_Printf(const char *fmt, ...){
	  int printed; 
	  va_list args;  	
	  char send_buff[USART1_REC_MAXLEN];  
    memset(send_buff,'\0',USART1_REC_MAXLEN);
    va_start(args, fmt);  
    printed = vsprintf(send_buff, fmt, args);  
    va_end(args);  
    UART1_SendString(send_buff);  
	
    return printed; 
}

/*******************************************************************************
* 函数名  : UART1_SendString
* 描述    : USART1发送字符串
* 输入    : *s字符串指针
* 输出    : 无
* 返回    : 无 
* 说明    : 无
*******************************************************************************/
void UART1_SendString(char* s)
{
	while(*s)//检测字符串结束符
	{
		while(USART_GetFlagStatus(USART1, USART_FLAG_TC)==RESET); 
		USART_SendData(USART1 ,*s++);//发送当前字符
	}
}

/*******************************************************************************
* 函数名  : USART1_IRQHandler
* 描述    : 串口1中断服务程序
* 输入    : 无
* 返回    : 无 
* 说明    : 1)、只启动了USART1中断接收，未启动USART1中断发送。
*           2)、接收到0x0d 0x0a(回车、"\r\n")代表帧数据接收完成
*******************************************************************************/
void USART1_IRQHandler(void)                	
{

} 	

/*******************************************************************************
* 函数名  : USART1_Init_Config
* 描述    : USART1初始化配置
* 输入    : bound：波特率(常用：2400、4800、9600、19200、38400、115200等)
* 输出    : 无
* 返回    : 无 
* 说明    : 无
*******************************************************************************/
void USART2_Init_Config(u32 bound)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;

	/*使能USART2外设时钟*/  
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	
	/*复位串口2*/
	USART_DeInit(USART2);  

	/*USART2_GPIO初始化设置*/ 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;			//USART2_TXD(PA.2)     
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		//复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//设置引脚输出最大速率为50MHz
	GPIO_Init(GPIOA, &GPIO_InitStructure);				//调用库函数中的GPIO初始化函数，初始化USART1_TXD(PA.9)  


	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;				//USART2_RXD(PA.3)
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	//浮空输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);					//调用库函数中的GPIO初始化函数，初始化USART1_RXD(PA.10)


	/*USART2 初始化设置*/
	USART_InitStructure.USART_BaudRate = bound;										//设置波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;						//8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;							//1个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;								//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					//工作模式设置为收发模式
	USART_Init(USART2, &USART_InitStructure);										//初始化串口2

	/*Usart1 NVIC配置*/
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;	//抢占优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//从优先级0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);							//根据指定的参数初始化VIC寄存器 

	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);			//使能串口2接收中断

	USART_Cmd(USART2, ENABLE);                    			//使能串口 
	USART_ClearFlag(USART2, USART_FLAG_TC);					//清除发送完成标志
}
/*******************************************************************************
* 函数名  : UART2_SendString
* 描述    : USART2发送字符串
* 输入    : *s字符串指针
* 输出    : 无
* 返回    : 无 
* 说明    : 无
*******************************************************************************/
void UART2_SendString(char* s)
{
	while(*s)//检测字符串结束符
	{
		while(USART_GetFlagStatus(USART2, USART_FLAG_TC)==RESET); 
		USART_SendData(USART2 ,*s++);//发送当前字符
	}
}

/*******************************************************************************
* 函数名  : UART2_SendU8Array
* 描述    : USART2发送长度为len的u8数组
* 输入    : *s字符串指针
* 输出    : 无
* 返回    : 无 
* 说明    : 无
*******************************************************************************/
void UART2_SendU8Array(u8* s,int len)
{
	int i = 0;
	for(i=0;i<len;++i)
	{
		while(USART_GetFlagStatus(USART2, USART_FLAG_TC)==RESET); 
		USART_SendData(USART2 ,s[i]);              //发送当前字符
	}
}

void UART2_Send_Command(char* s)
{
	// 每次发送命令前清空uart2的收数据的缓存
	clear_uart2_recv_buf(); 
	UART2_SendString(s); //发出字符串
	UART2_SendString("\r\n"); //再自动发送 \r\n两个字符
}

/*******************************************************************************
* 函数名 : Second_AT_Command
* 描述   : 发送AT指令函数
* 输入   : 发送数据的指针、希望收到的应答、发送等待时间(单位为秒)
* 输出   : 
* 返回   : 
* 注意   : 
*******************************************************************************/
extern u8 exist_in_uart2_recv_buf(char *a);

u8 UART2_Send_AT_Command(char *b,char *a,u8 wait_time,u32 s)         
{
	u8 i;
	i = 0;
	while(i < wait_time)    //如果没有找到 就继续再发一次指令 再进行查找目标字符串                
	{
		UART2_Send_Command(b);//串口2发送 b 字符串 他会自动发送\r\n  相当于发送了一个指令
		delays(s); //等待一定时间 传50的话就是 50*20ms = 1秒
		if(exist_in_uart2_recv_buf(a))            //查找需要应答的字符串 a
		{
			return 1;
		}
		i++;
	}
	
	return 0;
}

void UART2_Send_Command_END(char* s)
{
	clear_uart2_recv_buf(); //清空接收数据的buffer
	UART2_SendString(s);    //发出字符串
}

u8 UART2_Send_AT_Command_End(char *b,char *a,u8 wait_time,u32 s)         
{
	u8 i;
	i = 0;
	while(i < wait_time)    //如果没有找到 就继续再发一次指令 再进行查找目标字符串                
	{
		UART2_Send_Command_END(b);              //串口2发送 b 字符串 这里不发送\r\n 
		delays(s);                               //等待s秒
		if( exist_in_uart2_recv_buf(a) )            //查找需要应答的字符串 a
		{
			return 1;
		}
		i++;
		
	}
	
	return 0;
}



