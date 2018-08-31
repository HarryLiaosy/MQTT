










/**********************************************************************************
   STM32F103C8T6  	 
 * 硬件连接说明
	 使用单片串口2与GPRS模块通信  注：使用串口2可以避免下载和通信不会冲突
	 STM32      GPRS模块
	 打板子接线方法:               mini板子接线方法:
   PA3 (RXD2)->U_RX               PA3 (RXD2)->U_TX
	 PA2 (TXD2)->U_TX               PA2 (TXD2)->U_RX
	 GND	   ->GND                GND	   ->GND
	 
	 PA9(TXD1)--->调试信息端口
	 PA10(RXD1)-->调试信息端口
设计：
(1)LED0-7设置的引脚为:PB4-7 PB12-15
(2)KEY0-3设置的引脚为:PA4-7

果云技术:杜工
  
**********************************************************************************/
#include "stm32f10x.h"
#include "usart.h"
#include "Led.h"
#include "SysTick.h"
#include "timer.h"
#include "string.h"
#include "key.h"
#include "GA6_module_errors.h"

#include "MQTTPacket.h"

#include <stdio.h>

#define UART2_BUF_LEN 	    256         //串口2缓存长度
#define STABLE_TIMES        10          //等待系统上电后的稳定

// 定义MQTT传送数据buf的最大长度
#define MQTT_MAX_BUF_SIZE   256
// 定义MQTT服务器的ip和对于的端口，因为GA6模块tcp通信时，写入ip和端口是这个格式
// 所以就这么固定下来
#define MQTT_SERVER_IP_AND_PORT         "\"211.149.217.3\",1883"

/*************	本地常量声明	**************/

/*************  本地变量声明	**************/
//串口2接收数据的缓存
u8 uart2_recv_buf[UART2_BUF_LEN];
// 指示当前uart2_recv_buf中的可用的第一个index，用来接收串口消息
u16 first_index_to_recv = 0;

// 指示当前要读取串口消息的当前index，这里的21是因为给服务器发送消息后，GA6模块通过串口
// 发回来的数据前面有21个数据，所以读取时默认从第22个数据开始读取
static u16 first_index_to_read = 21;

/*************	本地函数声明	**************/
void clear_uart2_recv_buf(void);              //清除串口2接收缓存
u8 Wait_CREG(u8 query_times);                 //等待模块注册成功
// 检查是否在clear_uart2_recv_buf存在要查找的字符串
u8 exist_in_uart2_recv_buf(char *a);
void dump_uart2_recv_buf(void);

/*************  外部函数和变量声明*****************/


/***************************************************************
注：当然你可以返回其他值，来确定到底是哪一步发送指令出现失败了。
****************************************************************/
int transport_sendPacketBuffer(const char*server_ip_and_port,unsigned char* buf, int buflen)
{
	u8 ret;
	char end_char[2];
	char connect_server_ip_port_cmd[56];
	
	// 因为每次读取消息之前都要设置这个值，所以干脆放到这个函数中，以便下次读取
	// 尽量不去动MQTT中的代码
	first_index_to_read = 21;
	
	end_char[0] = 0x1A;//结束字符
	end_char[1] = '\0';
	
	ret = UART2_Send_AT_Command("AT+CIPSTATUS","CONNECT OK",3,1);//查询连接状态
	//UART1_Printf("0000 ret %d\n", ret);
	if(ret == 1)//说明服务器处于连接状态
	{
		ret = UART2_Send_AT_Command("AT+CIPSEND",">",3,1);//开发发送数据
		//UART1_Printf("1111 ret %d\n", ret);
		if(ret == 0)
		{
			return AT_CIPSEND_ERROR;
		}
		
		UART2_SendU8Array(buf,buflen);
		delays(2);
		ret = UART2_Send_AT_Command_End(end_char,"SEND OK",4,1);//发送结束符，等待返回ok,等待5S发一次，因为发送数据时间可能较长
		//UART1_Printf("2222 ret %d\n", ret);
		if(ret == 0)
		{
			return END_CHAR_ERROR;
		}
		//clear_uart2_recv_buf();
		return 1;
	}
	
	ret = UART2_Send_AT_Command("AT+CGATT=1","OK",2,1);//附着网络
	//UART1_Printf("3333 ret %d\n", ret);
	if(ret == 0)
	{
		return AT_CGATT_ERROR;
	}
	
	ret = UART2_Send_AT_Command("AT+CGACT=1,1","OK",2,1);//激活网络
	//UART1_Printf("0000 ret %d\n", ret);
	if(ret == 0)
	{
		return AT_CGATT1_ERROR;
	}

	memset(connect_server_ip_port_cmd,'\0',56);
	strcpy(connect_server_ip_port_cmd,"AT+CIPSTART=\"TCP\",");
	strcat(connect_server_ip_port_cmd,server_ip_and_port);
	
	ret = UART2_Send_AT_Command(connect_server_ip_port_cmd,"CONNECT OK",4,1);//连接服务器
	//UART1_Printf("0000 ret %d\n", ret);
	if(ret == 0)
	{
		return AT_CIPSTART_ERROR;
	}
	
	ret = UART2_Send_AT_Command("AT+CIPSEND",">",3,1);//开发发送数据
	//UART1_Printf("0000 ret %d\n", ret);
	if(ret == 0)
	{
		return AT_CIPSEND_ERROR;
	}
	
	UART2_SendU8Array(buf,buflen);
	delays(2);
	ret = UART2_Send_AT_Command_End(end_char,"SEND OK",4,1);//发送结束符，等待返回ok,等待5S发一次，因为发送数据时间可能较长
  	if(ret == 0)
	{
		 return END_CHAR_ERROR;
	}
	
	return 1;
}

/**
* 为了尽量不去动MQTT中的代码，这个收的函数这么实现，保留这个函实的定义
*/
int transport_getdata(unsigned char*buf, int count)
{
	memcpy(buf,&uart2_recv_buf[first_index_to_read],count);
	// 更新读取的序号，以便下次读取
	first_index_to_read += count;
	return count;
}


// 检查GA6模块的状态
int check_ga6_status(void)
{
	int ret;
	char esc_char[2];
	
	esc_char[0] = 0x1B;//退出字符
	esc_char[1] = '\0';
	
	ret = UART2_Send_AT_Command("AT","OK",5,1);//测试通信是否成功
	if(ret == 0)
	{
		UART2_SendString(esc_char);   //万一进入>状态，那么久发送一个ESC字符
		return COMMUNITE_ERROR;
	}
	
	ret = UART2_Send_AT_Command("AT+CPIN?","READY",2,1);//查询卡是否插上
	if(ret == 0)
	{
		return NO_SIM_CARD_ERROR;
	}
	
	ret = Wait_CREG(3);//查询卡是否注册到网络
	if(ret == 0)
	{
		return SIM_CARD_NO_REG_ERROR;
	}
    
  ret = UART2_Send_AT_Command("ATE0","OK",2,1);//关闭回显功能
	if(ret == 0)
	{
		return EAT0_ERROR;
	}
    
	return 1;
}


/*******************************************************************************
* 函数名 : main 
* 描述   : 主函数
* 输入   : 
* 输出   : 
* 返回   : 
* 注意   : 串口2负责与GA6模块通信，串口1用于串口调试，
*******************************************************************************/
int main(void)
{
	u16 key_value= 0, retry_count = 5;
	unsigned short submsgid;
	unsigned char buf[MQTT_MAX_BUF_SIZE];
	unsigned char sessionPresent, connack_rc;
	const char* payload = "mypayload";
	int payloadlen = strlen(payload);
	int ret = 0,len = 0,req_qos = 0,msgid = 1,loop = 1,granted_qos,subcount;
	
	MQTTString topicString = MQTTString_initializer;
	MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
	
	SysTick_Init_Config();   //系统滴答时钟初始化
	GPIO_Config();           //GPIO初始化
	Key_GPIO_Config();
	USART2_Init_Config(115200);  //串口2初始化
	Timer2_Init_Config();        //定时器2初始化
	
	USART1_Init_Config(115200);//UART1用作串口调试信息
	
	UART1_SendString("系统启动.......................\r\n");
	
	//等待系统上电后的稳定
	delays(STABLE_TIMES);
	
	data.clientID.cstring = "me";
	data.keepAliveInterval = 20;
	data.cleansession = 1;
	data.username.cstring = "testuser";
	data.password.cstring = "testpassword";
	
	len = MQTTSerialize_connect(buf, MQTT_MAX_BUF_SIZE, &data);
	UART1_Printf("1111 len %d\n", len);
	ret = transport_sendPacketBuffer(MQTT_SERVER_IP_AND_PORT, buf, len);
	if( ret != 1 ){
		UART1_Printf("1111 transport_sendPacketBuffer Error %d\n", ret);
		return -1;
	}
	/* wait for connack */
	if (MQTTPacket_read(buf, MQTT_MAX_BUF_SIZE, transport_getdata) != CONNACK)
	{
		UART1_Printf("MQTTPacket_read != CONNACK\n");
		return -1;
	}
	if (MQTTDeserialize_connack(&sessionPresent, &connack_rc, buf, MQTT_MAX_BUF_SIZE) != 1 || connack_rc != 0)
	{
		UART1_Printf("Unable to connect, return code %d\n", connack_rc);
		return -1;
	}
	topicString.cstring = "substopic";
	len = MQTTSerialize_subscribe(buf, MQTT_MAX_BUF_SIZE, 0, msgid, 1, &topicString, &req_qos);
	UART1_Printf("2222 len %d\n", len);
	ret = transport_sendPacketBuffer(MQTT_SERVER_IP_AND_PORT, buf, len);
	if( ret != 1 ){
		UART1_Printf("2222 transport_sendPacketBuffer Error %d\n", ret);
		return -1;
	}
	
	if (MQTTPacket_read(buf, MQTT_MAX_BUF_SIZE, transport_getdata) != SUBACK) 	/* wait for suback */
	{
		UART1_Printf("MQTTPacket_read != SUBACK\n");
		return -1;
	}
	
	MQTTDeserialize_suback(&submsgid, 1, &subcount, &granted_qos, buf, MQTT_MAX_BUF_SIZE);
	if (granted_qos != 0)
	{
		UART1_Printf("granted qos != 0, %d\n", granted_qos);
		return -1;
	}
	
	/* loop getting msgs on subscribed topic */
	topicString.cstring = "pubtopic";
	
	while( loop ){ 
		
		key_value = Key_Down_Scan();
		// add by james_xie 20171221 ,for debug
		UART1_Printf("key_value %d\r\n",key_value);
		// 按键KEY1即PA5被按下则跳出循环
		if( key_value == 0x0001 << 5 ){
			UART1_Printf("Key1 is pressed down , exit \r\n");
			loop = 0;
		}
		/* transport_getdata() has a built-in 1 second timeout,
		your mileage will vary */
		if (MQTTPacket_read(buf, MQTT_MAX_BUF_SIZE, transport_getdata) == PUBLISH)
		{
			int qos,payloadlen_in;
			unsigned char dup,retained;
			unsigned short msgid;
			unsigned char* payload_in;
			MQTTString receivedTopic;

			MQTTDeserialize_publish(&dup, &qos, &retained, &msgid, &receivedTopic,
					&payload_in, &payloadlen_in, buf, MQTT_MAX_BUF_SIZE);
			UART1_Printf("message arrived %.*s\n", payloadlen_in, payload_in);
		}

		UART1_Printf("publishing reading\n");
		len = MQTTSerialize_publish(buf, MQTT_MAX_BUF_SIZE, 0, 0, 0, 0, topicString, (unsigned char*)payload, payloadlen);
		UART1_Printf("33333 len %d\n", len);
		// 重复发送5次，如果5次都不成功，则退出程序
		while( retry_count > 0 ){
			ret = transport_sendPacketBuffer(MQTT_SERVER_IP_AND_PORT, buf, len);
			// 如果发送成功则跳出循环
			if( ret == 1 ){
				break;
			}
			delays(1);
		}
		if( !retry_count && ret != 1 ){
			 UART1_Printf("3333 transport_sendPacketBuffer Error %d\n", ret);
			 break;
		}
		// 每次发送之后，等两秒
		delays(2);
	}
	
	UART1_Printf("disconnecting\n");
	len = MQTTSerialize_disconnect(buf, MQTT_MAX_BUF_SIZE);
	ret = transport_sendPacketBuffer(MQTT_SERVER_IP_AND_PORT, buf, len);
	if( ret != 1 ){
		UART1_Printf("4444 transport_sendPacketBuffer Error %d\n", ret);
		return -1;
	}
}

/*******************************************************************************
* 函数名  : USART2_IRQHandler
* 描述    : 串口2中断服务程序
* 输入    : 无
* 返回    : 无 
* 说明    : 
*******************************************************************************/
void USART2_IRQHandler(void)                	
{
	u8 Res=0;
	Res = USART_ReceiveData(USART2);
	uart2_recv_buf[first_index_to_recv] = Res;  	  //将接收到的字符串存到缓存中
	first_index_to_recv++;                	  //缓存指针向后移动
	if(first_index_to_recv > UART2_BUF_LEN)       	  //如果缓存满,将缓存指针指向缓存的首地址
	{
		first_index_to_recv = 0;
	}
} 	

/*******************************************************************************
* 函数名  : TIM2_IRQHandler
* 描述    : 定时器2中断断服务函数
* 输入    : 无
* 输出    : 无
* 返回    : 无 
* 说明    : 无
*******************************************************************************/
void TIM2_IRQHandler(void)   //TIM2中断
{
	static u8 flag =1;

	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)  //检查TIM2更新中断发生与否
	{
		
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);  //清除TIM2更新中断标志 
	
		if(flag)
		{
			//LED4_ON(); 
			flag=0;
		}
		else
		{
			//LED4_OFF(); 
			flag=1;
		}
	}	
}

/*******************************************************************************
* 函数名 : clear_uart2_recv_buf
* 描述   : 清除串口2缓存数据
* 输入   : 
* 输出   : 
* 返回   : 
* 注意   : 
*******************************************************************************/
void clear_uart2_recv_buf(void)
{
	u16 k;
	for(k=0;k<UART2_BUF_LEN;k++)      //将缓存内容清零
	{
		uart2_recv_buf[k] = 0x00;
	}
    first_index_to_recv = 0;              //接收字符串的起始存储位置
}

/*******************************************************************************
* 函数名 : dump_uart2_recv_buf
* 描述   : 打印串口2缓存数据
* 输入   : 
* 输出   : 
* 返回   : 
* 注意   : 
*******************************************************************************/
void dump_uart2_recv_buf(void)
{
	u16 k;
	if( !first_index_to_recv ){
		UART1_Printf("Not recv any Msg from GA6!\n");
		return;
	}
	for(k=0;k<first_index_to_recv;k++)      //将缓存内容清零
	{
		if( !(k % 16) ){
			UART1_Printf("\n");
		}
		UART1_Printf(" 0x%02x ",uart2_recv_buf[k]);
	}
  UART1_Printf("\n");
}

/*******************************************************************************
* 函数名 : Wait_CREG
* 描述   : 等待模块注册成功
* 输入   : 
* 输出   : 
* 返回   : 
* 注意   : 
*******************************************************************************/
u8 Wait_CREG(u8 query_times)
{
	u8 i;
	u16 k;
	u8 j;
	i = 0;
	clear_uart2_recv_buf();
	while(i == 0)        			
	{

		UART2_Send_Command("AT+CREG?");
		// 等待2秒
		delays(2); 
		
		for(k=0;k<UART2_BUF_LEN;k++)      			
		{
			if((uart2_recv_buf[k] == '+')&&(uart2_recv_buf[k+1] == 'C')&&
				(uart2_recv_buf[k+2] == 'R')&&(uart2_recv_buf[k+3] == 'E')&&
				(uart2_recv_buf[k+4] == 'G')&&(uart2_recv_buf[k+5] == ':'))
			{
					 
				if((uart2_recv_buf[k+7] == '1')&&((uart2_recv_buf[k+9] == '1')||(uart2_recv_buf[k+9] == '5')))
				{
					i = 1;
					return 1;
				}
				
			}
		}
		j++;
		if(j > query_times)
		{
			return 0;
		}
		
	}
	return 0;
}

/*******************************************************************************
* 函数名 : exist_in_uart2_recv_buf
* 描述   : 判断缓存中是否含有指定的字符串
* 输入   : 
* 输出   : 
* 返回   : unsigned char:1 找到指定字符，0 未找到指定字符 
* 注意   : 
*******************************************************************************/

u8 exist_in_uart2_recv_buf(char *a)
{ 
	if(strstr((const char*)uart2_recv_buf, a)!=NULL)
	{
		return 1;
	}	
	else
	{
		return 0;
	}
}


