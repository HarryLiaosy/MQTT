










/**********************************************************************************
   STM32F103C8T6  	 
 * Ӳ������˵��
	 ʹ�õ�Ƭ����2��GPRSģ��ͨ��  ע��ʹ�ô���2���Ա������غ�ͨ�Ų����ͻ
	 STM32      GPRSģ��
	 ����ӽ��߷���:               mini���ӽ��߷���:
   PA3 (RXD2)->U_RX               PA3 (RXD2)->U_TX
	 PA2 (TXD2)->U_TX               PA2 (TXD2)->U_RX
	 GND	   ->GND                GND	   ->GND
	 
	 PA9(TXD1)--->������Ϣ�˿�
	 PA10(RXD1)-->������Ϣ�˿�
��ƣ�
(1)LED0-7���õ�����Ϊ:PB4-7 PB12-15
(2)KEY0-3���õ�����Ϊ:PA4-7

���Ƽ���:�Ź�
  
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

#define UART2_BUF_LEN 	    256         //����2���泤��
#define STABLE_TIMES        10          //�ȴ�ϵͳ�ϵ����ȶ�

// ����MQTT��������buf����󳤶�
#define MQTT_MAX_BUF_SIZE   256
// ����MQTT��������ip�Ͷ��ڵĶ˿ڣ���ΪGA6ģ��tcpͨ��ʱ��д��ip�Ͷ˿��������ʽ
// ���Ծ���ô�̶�����
#define MQTT_SERVER_IP_AND_PORT         "\"211.149.217.3\",1883"

/*************	���س�������	**************/

/*************  ���ر�������	**************/
//����2�������ݵĻ���
u8 uart2_recv_buf[UART2_BUF_LEN];
// ָʾ��ǰuart2_recv_buf�еĿ��õĵ�һ��index���������մ�����Ϣ
u16 first_index_to_recv = 0;

// ָʾ��ǰҪ��ȡ������Ϣ�ĵ�ǰindex�������21����Ϊ��������������Ϣ��GA6ģ��ͨ������
// ������������ǰ����21�����ݣ����Զ�ȡʱĬ�ϴӵ�22�����ݿ�ʼ��ȡ
static u16 first_index_to_read = 21;

/*************	���غ�������	**************/
void clear_uart2_recv_buf(void);              //�������2���ջ���
u8 Wait_CREG(u8 query_times);                 //�ȴ�ģ��ע��ɹ�
// ����Ƿ���clear_uart2_recv_buf����Ҫ���ҵ��ַ���
u8 exist_in_uart2_recv_buf(char *a);
void dump_uart2_recv_buf(void);

/*************  �ⲿ�����ͱ�������*****************/


/***************************************************************
ע����Ȼ����Է�������ֵ����ȷ����������һ������ָ�����ʧ���ˡ�
****************************************************************/
int transport_sendPacketBuffer(const char*server_ip_and_port,unsigned char* buf, int buflen)
{
	u8 ret;
	char end_char[2];
	char connect_server_ip_port_cmd[56];
	
	// ��Ϊÿ�ζ�ȡ��Ϣ֮ǰ��Ҫ�������ֵ�����Ըɴ�ŵ���������У��Ա��´ζ�ȡ
	// ������ȥ��MQTT�еĴ���
	first_index_to_read = 21;
	
	end_char[0] = 0x1A;//�����ַ�
	end_char[1] = '\0';
	
	ret = UART2_Send_AT_Command("AT+CIPSTATUS","CONNECT OK",3,1);//��ѯ����״̬
	//UART1_Printf("0000 ret %d\n", ret);
	if(ret == 1)//˵����������������״̬
	{
		ret = UART2_Send_AT_Command("AT+CIPSEND",">",3,1);//������������
		//UART1_Printf("1111 ret %d\n", ret);
		if(ret == 0)
		{
			return AT_CIPSEND_ERROR;
		}
		
		UART2_SendU8Array(buf,buflen);
		delays(2);
		ret = UART2_Send_AT_Command_End(end_char,"SEND OK",4,1);//���ͽ��������ȴ�����ok,�ȴ�5S��һ�Σ���Ϊ��������ʱ����ܽϳ�
		//UART1_Printf("2222 ret %d\n", ret);
		if(ret == 0)
		{
			return END_CHAR_ERROR;
		}
		//clear_uart2_recv_buf();
		return 1;
	}
	
	ret = UART2_Send_AT_Command("AT+CGATT=1","OK",2,1);//��������
	//UART1_Printf("3333 ret %d\n", ret);
	if(ret == 0)
	{
		return AT_CGATT_ERROR;
	}
	
	ret = UART2_Send_AT_Command("AT+CGACT=1,1","OK",2,1);//��������
	//UART1_Printf("0000 ret %d\n", ret);
	if(ret == 0)
	{
		return AT_CGATT1_ERROR;
	}

	memset(connect_server_ip_port_cmd,'\0',56);
	strcpy(connect_server_ip_port_cmd,"AT+CIPSTART=\"TCP\",");
	strcat(connect_server_ip_port_cmd,server_ip_and_port);
	
	ret = UART2_Send_AT_Command(connect_server_ip_port_cmd,"CONNECT OK",4,1);//���ӷ�����
	//UART1_Printf("0000 ret %d\n", ret);
	if(ret == 0)
	{
		return AT_CIPSTART_ERROR;
	}
	
	ret = UART2_Send_AT_Command("AT+CIPSEND",">",3,1);//������������
	//UART1_Printf("0000 ret %d\n", ret);
	if(ret == 0)
	{
		return AT_CIPSEND_ERROR;
	}
	
	UART2_SendU8Array(buf,buflen);
	delays(2);
	ret = UART2_Send_AT_Command_End(end_char,"SEND OK",4,1);//���ͽ��������ȴ�����ok,�ȴ�5S��һ�Σ���Ϊ��������ʱ����ܽϳ�
  	if(ret == 0)
	{
		 return END_CHAR_ERROR;
	}
	
	return 1;
}

/**
* Ϊ�˾�����ȥ��MQTT�еĴ��룬����յĺ�����ôʵ�֣����������ʵ�Ķ���
*/
int transport_getdata(unsigned char*buf, int count)
{
	memcpy(buf,&uart2_recv_buf[first_index_to_read],count);
	// ���¶�ȡ����ţ��Ա��´ζ�ȡ
	first_index_to_read += count;
	return count;
}


// ���GA6ģ���״̬
int check_ga6_status(void)
{
	int ret;
	char esc_char[2];
	
	esc_char[0] = 0x1B;//�˳��ַ�
	esc_char[1] = '\0';
	
	ret = UART2_Send_AT_Command("AT","OK",5,1);//����ͨ���Ƿ�ɹ�
	if(ret == 0)
	{
		UART2_SendString(esc_char);   //��һ����>״̬����ô�÷���һ��ESC�ַ�
		return COMMUNITE_ERROR;
	}
	
	ret = UART2_Send_AT_Command("AT+CPIN?","READY",2,1);//��ѯ���Ƿ����
	if(ret == 0)
	{
		return NO_SIM_CARD_ERROR;
	}
	
	ret = Wait_CREG(3);//��ѯ���Ƿ�ע�ᵽ����
	if(ret == 0)
	{
		return SIM_CARD_NO_REG_ERROR;
	}
    
  ret = UART2_Send_AT_Command("ATE0","OK",2,1);//�رջ��Թ���
	if(ret == 0)
	{
		return EAT0_ERROR;
	}
    
	return 1;
}


/*******************************************************************************
* ������ : main 
* ����   : ������
* ����   : 
* ���   : 
* ����   : 
* ע��   : ����2������GA6ģ��ͨ�ţ�����1���ڴ��ڵ��ԣ�
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
	
	SysTick_Init_Config();   //ϵͳ�δ�ʱ�ӳ�ʼ��
	GPIO_Config();           //GPIO��ʼ��
	Key_GPIO_Config();
	USART2_Init_Config(115200);  //����2��ʼ��
	Timer2_Init_Config();        //��ʱ��2��ʼ��
	
	USART1_Init_Config(115200);//UART1�������ڵ�����Ϣ
	
	UART1_SendString("ϵͳ����.......................\r\n");
	
	//�ȴ�ϵͳ�ϵ����ȶ�
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
		// ����KEY1��PA5������������ѭ��
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
		// �ظ�����5�Σ����5�ζ����ɹ������˳�����
		while( retry_count > 0 ){
			ret = transport_sendPacketBuffer(MQTT_SERVER_IP_AND_PORT, buf, len);
			// ������ͳɹ�������ѭ��
			if( ret == 1 ){
				break;
			}
			delays(1);
		}
		if( !retry_count && ret != 1 ){
			 UART1_Printf("3333 transport_sendPacketBuffer Error %d\n", ret);
			 break;
		}
		// ÿ�η���֮�󣬵�����
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
* ������  : USART2_IRQHandler
* ����    : ����2�жϷ������
* ����    : ��
* ����    : �� 
* ˵��    : 
*******************************************************************************/
void USART2_IRQHandler(void)                	
{
	u8 Res=0;
	Res = USART_ReceiveData(USART2);
	uart2_recv_buf[first_index_to_recv] = Res;  	  //�����յ����ַ����浽������
	first_index_to_recv++;                	  //����ָ������ƶ�
	if(first_index_to_recv > UART2_BUF_LEN)       	  //���������,������ָ��ָ�򻺴���׵�ַ
	{
		first_index_to_recv = 0;
	}
} 	

/*******************************************************************************
* ������  : TIM2_IRQHandler
* ����    : ��ʱ��2�ж϶Ϸ�����
* ����    : ��
* ���    : ��
* ����    : �� 
* ˵��    : ��
*******************************************************************************/
void TIM2_IRQHandler(void)   //TIM2�ж�
{
	static u8 flag =1;

	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)  //���TIM2�����жϷ������
	{
		
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);  //���TIM2�����жϱ�־ 
	
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
* ������ : clear_uart2_recv_buf
* ����   : �������2��������
* ����   : 
* ���   : 
* ����   : 
* ע��   : 
*******************************************************************************/
void clear_uart2_recv_buf(void)
{
	u16 k;
	for(k=0;k<UART2_BUF_LEN;k++)      //��������������
	{
		uart2_recv_buf[k] = 0x00;
	}
    first_index_to_recv = 0;              //�����ַ�������ʼ�洢λ��
}

/*******************************************************************************
* ������ : dump_uart2_recv_buf
* ����   : ��ӡ����2��������
* ����   : 
* ���   : 
* ����   : 
* ע��   : 
*******************************************************************************/
void dump_uart2_recv_buf(void)
{
	u16 k;
	if( !first_index_to_recv ){
		UART1_Printf("Not recv any Msg from GA6!\n");
		return;
	}
	for(k=0;k<first_index_to_recv;k++)      //��������������
	{
		if( !(k % 16) ){
			UART1_Printf("\n");
		}
		UART1_Printf(" 0x%02x ",uart2_recv_buf[k]);
	}
  UART1_Printf("\n");
}

/*******************************************************************************
* ������ : Wait_CREG
* ����   : �ȴ�ģ��ע��ɹ�
* ����   : 
* ���   : 
* ����   : 
* ע��   : 
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
		// �ȴ�2��
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
* ������ : exist_in_uart2_recv_buf
* ����   : �жϻ������Ƿ���ָ�����ַ���
* ����   : 
* ���   : 
* ����   : unsigned char:1 �ҵ�ָ���ַ���0 δ�ҵ�ָ���ַ� 
* ע��   : 
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


