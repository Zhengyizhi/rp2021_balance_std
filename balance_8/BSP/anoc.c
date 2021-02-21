/**
 * @file        anoc.c
 * @author      MaWeiming
 * @Version     V1.0
 * @date        26-February-2020
 * @brief       This file includes the ANOC Protocol external functions 
 * 				(based on ST Peripherals Libaray Keil.STM32F4xx_DFP.2.9.0)
 * @Version		
 */

/**
 *	@Zigbee\anoc
 *	Zigbee���������λ�����ߵ���
 */
 
/* Includes ------------------------------------------------------------------*/
#include "anoc.h"


/* Private typedef -----------------------------------------------------------*/
//typedef struct {
//	uint8_t sof[2];
//	uint8_t cmd_id;
//	uint8_t data_length;
//}Anoc_Frame_Header_t;

//typedef struct {
//	uint8_t buf[ANOC_MAX_DATA_LENGTH];
//}Anoc_Data_t;

//typedef struct {
//	uint8_t check_sum;
//}Anoc_Frame_Tailer_t;

//typedef struct {
//	Anoc_Frame_Header_t	FrameHeader;
//	Anoc_Data_t			Data;
//	Anoc_Frame_Tailer_t	FrameTailer;
//}Anoc_TxPacket_t;

//typedef enum {
//	ANOC_CMD_SENSER = 0x02
//}Anoc_Cmd_Names_t;

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
//static Anoc_TxPacket_t	Anoc_TxPacket = {
//	{
//		.sof[0] = 0xAA,
//		.sof[1] = 0xAA,
//		.cmd_id = ANOC_CMD_SENSER,
//		.data_length = 18,
//	},
//};

/* ## Global variables ## ----------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/* API functions -------------------------------------------------------------*/

void usart1_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //ʹ��GPIOAʱ��
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);//ʹ��USART1ʱ��
 
	//����5��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource0,GPIO_AF_UART4); //GPIOC 12 ����Ϊ usart1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource1,GPIO_AF_UART4); //GPIOD 2 ����Ϊ usart1
	
	// usart1 �˿�����
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1; //GPIO D2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; //������
	GPIO_Init(GPIOA,&GPIO_InitStructure); //��ʼ��PD2

   //USART1 ��ʼ������
	USART_InitStructure.USART_BaudRate = 115200;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//��У��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//����ģʽ
  USART_Init(UART4, &USART_InitStructure); //��ʼ�� usart1

	USART_Cmd(UART4, ENABLE);

}

void Anoc_SendChar(u8 chr)
{
	UART4->DR = chr;
	
	while((UART4->SR&0x40)==0);
}

void UART4_IRQHandler(void)
{
  u8 receive_lenth;
	
	if(USART_GetFlagStatus(UART4, USART_FLAG_IDLE) != RESET)
	{
		receive_lenth = UART4->SR;
		receive_lenth = UART4->DR;
	}
	
}


void ANOC_SendToPc1(int16_t rol, int16_t pit, int16_t yaw)
{
	uint16_t static send_cnt = 0;
	uint8_t i;
	uint16_t check_sum = 0;
	uint8_t data_to_pc[17];
	
	data_to_pc[0] = 0xAA;
	data_to_pc[1] = 0xAA;
	data_to_pc[2] = 0x01;
	data_to_pc[3] = 12;
	
	data_to_pc[4] = (rol & 0xff00) >> 8;
	data_to_pc[5] = (rol & 0xff);
	data_to_pc[6] = (pit & 0xff00) >> 8;
	data_to_pc[7] = (pit & 0xff);
	data_to_pc[8] = (yaw & 0xff00) >> 8;
	data_to_pc[9] = (yaw & 0xff);
	
	data_to_pc[10] = 0;
	data_to_pc[11] = 0;
	data_to_pc[12] = 0;
	data_to_pc[13] = 0;
	data_to_pc[14] = 0;
	data_to_pc[15] = 0;
		
	for(i = 0; i < 16; i++) {
		check_sum += data_to_pc[i];
	}
	data_to_pc[16] = check_sum & 0xff;
	
	if(send_cnt < 5000) {
		for(i = 0; i < 17; i++) {
			Anoc_SendChar(data_to_pc[i]);
		}
		send_cnt++;
	}
}

void ANOC_SendToPc(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz, int16_t mx, int16_t my, int16_t mz)
{
	uint8_t i;
	uint16_t check_sum = 0;
	uint8_t data_to_pc[28];
	
	data_to_pc[0] = 0xAA;
	data_to_pc[1] = 0xAA;
	data_to_pc[2] = 0x02;
	data_to_pc[3] = 18;
	
	data_to_pc[4] = (ax & 0xff00) >> 8;
	data_to_pc[5] = (ax & 0xff);
	data_to_pc[6] = (ay & 0xff00) >> 8;
	data_to_pc[7] = (ay & 0xff);
	data_to_pc[8] = (az & 0xff00) >> 8;
	data_to_pc[9] = (az & 0xff);
	
	data_to_pc[10] = (gx & 0xff00) >> 8;
	data_to_pc[11] = (gx & 0xff);
	data_to_pc[12] = (gy & 0xff00) >> 8;
	data_to_pc[13] = (gy & 0xff);
	data_to_pc[14] = (gz & 0xff00) >> 8;
	data_to_pc[15] = (gz & 0xff);
	
	data_to_pc[16] = (mx & 0xff00) >> 8;
	data_to_pc[17] = (mx & 0xff);
	data_to_pc[18] = (my & 0xff00) >> 8;
	data_to_pc[19] = (my & 0xff);
	data_to_pc[20] = (mz & 0xff00) >> 8;
	data_to_pc[21] = (mz & 0xff);
	
	for(i = 0; i < 22; i++) {
		check_sum += data_to_pc[i];
	}
	data_to_pc[22] = check_sum & 0xff;
	
	for(i = 0; i < 23; i++) {
		Anoc_SendChar(data_to_pc[i]);
	}
}


void RP_SendToPc(float yaw, float pitch, float roll, float rateYaw, float ratePitch, float rateRoll)
{
	uint8_t i;
	uint16_t check_sum = 0;
	uint8_t data_to_pc[29];
	
	data_to_pc[0] = 0xAA;
	data_to_pc[1] = 0xAA;
	data_to_pc[2] = 0x01;
	data_to_pc[3] = 24;
	
	/* ��Ĭ�ϵ�С��ģʽ�������� */
	memcpy(&data_to_pc[4], (uint8_t*)&yaw, 4);
	memcpy(&data_to_pc[8], (uint8_t*)&pitch, 4);
	memcpy(&data_to_pc[12], (uint8_t*)&roll, 4);
	memcpy(&data_to_pc[16], (uint8_t*)&rateYaw, 4);
	memcpy(&data_to_pc[20], (uint8_t*)&ratePitch, 4);
	memcpy(&data_to_pc[24], (uint8_t*)&rateRoll, 4);
	
//	send_cnt++;
//	���²����Ὣ����ת���ɴ��ģʽ���ͳ�ȥ
//	data_to_pc[16] = (rateYaw & 0xff00) >> 8;
//	data_to_pc[17] = (rateYaw & 0xff);
//	data_to_pc[18] = (ratePitch & 0xff00) >> 8;
//	data_to_pc[19] = (ratePitch & 0xff);
//	data_to_pc[20] = (rateRoll & 0xff00) >> 8;
//	data_to_pc[21] = (rateRoll & 0xff);
	
	for(i = 0; i < 28; i++) {
		check_sum += data_to_pc[i];
	}
	data_to_pc[28] = check_sum & 0xff;
	
//	USART1_DMA_SendBuf(data_to_pc, 23);
	for(i = 0; i < 29; i++) {
		Anoc_SendChar(data_to_pc[i]);
	}
}

void RP_SendToPc2(float shoot_freq, float shoot_ping, float shoot_heat, float shoot_pwm, float shoot_speed, float shoot_xxx)
{
	uint8_t i;
	uint16_t check_sum = 0;
	uint8_t data_to_pc[29];	
	
	data_to_pc[0] = 0xAA;
	data_to_pc[1] = 0xAA;
	data_to_pc[2] = 0x02;
	data_to_pc[3] = 24;	
	
	memcpy(&data_to_pc[4], (uint8_t*)&shoot_freq, 4);
	memcpy(&data_to_pc[8], (uint8_t*)&shoot_ping, 4);
	memcpy(&data_to_pc[12], (uint8_t*)&shoot_heat, 4);
	memcpy(&data_to_pc[16], (uint8_t*)&shoot_pwm, 4);
	memcpy(&data_to_pc[20], (uint8_t*)&shoot_speed, 4);
	memcpy(&data_to_pc[24], (uint8_t*)&shoot_speed, 4);
	
	for(i = 0; i < 28; i++) {
		check_sum += data_to_pc[i];
	}
	data_to_pc[28] = check_sum & 0xff;	
	
	for(i = 0; i < 29; i++) {
		Anoc_SendChar(data_to_pc[i]);
	}
}

void RP_SendToPc3(void)
{
	
}
