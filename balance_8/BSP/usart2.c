#include "usart2.h"
/* ----------------------- RC Channel Definition---------------------------- */
#define RC_CH_VALUE_MIN ((uint16_t)364 )
#define RC_CH_VALUE_OFFSET ((uint16_t)1024)
#define RC_CH_VALUE_MAX ((uint16_t)1684)
/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP ((uint16_t)1)
#define RC_SW_MID ((uint16_t)3)
#define RC_SW_DOWN ((uint16_t)2)
/* ----------------------- PC Key Definition-------------------------------- */
//#define KEY_PRESSED_OFFSET_W ((uint16_t)0x01<<0)
//#define KEY_PRESSED_OFFSET_S ((uint16_t)0x01<<1)
//#define KEY_PRESSED_OFFSET_A ((uint16_t)0x01<<2)
//#define KEY_PRESSED_OFFSET_D ((uint16_t)0x01<<3)
//#define KEY_PRESSED_OFFSET_Q ((uint16_t)0x01<<4)
//#define KEY_PRESSED_OFFSET_E ((uint16_t)0x01<<5)
//#define KEY_PRESSED_OFFSET_SHIFT ((uint16_t)0x01<<6)
//#define KEY_PRESSED_OFFSET_CTRL ((uint16_t)0x01<<7)

float DMAinterruptnumber;

volatile unsigned char sbus_rx_buffer[25];
RC_Ctl_t RC_Ctl;

void RC_Init(void)
{
	NVIC_InitTypeDef nvic;
	GPIO_InitTypeDef gpio;
	DMA_InitTypeDef dma;
	USART_InitTypeDef usart2;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1 | RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3, GPIO_AF_USART2);

	gpio.GPIO_Pin = GPIO_Pin_3 ;
	gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &gpio);
	USART_DeInit(USART2);
	usart2.USART_BaudRate = 100000;//波特率
	usart2.USART_WordLength = USART_WordLength_8b;
	usart2.USART_StopBits = USART_StopBits_1;
	usart2.USART_Parity = USART_Parity_Even;
	usart2.USART_Mode = USART_Mode_Rx;
	usart2.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USART2,&usart2);
	USART_Cmd(USART2,ENABLE);
	USART_DMACmd(USART2,USART_DMAReq_Rx,ENABLE);
	

	nvic.NVIC_IRQChannel = DMA1_Stream5_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 1;
	nvic.NVIC_IRQChannelSubPriority = 1;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);
	
	
	DMA_DeInit(DMA1_Stream5);
	dma.DMA_Channel = DMA_Channel_4;
	dma.DMA_PeripheralBaseAddr = (uint32_t)&(USART2->DR);//DMA外设地址
	dma.DMA_Memory0BaseAddr = (uint32_t)sbus_rx_buffer;//DMA储存器地址
	dma.DMA_DIR = DMA_DIR_PeripheralToMemory;//存储器到外设模式
	dma.DMA_BufferSize = 18;//数据传输量 
	dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设非增量模式
	dma.DMA_MemoryInc = DMA_MemoryInc_Enable;//存储器增量模式
	dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//外设数据长度:8位
	dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//存储器数据长度:8位
	dma.DMA_Mode = DMA_Mode_Circular;//循环模式
	dma.DMA_Priority = DMA_Priority_VeryHigh;
	dma.DMA_FIFOMode = DMA_FIFOMode_Disable;
	dma.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;//!!
	dma.DMA_MemoryBurst = DMA_Mode_Normal;
	dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//外设突发单次传输
	DMA_Init(DMA1_Stream5,&dma);
	DMA_ITConfig(DMA1_Stream5,DMA_IT_TC,ENABLE);
	DMA_Cmd(DMA1_Stream5,ENABLE);
}


extern uint32_t  Remote_time;


//char W_key,S_key,A_key,D_key,V_key,Q_key,E_key,C_key,F_key,G_key,R_key,Shift_key,Ctrl_key;
//void Keyanaly()
//{
//	W_key = (char)(RC_Ctl.key.v & 1);
//	S_key = (char)((RC_Ctl.key.v >> 1) & 1);
//	A_key = (char)((RC_Ctl.key.v >> 2) & 1);
//	D_key = (char)((RC_Ctl.key.v >> 3) & 1);
//	Shift_key = (char)((RC_Ctl.key.v >> 4) & 1);
//	Ctrl_key = (char)((RC_Ctl.key.v >> 5) & 1);
//	Q_key = (char)((RC_Ctl.key.v >> 6) & 1);
//	E_key = (char)((RC_Ctl.key.v >> 7) & 1);
//	R_key = (char)((RC_Ctl.key.v >> 8) & 1);
//	F_key	= (char)((RC_Ctl.key.v >> 9) & 1);
//	G_key = (char)((RC_Ctl.key.v >> 10) & 1);
//	
//	C_key = (char)((RC_Ctl.key.v >> 13) & 1);
//	V_key = (char)((RC_Ctl.key.v >> 14) & 1);
//}







void DMA1_Stream5_IRQHandler(void)
{
	DMAinterruptnumber++;
	if(DMA_GetITStatus(DMA1_Stream5, DMA_IT_TCIF5))
	{
		DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF5);
		DMA_ClearITPendingBit(DMA1_Stream5, DMA_IT_TCIF5);
		
		
		RC_Ctl.rc.ch0 = (((int16_t)sbus_rx_buffer[0]| ((int16_t)sbus_rx_buffer[1] << 8)) & 0x07ff )- 1024; //!< Channel 0
		RC_Ctl.rc.ch1 = ((((int16_t)sbus_rx_buffer[1] >> 3) | ((int16_t)sbus_rx_buffer[2] << 5)) & 0x07ff )- 1024; //!< Channel 1
		RC_Ctl.rc.ch2 = ((((int16_t)sbus_rx_buffer[2] >> 6) | ((int16_t)sbus_rx_buffer[3] << 2) | //!< Channel 2
		((int16_t)sbus_rx_buffer[4] << 10)) & 0x07ff )- 1024;
		RC_Ctl.rc.ch3 = ((((int16_t)sbus_rx_buffer[4] >> 1) | ((int16_t)sbus_rx_buffer[5] << 7)) & 0x07ff) - 1024; //!< Channel 3
		RC_Ctl.rc.s1 = ((sbus_rx_buffer[5] >> 4)& 0x000C) >> 2; //!< Switch left
		RC_Ctl.rc.s2 = ((sbus_rx_buffer[5] >> 4)& 0x0003); //!< Switch right
		RC_Ctl.mouse.x = (int16_t)sbus_rx_buffer[6] | ((int16_t)sbus_rx_buffer[7] << 8); //!< Mouse X axis
		RC_Ctl.mouse.y = (int16_t)sbus_rx_buffer[8] | ((int16_t)sbus_rx_buffer[9] << 8); //!< Mouse Y axis
		RC_Ctl.mouse.z = (int16_t)sbus_rx_buffer[10] | ((int16_t)sbus_rx_buffer[11] << 8); //!< Mouse Z axis
		RC_Ctl.mouse.press_l = (int16_t)sbus_rx_buffer[12]; //!< Mouse Left Is Press ?
		RC_Ctl.mouse.press_r = (int16_t)sbus_rx_buffer[13]; //!< Mouse Right Is Press ?
		RC_Ctl.key.v = (int16_t)sbus_rx_buffer[14] | ((int16_t)sbus_rx_buffer[15] << 8); //!< KeyBoard value
		RC_Ctl.rc.sw = ((int16_t)sbus_rx_buffer[16] | ((int16_t)sbus_rx_buffer[17] << 8)) & 0x07FF;
		
		
		if(RC_Ctl.rc.ch3<=1 && RC_Ctl.rc.ch3>=-1)
			RC_Ctl.rc.ch3=0;
		if(RC_Ctl.rc.ch2<=1 && RC_Ctl.rc.ch2>=-1)
			RC_Ctl.rc.ch2=0;
		if(RC_Ctl.rc.ch1<=1 && RC_Ctl.rc.ch1>=-1)
			RC_Ctl.rc.ch1=0;
		if(RC_Ctl.rc.ch0<=1 && RC_Ctl.rc.ch0>=-1)
			RC_Ctl.rc.ch0=0;//死区
		
		

		
		
		
		if(RC_Ctl.rc.ch0>=661 || RC_Ctl.rc.ch0<=-661 || RC_Ctl.rc.ch1>=661 || RC_Ctl.rc.ch1<=-661 || RC_Ctl.rc.ch2>=661 || RC_Ctl.rc.ch2<=-661 || RC_Ctl.rc.ch3>=661 || RC_Ctl.rc.ch3<=-661)
		{
			RC_Ctl.rc.ch0=RC_Ctl.rc.ch1=RC_Ctl.rc.ch2=RC_Ctl.rc.ch3=0;
			AllFlag.RemoteData_Flag = _error;
		}
		else
			AllFlag.RemoteData_Flag = _normal;
		Remote_time = micros() + 30000;
	}
}
		




