#include "can1.h"

//#define Chassis_Moto_count 2
//#define Gimbel_Moto_count 2
 _Receive Chassis_Moto_Info_Array[Chassis_Moto_count+1];//0-left,1-right
 _Receive Gimbel_Moto_Info_Array[Gimbel_Moto_count+1];//0-down,1-up

float CAN1interruptnumber;
void CAN1_Init()
{
	GPIO_InitTypeDef gpio;
	NVIC_InitTypeDef nvic;
	CAN_InitTypeDef can;
	CAN_FilterInitTypeDef filter;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1,ENABLE);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource1,GPIO_AF_CAN1);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource0,GPIO_AF_CAN1);

  gpio.GPIO_Pin=GPIO_Pin_1|GPIO_Pin_0;
	gpio.GPIO_Mode=GPIO_Mode_AF;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
	
	GPIO_Init(GPIOD,&gpio);


	nvic.NVIC_IRQChannel = CAN1_RX0_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 0;
	nvic.NVIC_IRQChannelSubPriority = 1;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);
	
	can.CAN_TTCM = DISABLE;		//��ʱ�䴥��ͨ��ģʽ
	can.CAN_ABOM = DISABLE;		//����Զ����߹���
	can.CAN_AWUM = DISABLE;		//˯��ģʽͨ���������(���CAN->MCR��SLEEPλ)
	can.CAN_NART = DISABLE;		//��ֹ�����Զ����� ���߸������һ��CAN ��Ӱ�췢�� ��ʱ�ɸ�ΪENABLE
	can.CAN_RFLM = DISABLE;		//���Ĳ��������µĸ��Ǿɵ�
	can.CAN_TXFP = ENABLE;		//���ȼ��ɱ��ı�ʶ������
	can.CAN_BS1=CAN_BS1_9tq;
	can.CAN_BS2=CAN_BS2_4tq;
	can.CAN_Mode=CAN_Mode_Normal;
	can.CAN_Prescaler=3;
	can.CAN_SJW=CAN_SJW_1tq;
	CAN_Init(CAN1,&can);
	
	filter.CAN_FilterNumber=0;  							 			//������0
	filter.CAN_FilterMode=CAN_FilterMode_IdMask;   	//����ģʽ
	filter.CAN_FilterScale=CAN_FilterScale_32bit;   // 32λ��
	filter.CAN_FilterFIFOAssignment=0;              //������0������FIFO0
	filter.CAN_FilterActivation=ENABLE;   				  //���������
	filter.CAN_FilterIdHigh=0x0000;                 //32λID
	filter.CAN_FilterIdLow=0x0000;
	filter.CAN_FilterMaskIdHigh=0x0000;             //32λMask
	filter.CAN_FilterMaskIdLow=0x0000;
	CAN_FilterInit(&filter);
	
	CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);    ////FIFO0��Ϣ�Һ��ж�����
}



u8 CAN1_Send_Msg_chassis(u8* msg,u8 len)
{	
  u8 mbox;
  u16 i=0;
  CanTxMsg TxMessage;
  TxMessage.StdId=0x200;	 // ��׼��ʶ��Ϊ0
  TxMessage.ExtId=0x12;	 // ������չ��ʾ����29λ��
  TxMessage.IDE=0;		  // ʹ����չ��ʶ��
  TxMessage.RTR=0;		  // ��Ϣ����Ϊ����֡��һ֡8λ
  TxMessage.DLC=len;							 // ������֡��Ϣ
  for(i=0;i<len;i++)
  TxMessage.Data[i]=msg[i];				 // ��һ֡��Ϣ          
  mbox= CAN_Transmit(CAN1, &TxMessage);   
  i=0;
  while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//�ȴ����ͽ���
  if(i>=0XFFF)
	{
		return 1;
	}
  return 0;		
}

u8 CAN1_Send_Msg_chassis_2(u8* msg,u8 len)
{	
  u8 mbox;
  u16 i=0;
  CanTxMsg TxMessage;
  TxMessage.StdId=0x1ff;	 // ��׼��ʶ��Ϊ0
  TxMessage.ExtId=0x12;	 // ������չ��ʾ����29λ��
  TxMessage.IDE=0;		  // ʹ����չ��ʶ��
  TxMessage.RTR=0;		  // ��Ϣ����Ϊ����֡��һ֡8λ
  TxMessage.DLC=len;							 // ������֡��Ϣ
  for(i=0;i<len;i++)
  TxMessage.Data[i]=msg[i];				 // ��һ֡��Ϣ          
  mbox= CAN_Transmit(CAN1, &TxMessage);   
  i=0;
  while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//�ȴ����ͽ���
  if(i>=0XFFF)
	{
		return 1;
	}
  return 0;		
}

u8 CAN1_Send_Msg_gimbel(u8* msg,u8 len)
{	
  u8 mbox;
  u16 i=0;
  CanTxMsg TxMessage;
  TxMessage.StdId=0x2ff;	 // ��׼��ʶ��Ϊ0
  TxMessage.ExtId=0x12;	 // ������չ��ʾ����29λ��
  TxMessage.IDE=0;		  // ʹ����չ��ʶ��
  TxMessage.RTR=0;		  // ��Ϣ����Ϊ����֡��һ֡8λ
  TxMessage.DLC=len;							 // ������֡��Ϣ
  for(i=0;i<len;i++)
  TxMessage.Data[i]=msg[i];				 // ��һ֡��Ϣ          
  mbox= CAN_Transmit(CAN1, &TxMessage);   
  i=0;
  while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//�ȴ����ͽ���
  if(i>=0XFFF)
	{
		return 1;
	}
	
  return 0;		
}

void CAN1_RX0_IRQHandler()
{
	CanRxMsg RxMessage;
	if(CAN_GetITStatus(CAN1,CAN_IT_FMP0)!=RESET)
	{
		CAN1interruptnumber++;
		CAN_ClearITPendingBit(CAN1,CAN_IT_FMP0);		//����жϹ���
		CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);		//����can����
		DataCan1Analy(RxMessage.StdId,RxMessage.Data);
	}
}
char ReceiveFlag_chassis;
char ReceiveFlag_Gimble;
static char choose_receive;
u8 DataCan1Analy(u16 id,u8 *Recbuf)
{
			switch(id)//�������	
		{
			case 0x201:ReceiveFlag_chassis=0;choose_receive = 1;break;//����
			case 0x202:ReceiveFlag_chassis=1;choose_receive = 1;break;//Alldata.Redata[5]
			case 0x208:ReceiveFlag_chassis=2;choose_receive = 1;break;//����
			case 0x207:ReceiveFlag_chassis=3;choose_receive = 1;break;//ǰ��
			case 0x203:ReceiveFlag_chassis=4;choose_receive = 1;break;//����������
			case 0x204:ReceiveFlag_chassis=5;choose_receive = 1;break;//����������
			case 0x209:ReceiveFlag_Gimble=0;choose_receive = 2;break;
			case 0x20a:ReceiveFlag_Gimble=1;choose_receive = 2;break;

			default:ReceiveFlag_chassis =4;return 0;//δ֪ID
		}
		if(choose_receive == 1)
		{
		Chassis_Moto_Info_Array[ReceiveFlag_chassis].NowAngle=(short int)(Recbuf[0]<<8 | Recbuf[1]);
		Chassis_Moto_Info_Array[ReceiveFlag_chassis].NowSpeed=(short int)(Recbuf[2]<<8 | Recbuf[3]);
		Chassis_Moto_Info_Array[ReceiveFlag_chassis].NowCurrent=(short int)(Recbuf[4]<<8 | Recbuf[5]);
		Chassis_Moto_Info_Array[ReceiveFlag_chassis].Nowtemper=Recbuf[6];//�������
		}
		else if (choose_receive == 2)
		{
		Gimbel_Moto_Info_Array[ReceiveFlag_Gimble].NowAngle=(short int)(Recbuf[0]<<8 | Recbuf[1]);
		Gimbel_Moto_Info_Array[ReceiveFlag_Gimble].NowSpeed=(short int)(Recbuf[2]<<8 | Recbuf[3]);
		Gimbel_Moto_Info_Array[ReceiveFlag_Gimble].NowCurrent=(short int)(Recbuf[4]<<8 | Recbuf[5]);
		Gimbel_Moto_Info_Array[ReceiveFlag_Gimble].Nowtemper=Recbuf[6];//�������	
		}
		return 1;
}

void Chassis_AngleAnaly(char num,float jump,float center,float degree)
{
	Chassis_Moto_Info_Array[num].detaNowAngle = Chassis_Moto_Info_Array[num].NowAngle - Chassis_Moto_Info_Array[num].preAngle;
	if(Chassis_Moto_Info_Array[num].detaNowAngle < -jump)//��תԾ��
		Chassis_Moto_Info_Array[num].longtimecircle ++;//����ת��һȦ
	else if (Chassis_Moto_Info_Array[num].detaNowAngle > jump)//��תԾ��
		Chassis_Moto_Info_Array[num].longtimecircle --;//����һȦ
	Chassis_Moto_Info_Array[num].longtimeangle = -degree * Chassis_Moto_Info_Array[num].longtimecircle + ( center - Chassis_Moto_Info_Array[num].NowAngle );	
Chassis_Moto_Info_Array[num].preAngle =  Chassis_Moto_Info_Array[num].NowAngle;
}

//void CAN1_TX_IRQHandler()
//{
//	if(CAN_GetITStatus!=RESET)
//		{
//			CAN_ClearITPendingBit(CAN1,CAN_IT_TME);
//		}
//}


