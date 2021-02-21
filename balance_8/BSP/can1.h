#ifndef __CAN1_H
#define __CAN1_H

#include "system.h"
#define Chassis_Moto_count 5
#define Gimbel_Moto_count 2
void CAN1_Init(void);
//void CAN1_Send(uint32_t Equipment_ID,int16_t Data0,int16_t Data1,int16_t Data2,int16_t Data3);
u8 DataCan1Analy(u16 id,u8 *Recbuf);
u8 CAN1_Send_Msg_chassis(u8* msg,u8 len);
u8 CAN1_Send_Msg_gimbel(u8* msg,u8 len);
void Chassis_AngleAnaly(char num,float jump,float center,float degree);




typedef struct {
	float NowSpeed;//�յ����ٶ�
	float NowAngle;//�յ��ĽǶ�
	float Nowtorque;//ʵ��ת��
	float preAngle;//֮ǰ�ĽǶ�
	float detaNowAngle;//��ֵ��С���
	float NowCurrent;
	float Nowtemper;
	float longtimecircle;                 //////////////////ֻ���������˲��ֵ
	float longtimeangle;
}_Receive;//����Ҫ��ʼ��


extern _Receive Chassis_Moto_Info_Array[Chassis_Moto_count+1];//0-left,1-right
extern _Receive Gimbel_Moto_Info_Array[Gimbel_Moto_count+1];//0-down,1-up
extern _Receive mpu_data_Array[6];
extern _Receive wheel_data_Array[2];//0-ǰ�֣�1-����
u8 CAN1_Send_Msg_chassis_2(u8* msg,u8 len);
#endif

