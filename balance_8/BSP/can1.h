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
	float NowSpeed;//收到的速度
	float NowAngle;//收到的角度
	float Nowtorque;//实际转矩
	float preAngle;//之前的角度
	float detaNowAngle;//数值大小相减
	float NowCurrent;
	float Nowtemper;
	float longtimecircle;                 //////////////////只有这个不是瞬间值
	float longtimeangle;
}_Receive;//不需要初始化


extern _Receive Chassis_Moto_Info_Array[Chassis_Moto_count+1];//0-left,1-right
extern _Receive Gimbel_Moto_Info_Array[Gimbel_Moto_count+1];//0-down,1-up
extern _Receive mpu_data_Array[6];
extern _Receive wheel_data_Array[2];//0-前轮，1-后轮
u8 CAN1_Send_Msg_chassis_2(u8* msg,u8 len);
#endif

