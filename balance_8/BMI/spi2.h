#ifndef __SPI2_H
#define __SPI2_H
//#include "sys.h"
#include "system.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ���������ɣ��������������κ���;
//ALIENTEK STM32F407������
//SPI ��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2014/5/6
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) �������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	
 	    													  
void SPI2_Init(void);			 //��ʼ��SPI1��
void SPI2_SetSpeed(u8 SpeedSet); //����SPI1�ٶ�   
u8 SPI2_ReadWriteByte(u8 TxData);//SPI1���߶�дһ���ֽ�
		 
#endif
