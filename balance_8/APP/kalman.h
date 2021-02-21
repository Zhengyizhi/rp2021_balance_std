/** ******************************************************************************
  * @file    kalman.h                                                            *
  * @author  Liu heng                                                            *
  * @version V1.0.0                                                              *
  * @date    27-August-2013                                                      *
  * @brief   Hearder file for kalman filter                                      *
  *                                                                              *
  ********************************************************************************
  *          �˴�������⴫����ʹ�ã�����ע��������                              *
  ********************************************************************************/
#ifndef _KALMAN_H
#define _KALMAN_H

#include "stdlib.h"
#include "system.h"
#include "system.h"

#define MAF_MaxSize 100
typedef struct {
    float X_last; //��һʱ�̵����Ž��
    float X_mid;  //��ǰʱ�̵�Ԥ����
    float X_now;  //��ǰʱ�̵����Ž��
    float P_mid;  //��ǰʱ��Ԥ������Э����
    float P_now;  //��ǰʱ�����Ž����Э����
    float P_last; //��һʱ�����Ž����Э����
    float kg;     //kalman����
    float A;      //ϵͳ����
    float Q;
    float R;
    float H;
}kalman;

typedef struct{
	kalman P_out_Kalman_Array[30];
	
}P_out_filter;

typedef struct moving_Average_Filter
{
  float num[MAF_MaxSize];
	u8 lenth;
	u8 pot;
	float total;
	float aver_num;
}moving_Average_Filter;
//_Average_Filter
typedef struct{
	moving_Average_Filter Average_Filter_Array[20];
}Aver_Filter;
extern Aver_Filter _Aver_Filter;
void kalmanCreate(P_out_filter * p);
float KalmanFilter(kalman* p,float dat);
void Init_kal(void);
extern P_out_filter P_out_kalman;
float average_add(Aver_Filter* Aver,char i,float add_dats);
#endif
