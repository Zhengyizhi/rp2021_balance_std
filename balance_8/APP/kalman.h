/** ******************************************************************************
  * @file    kalman.h                                                            *
  * @author  Liu heng                                                            *
  * @version V1.0.0                                                              *
  * @date    27-August-2013                                                      *
  * @brief   Hearder file for kalman filter                                      *
  *                                                                              *
  ********************************************************************************
  *          此代码可任意传播与使用，但请注明出处。                              *
  ********************************************************************************/
#ifndef _KALMAN_H
#define _KALMAN_H

#include "stdlib.h"
#include "system.h"
#include "system.h"

#define MAF_MaxSize 100
typedef struct {
    float X_last; //上一时刻的最优结果
    float X_mid;  //当前时刻的预测结果
    float X_now;  //当前时刻的最优结果
    float P_mid;  //当前时刻预测结果的协方差
    float P_now;  //当前时刻最优结果的协方差
    float P_last; //上一时刻最优结果的协方差
    float kg;     //kalman增益
    float A;      //系统参数
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
