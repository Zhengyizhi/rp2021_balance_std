/** *****************************************************************************************
  * @file    kalman.c                                                                                                                                      *
  * @author  Liu heng                                                                                                                                    *
  * @version V1.0.0                                                                                                                                      *
  * @date    27-August-2013                                                                                                                         *
  * @brief   һά�������˲����ľ���ʵ�֡�ʵ�ֹ�����ȫ��Ӳ���޹أ�   *
  *   ��ֱ�ӵ��ã�������ֲ��                                                                                               *
  *   ʹ��ʱ�ȶ���һ��kalmanָ�룬Ȼ�����kalmanCreate()����һ���˲����**
  *   ÿ�ζ�ȡ�����������ݺ󼴿ɵ���KalmanFilter()�������ݽ����˲���               *
  *****************************************************************************************
  *                          ʹ��ʾ��                                                     *
  *          kalman p;                                                                   *
  *          float SersorData;                                                            *
  *          kalmanCreate(&p,20,200);                                                  *
  *          while(1)                                                                     *
  *          {                                                                            *
  *             SersorData = sersor();                                                    *
  *             SersorData = KalmanFilter(&p,SersorData);                                  *
  *             printf("%2.2f",SersorData);                                               *
  *          }                                                                            *
  *****************************************************************************************
  *          MPU6050�Ŀ������˲����ο����� Q��10 R��400                                   *
  *****************************************************************************************/

#include "kalman.h"
                  //   0   1  2  3  4  5     6    7  8  9  10  11  12  13  14  15  16  17  18  19  20  21  22
float T_Q_Array[30] = {1,  0, 1, 1, 1, 1,    1,   1, 0, 1, 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  };
float T_R_Array[30] = {40,0,100,400,100,10000,1000,1, 0, 1, 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 30,  };


/**
  * @name   kalmanCreate
  * @brief  ����һ���������˲���
  * @param  p:  �˲���
  *         T_Q:ϵͳ����Э����
  *         T_R:��������Э����
  *         
  * @retval none
  */
void kalmanCreate(P_out_filter * p)
{
	static char i;
    //kalman* p = ( kalman*)malloc(sizeof( kalman));
	for(i=0;i<30;i++)
	{
    p->P_out_Kalman_Array[i].X_last = (float)0;
    p->P_out_Kalman_Array[i].P_last = 0;
    p->P_out_Kalman_Array[i].Q = T_Q_Array[i];
    p->P_out_Kalman_Array[i].R = T_R_Array[i];
    p->P_out_Kalman_Array[i].A = 1;
    p->P_out_Kalman_Array[i].H = 1;
    p->P_out_Kalman_Array[i].X_mid = p->P_out_Kalman_Array[i].X_last;
    //return p;
	}
}

/**
  * @name   KalmanFilter
  * @brief  �������˲���
  * @param  p:  �˲���
  *         dat:���˲�����
  * @retval �˲��������
  */

float KalmanFilter(kalman* p,float dat)
{
    p->X_mid =p->A*p->X_last;                     //x(k|k-1) = AX(k-1|k-1)+BU(k)
    p->P_mid = p->A*p->P_last+p->Q;               //p(k|k-1) = Ap(k-1|k-1)A'+Q
    p->kg = p->P_mid/(p->P_mid+p->R);             //kg(k) = p(k|k-1)H'/(Hp(k|k-1)'+R)
    p->X_now = p->X_mid+p->kg*(dat-p->X_mid);     //x(k|k) = X(k|k-1)+kg(k)(Z(k)-HX(k|k-1))
    p->P_now = (1-p->kg)*p->P_mid;                //p(k|k) = (I-kg(k)H)P(k|k-1)
    p->P_last = p->P_now;                         //״̬����
    p->X_last = p->X_now;
    return p->X_now;
}
P_out_filter P_out_kalman;
Aver_Filter _Aver_Filter;
float Pout_Q=1,Pout_R=1;



float average_add(Aver_Filter* Aver,char i,float add_dats)
{
	Aver ->Average_Filter_Array[i].total -= Aver ->Average_Filter_Array[i].num[Aver ->Average_Filter_Array[i].pot];
	Aver ->Average_Filter_Array[i].total += add_dats;
	Aver ->Average_Filter_Array[i].num[Aver ->Average_Filter_Array[i].pot] = add_dats;
	
	Aver ->Average_Filter_Array[i].aver_num = (Aver ->Average_Filter_Array[i].total)/MAF_MaxSize;
	Aver ->Average_Filter_Array[i].pot++;
	
	if(Aver ->Average_Filter_Array[i].pot == MAF_MaxSize)
	{
		Aver ->Average_Filter_Array[i].pot = 0;
	}
	return (Aver ->Average_Filter_Array[i].aver_num);
}

void Init_kal(void)
{
	kalmanCreate ( &P_out_kalman) ;
}


