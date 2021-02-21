#ifndef __pid_H
#define __pid_H
#include "system.h"
#include "fuzzypid.h"
typedef struct{
	float Kp;
	float Ki;
	float Kd;
	float target;
	float feedback;
	float error;
	float error_limit;
	float pre_error;
	float integrate;
	float integrate_max;
	float Pout;
	float Iout;
	float Dout;
	float out;
	float out_max;//_R
	float detaerror;
}PID_Object;


typedef struct{
	PID_Object Speed;
	PID_Object Angle;
}chassis_pid_t;//

typedef struct{
	PID_Object Speed;
	PID_Object Angle;
}gimble_pid_t;


typedef struct{
	PID_Object pid_test_Array[30];//∫Û√Ê»•µÙ
}_Total_Pid;


float pid_handle1(PID_Object* p,float SetGoal,float ActualVar);

float pid_handle2(PID_Object* p,float err);
void PID_Init (_Total_Pid* p);
extern _Total_Pid Total_pid;


float pid_handle1_test(_Total_Pid* p,char i,float Err);
float pid_handle2_test(_Total_Pid* p,char i,float SetGoal,float ActualVar);
float pid_handle3_test(_Total_Pid* p,char i,float SetGoal,float ActualVar);
float balanSpeed(_Total_Pid* p,char i,float encoder_left,float encoder_right,float remote);
float balanstand(_Total_Pid* p,char i,float SetGoal,float ActualVar,float mpuerror);
float balanturn(_Total_Pid* p,char i,float encoder_left,float encoder_right,float mpuspeed,float mpuerror,float remote);
float balanturn2(_Total_Pid* p,char i,float mpugoalangle,float mpuspeed,float mpuerror);
float balanturn3(_Total_Pid* p,char i,float mpuspeed,float mpuerror);
float balanstand2(_Total_Pid* p,char i,float SetGoal,float ActualVar,float mpuspeed,float mpuerror);
float singlespeedPID(_Total_Pid* p,char i,float encoder_left,float remote);
void clear_PID(_Total_Pid* p,char i);

float gimble_PID2(_Total_Pid* p,char i,float SetGoal,float ActualVar);
float gimble_PID(_Total_Pid* p,char i,float SetGoal,float ActualVar);

float Incremental_PI(_Total_Pid* p,char i,float encoder_speed,float Target);
float balance_momentum(_Total_Pid* p,char i,float Angle,float gyro,float zhongzhi);
float velocity_momentum(_Total_Pid* p,char i,float speed);
float balanstand_momentum(_Total_Pid* p,char i,float SetGoal,float ActualVar,float mpuerror,float zhongzhi,float min,float max);


#endif

