#include "pid.h"

//PID_Object pid_gimble_Array[4];//0-speeddown,1-angledown,2-speedup,3-angleup

//PID_Object pid_chassis_Array[5];

//chassis_pid_t pid_chassis_Array[2];
//gimble_pid_t pid_gimble_Array[2];

extern float brake;


	float pid_handle1(PID_Object* p,float SetGoal,float ActualVar)
{
	float detaerror;
	p->target = SetGoal;
	p->feedback = ActualVar;
	p->error = p->target - p->feedback;
	if(abs(p->error) <= p->error_limit)
		p->error = 0;
	
	p->integrate += p->error;
	detaerror = p->error - p->pre_error;
	
	if(p->integrate >= p->integrate_max)
		p->integrate = p->integrate_max;
	if(p->integrate <= - p->integrate_max)
		p->integrate = - p->integrate_max;

	p->Pout = p->error * p->Kp;
	p->Iout = p->integrate * p->Ki;
	p->Dout = p->Kd * detaerror;
	
	p->pre_error = p->error;
	p->out = p->Pout + p->Iout + p->Dout;
	
	if(p->out >= p->out_max)
		p->out = p->out_max;
	if(p->out <= - p->out_max)
		p->out = - p->out_max;
	
	return(p->out);

}

float pid_handle2(PID_Object* p,float err)
{
//	float detaerror;没有KD
	p->error = err;
	if(abs(p->error) <= p->error_limit)
		p->error = 0;
		
	p->integrate += p->error;
	if(p->integrate >= p->integrate_max)
		p->integrate = p->integrate_max;
	if(p->integrate <= - p->integrate_max)
		p->integrate = - p->integrate_max;
	
	p->Pout = p->error * p->Kp;
	p->Iout = p->integrate * p->Ki;
	
	p->out = p->Pout + p->Iout;
	if(p->out >= p->out_max)
		p->out = p->out_max;
	if(p->out <= - p->out_max)
		p->out = - p->out_max;
	
	return(p->out);
}

float gimble_PID(_Total_Pid* p,char i,float SetGoal,float ActualVar)
{
	float detaerror;
	p->pid_test_Array[i].target = SetGoal;
	p->pid_test_Array[i].feedback = ActualVar;
	p->pid_test_Array[i].error = p->pid_test_Array[i].target - p->pid_test_Array[i].feedback;
	if(abs(p->pid_test_Array[i].error) <= p->pid_test_Array[i].error_limit)
		p->pid_test_Array[i].error = 0;
	
	p->pid_test_Array[i].integrate += p->pid_test_Array[i].error;
	detaerror = p->pid_test_Array[i].error - p->pid_test_Array[i].pre_error;
	
	if(p->pid_test_Array[i].integrate >= p->pid_test_Array[i].integrate_max)
		p->pid_test_Array[i].integrate = p->pid_test_Array[i].integrate_max;
	if(p->pid_test_Array[i].integrate <= - p->pid_test_Array[i].integrate_max)
		p->pid_test_Array[i].integrate = - p->pid_test_Array[i].integrate_max;

	p->pid_test_Array[i].Pout = p->pid_test_Array[i].error * p->pid_test_Array[i].Kp;
	p->pid_test_Array[i].Iout = p->pid_test_Array[i].integrate * p->pid_test_Array[i].Ki;
	p->pid_test_Array[i].Dout = p->pid_test_Array[i].Kd * detaerror;
	
	p->pid_test_Array[i].pre_error = p->pid_test_Array[i].error;
	p->pid_test_Array[i].out = p->pid_test_Array[i].Pout + p->pid_test_Array[i].Iout + p->pid_test_Array[i].Dout;
	
	if(p->pid_test_Array[i].out >= p->pid_test_Array[i].out_max)
		p->pid_test_Array[i].out = p->pid_test_Array[i].out_max;
	if(p->pid_test_Array[i].out <= - p->pid_test_Array[i].out_max)
		p->pid_test_Array[i].out = - p->pid_test_Array[i].out_max;
	
	return(p->pid_test_Array[i].out);
}

float gimble_PID2(_Total_Pid* p,char i,float SetGoal,float ActualVar)
{
	float detaerror;
	p->pid_test_Array[i].target = SetGoal;
	p->pid_test_Array[i].feedback = ActualVar;
	p->pid_test_Array[i].error = p->pid_test_Array[i].target - p->pid_test_Array[i].feedback;
	if(abs(p->pid_test_Array[i].error) <= p->pid_test_Array[i].error_limit)
		p->pid_test_Array[i].error = 0;
	
	p->pid_test_Array[i].integrate += p->pid_test_Array[i].error;
	detaerror = p->pid_test_Array[i].error - p->pid_test_Array[i].pre_error;
	
	if(p->pid_test_Array[i].integrate >= p->pid_test_Array[i].integrate_max)
		p->pid_test_Array[i].integrate = p->pid_test_Array[i].integrate_max;
	if(p->pid_test_Array[i].integrate <= - p->pid_test_Array[i].integrate_max)
		p->pid_test_Array[i].integrate = - p->pid_test_Array[i].integrate_max;

	p->pid_test_Array[i].Pout = p->pid_test_Array[i].error * p->pid_test_Array[i].Kp;
	p->pid_test_Array[i].Pout = KalmanFilter(&P_out_kalman.P_out_Kalman_Array[i],p->pid_test_Array[i].Pout);
	p->pid_test_Array[i].Iout = p->pid_test_Array[i].integrate * p->pid_test_Array[i].Ki;
	p->pid_test_Array[i].Dout = p->pid_test_Array[i].Kd * detaerror;
	
	p->pid_test_Array[i].pre_error = p->pid_test_Array[i].error;
	p->pid_test_Array[i].out = p->pid_test_Array[i].Pout + p->pid_test_Array[i].Iout + p->pid_test_Array[i].Dout;
	
	if(p->pid_test_Array[i].out >= p->pid_test_Array[i].out_max)
		p->pid_test_Array[i].out = p->pid_test_Array[i].out_max;
	if(p->pid_test_Array[i].out <= - p->pid_test_Array[i].out_max)
		p->pid_test_Array[i].out = - p->pid_test_Array[i].out_max;
	
	return(p->pid_test_Array[i].out);
}

float Incremental_PI(_Total_Pid* p,char i,float encoder_speed,float Target)
{
	p->pid_test_Array[i].target = Target;
	p->pid_test_Array[i].feedback = encoder_speed;
	p->pid_test_Array[i].error = p->pid_test_Array[i].target - p->pid_test_Array[i].feedback;
	p->pid_test_Array[i].detaerror = p->pid_test_Array[i].error - p->pid_test_Array[i].pre_error;	
	p->pid_test_Array[i].Pout = p->pid_test_Array[i].error * p->pid_test_Array[i].Kp;
	p->pid_test_Array[i].Dout = p->pid_test_Array[i].Kd * p->pid_test_Array[i].detaerror;

	p->pid_test_Array[i].out += p->pid_test_Array[i].Pout + p->pid_test_Array[i].Dout;
	p->pid_test_Array[i].pre_error = p->pid_test_Array[i].error;
	p->pid_test_Array[i].out = myconstrain(p->pid_test_Array[i].out,-(p->pid_test_Array[i].out_max),(p->pid_test_Array[i].out_max));
	
	return (p->pid_test_Array[i].out);
}

float balance_momentum(_Total_Pid* p,char i,float Angle,float gyro,float zhongzhi)
{
	p->pid_test_Array[i].error = Angle - zhongzhi;
	p->pid_test_Array[i].integrate += p->pid_test_Array[i].error;
	p->pid_test_Array[i].integrate = myconstrain(p->pid_test_Array[i].integrate,- (p->pid_test_Array[i].integrate_max),p->pid_test_Array[i].integrate_max);
	
	p->pid_test_Array[i].Pout = p->pid_test_Array[i].error * p->pid_test_Array[i].Kp;
	p->pid_test_Array[i].Iout = p->pid_test_Array[i].integrate * p->pid_test_Array[i].Ki;
	p->pid_test_Array[i].Dout = p->pid_test_Array[i].Kd * gyro;
	
	p->pid_test_Array[i].out = p->pid_test_Array[i].Pout + p->pid_test_Array[i].Iout + p->pid_test_Array[i].Dout;
	p->pid_test_Array[i].out = myconstrain(p->pid_test_Array[i].out,-(p->pid_test_Array[i].out_max),(p->pid_test_Array[i].out_max));

	return(p->pid_test_Array[i].out);
}

float velocity_momentum(_Total_Pid* p,char i,float speed)
{
	p->pid_test_Array[i].error *= 0.65f;
	p->pid_test_Array[i].error += speed*0.35f;
	p->pid_test_Array[i].Ki = p->pid_test_Array[i].Kp /200;
	p->pid_test_Array[i].integrate += p->pid_test_Array[i].error;
	p->pid_test_Array[i].integrate = myconstrain(p->pid_test_Array[i].integrate,- (p->pid_test_Array[i].integrate_max),p->pid_test_Array[i].integrate_max);
	
	p->pid_test_Array[i].detaerror = p->pid_test_Array[i].error - p->pid_test_Array[i].pre_error;

	p->pid_test_Array[i].Pout = p->pid_test_Array[i].error * p->pid_test_Array[i].Kp;
	p->pid_test_Array[i].Iout = p->pid_test_Array[i].integrate * p->pid_test_Array[i].Ki;
	p->pid_test_Array[i].Dout = p->pid_test_Array[i].Kd * p->pid_test_Array[i].detaerror;
	
	p->pid_test_Array[i].pre_error = p->pid_test_Array[i].error;
	p->pid_test_Array[i].out = p->pid_test_Array[i].Pout + p->pid_test_Array[i].Iout + p->pid_test_Array[i].Dout;
	p->pid_test_Array[i].out = myconstrain(p->pid_test_Array[i].out,-(p->pid_test_Array[i].out_max),(p->pid_test_Array[i].out_max));

	return(p->pid_test_Array[i].out);

}
float true_KP = 2;
float true_Ki = 0.7;


float balanstand_momentum(_Total_Pid* p,char i,float SetGoal,float ActualVar,float mpuerror,float zhongzhi,float min,float max)
{
	
	p->pid_test_Array[i].target = SetGoal;
	p->pid_test_Array[i].feedback = ActualVar+mpuerror;
	
	Fuzzytrans(zhongzhi,pitch,p->pid_test_Array[i].pre_error,min,max);
	
	p->pid_test_Array[i].error =KalmanFilter(&P_out_kalman.P_out_Kalman_Array[i], p->pid_test_Array[i].target - p->pid_test_Array[i].feedback );
	
	if(abs(p->pid_test_Array[i].error) <= p->pid_test_Array[i].error_limit)
		p->pid_test_Array[i].error = 0;

	p->pid_test_Array[i].integrate += p->pid_test_Array[i].error;
		if(p->pid_test_Array[i].integrate >= p->pid_test_Array[i].integrate_max)
		p->pid_test_Array[i].integrate = p->pid_test_Array[i].integrate_max;
	if(p->pid_test_Array[i].integrate <= - p->pid_test_Array[i].integrate_max)
		p->pid_test_Array[i].integrate = - p->pid_test_Array[i].integrate_max;
	
	p->pid_test_Array[i].Kp = true_KP + FPID.deta_Kp;
	p->pid_test_Array[i].Ki = true_Ki + FPID.deta_Ki;
	
	p->pid_test_Array[i].Iout = p->pid_test_Array[i].integrate * p->pid_test_Array[i].Ki;
	
	p->pid_test_Array[i].Pout =p->pid_test_Array[i].error * p->pid_test_Array[i].Kp;
	 
	
	p->pid_test_Array[i].pre_error =zhongzhi - pitch;
	
	p->pid_test_Array[i].out = p->pid_test_Array[i].Pout  + p->pid_test_Array[i].Iout;
	p->pid_test_Array[i].out = myconstrain(p->pid_test_Array[i].out,-(p->pid_test_Array[i].out_max),(p->pid_test_Array[i].out_max));
	
	return (p->pid_test_Array[i].out);
}





/*调试PID*/
_Total_Pid Total_pid;                                                                                                                //                 //动量轮右轮          
///////////////////////0       1       2       3       4       5       6       7       8       9       10       11       12       13       14       15       16      17     18      19      20      21       22       23      24      25       26      27    28  29

float _KP[30]         ={7,    45,    6,      1.1,     0.08,   0.5,     6,    370,      0.1,   90 ,     6,       1.7,       6,     0.3,     0.3,       6,       0,      1,   -99,    1 ,      60,     4,     };   //
float _KI[30]         ={0.09,  0,    0.8,    0.003,   0,       0,       0.8,   0.11,   0,      0.6 ,    0.8,      0,       0.8,      0,      0,     0.8,       0,      0,    0,     0,       0,      0.7,   };  //  
float _KD[30]         ={0,     0,      8,      0,     3.5,      2,     8,      4,      0,       0,       8,      0,        8,       0,       0,       8,       25,     0.1, -11.138, 0,       0,       0,     }; 
float KI_addlimit[30] ={18000, 0,   5000,   12000,     0,      0,      5000,  18000,   0,     20000,   5000,   0,       5000,     0,       0,     5000,      0,       0,   0,    20000,     0,     2000,   }; //
float Set_out_limit[30]={8000, 2000,   10000,  800,    4000,   5000,    10000,  10000, 50,   8000,     10000, 1000,      10000,    1000,   50,    10000,  8000,    8000,  8000, 0,      8000,     10000,      };// 
float Blind[30]       ={0,       0,    5,        0,     0,     0,       5,     0,      0,      0 ,       5,     0,         5 ,     0,      0,       5,       0,        0,   0,    0 ,      0,      0,       }; //
//float P_out_limit=50;
void PID_Init (_Total_Pid* p)//30//110 0.001 5 10000
{
	char i=0;
	for(i=0;i<30;i++)///////////!!!!!!!!!!!!!!!!
	{
		p->pid_test_Array[i].feedback=0.0;
		p->pid_test_Array[i].error=0.0;
		p->pid_test_Array[i].pre_error=0.0;
		p->pid_test_Array[i].integrate=0.0;
		p->pid_test_Array[i].out=0.0;
		p->pid_test_Array[i].target=0.0;
		p->pid_test_Array[i].Dout=0.0;
		p->pid_test_Array[i].Iout=0.0;
		p->pid_test_Array[i].Pout=0.0;
		p->pid_test_Array[i].detaerror = 0.0;
		p->pid_test_Array[i].Kp = _KP[i];
		p->pid_test_Array[i].Ki = _KI[i];
		p->pid_test_Array[i].Kd = _KD[i];
		p->pid_test_Array[i].integrate_max = KI_addlimit[i];
		p->pid_test_Array[i].out_max=Set_out_limit[i];
		p->pid_test_Array[i].error_limit = Blind[i];
	}
}

void clear_PID(_Total_Pid* p,char i)
{
		p->pid_test_Array[i].feedback=0.0;
		p->pid_test_Array[i].error=0.0;
		p->pid_test_Array[i].pre_error=0.0;
		p->pid_test_Array[i].integrate=0.0;
		p->pid_test_Array[i].out=0.0;
		p->pid_test_Array[i].target=0.0;
		p->pid_test_Array[i].Dout=0.0;
		p->pid_test_Array[i].Iout=0.0;
		p->pid_test_Array[i].Pout=0.0;
		p->pid_test_Array[i].detaerror = 0.0;
		p->pid_test_Array[i].Kp = _KP[i];
		p->pid_test_Array[i].Ki = _KI[i];
		p->pid_test_Array[i].Kd = _KD[i];
		p->pid_test_Array[i].integrate_max = KI_addlimit[i];
		p->pid_test_Array[i].out_max=Set_out_limit[i];
		p->pid_test_Array[i].error_limit = Blind[i];
}

float pid_handle3_test(_Total_Pid* p,char i,float SetGoal,float ActualVar)
{
	p->pid_test_Array[i].target = SetGoal;
	p->pid_test_Array[i].feedback = ActualVar;
	p->pid_test_Array[i].error = p->pid_test_Array[i].target - p->pid_test_Array[i].feedback;
	if(abs(p->pid_test_Array[i].error) <= p->pid_test_Array[i].error_limit)
		p->pid_test_Array[i].error = 0;
	
  p->pid_test_Array[i].integrate += p->pid_test_Array[i].error;
	p->pid_test_Array[i].detaerror = p->pid_test_Array[i].error - p->pid_test_Array[i].pre_error;

	
	if(p->pid_test_Array[i].integrate >= p->pid_test_Array[i].integrate_max)
		p->pid_test_Array[i].integrate = p->pid_test_Array[i].integrate_max;
	if(p->pid_test_Array[i].integrate <= - p->pid_test_Array[i].integrate_max)
		p->pid_test_Array[i].integrate = - p->pid_test_Array[i].integrate_max;

	p->pid_test_Array[i].Pout = p->pid_test_Array[i].error * p->pid_test_Array[i].Kp;
	p->pid_test_Array[i].Iout = p->pid_test_Array[i].integrate * p->pid_test_Array[i].Ki;
	p->pid_test_Array[i].Dout = p->pid_test_Array[i].Kd * p->pid_test_Array[i].detaerror;
	
	p->pid_test_Array[i].pre_error = p->pid_test_Array[i].error;
	p->pid_test_Array[i].out = p->pid_test_Array[i].Pout + p->pid_test_Array[i].Iout + p->pid_test_Array[i].Dout;
	
	if(p->pid_test_Array[i].out >= p->pid_test_Array[i].out_max)
		p->pid_test_Array[i].out = p->pid_test_Array[i].out_max;
	if(p->pid_test_Array[i].out <= - p->pid_test_Array[i].out_max)
		p->pid_test_Array[i].out = - p->pid_test_Array[i].out_max;
	
	return(p->pid_test_Array[i].out);

}




float pid_handle1_test(_Total_Pid* p,char i,float Err)
{
//	p->pid_test_Array[i].error = KalmanFilter(&P_out_kalman.P_out_Kalman_Array[i],Err);
	p->pid_test_Array[i].error = Err;
	if(abs(p->pid_test_Array[i].error) <= p->pid_test_Array[i].error_limit)
		p->pid_test_Array[i].error = 0;
	
	p->pid_test_Array[i].integrate += p->pid_test_Array[i].error;
	p->pid_test_Array[i].detaerror = p->pid_test_Array[i].error - p->pid_test_Array[i].pre_error;
	
	if(p->pid_test_Array[i].integrate >= p->pid_test_Array[i].integrate_max)
		p->pid_test_Array[i].integrate = p->pid_test_Array[i].integrate_max;
	if(p->pid_test_Array[i].integrate <= - p->pid_test_Array[i].integrate_max)
		p->pid_test_Array[i].integrate = - p->pid_test_Array[i].integrate_max;

	p->pid_test_Array[i].Pout = p->pid_test_Array[i].error * p->pid_test_Array[i].Kp;
	p->pid_test_Array[i].Iout = p->pid_test_Array[i].integrate * p->pid_test_Array[i].Ki;
	p->pid_test_Array[i].Dout = p->pid_test_Array[i].Kd * p->pid_test_Array[i].detaerror;
	
	p->pid_test_Array[i].pre_error = p->pid_test_Array[i].error;
	p->pid_test_Array[i].out = p->pid_test_Array[i].Pout + p->pid_test_Array[i].Iout + p->pid_test_Array[i].Dout;
	
	if(p->pid_test_Array[i].out >= p->pid_test_Array[i].out_max)
		p->pid_test_Array[i].out = p->pid_test_Array[i].out_max;
	if(p->pid_test_Array[i].out <= - p->pid_test_Array[i].out_max)
		p->pid_test_Array[i].out = - p->pid_test_Array[i].out_max;
	
	return(p->pid_test_Array[i].out);

}

	float pid_handle2_test(_Total_Pid* p,char i,float SetGoal,float ActualVar)
{
	p->pid_test_Array[i].target = SetGoal;
	p->pid_test_Array[i].feedback = ActualVar;
	p->pid_test_Array[i].error = p->pid_test_Array[i].target - p->pid_test_Array[i].feedback;
	if(abs(p->pid_test_Array[i].error) <= p->pid_test_Array[i].error_limit)
		p->pid_test_Array[i].error = 0;
	
	p->pid_test_Array[i].integrate += p->pid_test_Array[i].error;
	p->pid_test_Array[i].detaerror = p->pid_test_Array[i].error - p->pid_test_Array[i].pre_error;
	
	if(p->pid_test_Array[i].integrate >= p->pid_test_Array[i].integrate_max)
		p->pid_test_Array[i].integrate = p->pid_test_Array[i].integrate_max;
	if(p->pid_test_Array[i].integrate <= - p->pid_test_Array[i].integrate_max)
		p->pid_test_Array[i].integrate = - p->pid_test_Array[i].integrate_max;

	p->pid_test_Array[i].Pout = p->pid_test_Array[i].error * p->pid_test_Array[i].Kp;
	p->pid_test_Array[i].Pout = KalmanFilter(&P_out_kalman.P_out_Kalman_Array[i],p->pid_test_Array[i].Pout);
	p->pid_test_Array[i].Iout = p->pid_test_Array[i].integrate * p->pid_test_Array[i].Ki;
	p->pid_test_Array[i].Dout = p->pid_test_Array[i].Kd * p->pid_test_Array[i].detaerror;
	
	p->pid_test_Array[i].pre_error = p->pid_test_Array[i].error;
	p->pid_test_Array[i].out = p->pid_test_Array[i].Pout + p->pid_test_Array[i].Iout + p->pid_test_Array[i].Dout;
	
	if(p->pid_test_Array[i].out >= p->pid_test_Array[i].out_max)
		p->pid_test_Array[i].out = p->pid_test_Array[i].out_max;
	if(p->pid_test_Array[i].out <= - p->pid_test_Array[i].out_max)
		p->pid_test_Array[i].out = - p->pid_test_Array[i].out_max;
	
	return(p->pid_test_Array[i].out);

}
float balanstand2(_Total_Pid* p,char i,float SetGoal,float ActualVar,float mpuspeed,float mpuerror)
{
	p->pid_test_Array[i].target = SetGoal;
	p->pid_test_Array[i].feedback = ActualVar;
	p->pid_test_Array[i].error = p->pid_test_Array[i].target - p->pid_test_Array[i].feedback + brake;
	if(abs(p->pid_test_Array[i].error) <= p->pid_test_Array[i].error_limit)
		p->pid_test_Array[i].error = 0;

	p->pid_test_Array[i].Iout = p->pid_test_Array[i].integrate * p->pid_test_Array[i].Ki;
//	p->pid_test_Array[i].detaerror = mpuspeed+mpuerror;
	p->pid_test_Array[i].detaerror = KalmanFilter(&P_out_kalman.P_out_Kalman_Array[i],mpuspeed+mpuerror);//P-error
	p->pid_test_Array[i].integrate += p->pid_test_Array[i].detaerror;
	p->pid_test_Array[i].integrate = myconstrain(p->pid_test_Array[i].integrate,- (p->pid_test_Array[i].integrate_max),p->pid_test_Array[i].integrate_max);
	
	p->pid_test_Array[i].Pout = p->pid_test_Array[i].error * p->pid_test_Array[i].Kp;
	p->pid_test_Array[i].Iout = p->pid_test_Array[i].integrate * p->pid_test_Array[i].Ki;
	p->pid_test_Array[i].Dout = (p->pid_test_Array[i].detaerror)*p->pid_test_Array[i].Kd;
	
	p->pid_test_Array[i].out = p->pid_test_Array[i].Pout + 	p->pid_test_Array[i].Dout + p->pid_test_Array[i].Iout;
	p->pid_test_Array[i].out = myconstrain(p->pid_test_Array[i].out,-(p->pid_test_Array[i].out_max),(p->pid_test_Array[i].out_max));
	
	return (p->pid_test_Array[i].out);
}

float balanSpeed(_Total_Pid* p,char i,float encoder_left,float encoder_right,float remote)
{
	p->pid_test_Array[i].target = (encoder_left - encoder_right) - 0;
	p->pid_test_Array[i].error = p->pid_test_Array[i].pre_error *0.7f + 0.3f * p->pid_test_Array[i].target;
	p->pid_test_Array[i].integrate += p->pid_test_Array[i].error + remote;
	
	p->pid_test_Array[i].integrate = myconstrain(p->pid_test_Array[i].integrate,- (p->pid_test_Array[i].integrate_max),p->pid_test_Array[i].integrate_max);
	
	p->pid_test_Array[i].Ki = p->pid_test_Array[i].Kp /200;
//	p->pid_test_Array[i].Pout = p->pid_test_Array[i].error * p->pid_test_Array[i].Kp;
	
//	P_out_limit = p->pid_test_Array[i].out_max - ( p->pid_test_Array[i].integrate_max *  p->pid_test_Array[i].Ki);
//	p->pid_test_Array[i].Pout = myconstrain(p->pid_test_Array[i].Pout , -(P_out_limit),P_out_limit); 
	p->pid_test_Array[i].Pout = KalmanFilter(&P_out_kalman.P_out_Kalman_Array[i],p->pid_test_Array[i].error * p->pid_test_Array[i].Kp);
	p->pid_test_Array[i].Iout = p->pid_test_Array[i].integrate * p->pid_test_Array[i].Ki;
	p->pid_test_Array[i].out = p->pid_test_Array[i].Pout + p->pid_test_Array[i].Iout;
	

	p->pid_test_Array[i].pre_error = p->pid_test_Array[i].error;
	p->pid_test_Array[i].out = myconstrain(p->pid_test_Array[i].out,-(p->pid_test_Array[i].out_max),(p->pid_test_Array[i].out_max));
	
	return (p->pid_test_Array[i].out);
}

float balanstand(_Total_Pid* p,char i,float SetGoal,float ActualVar,float mpuerror)
{
	p->pid_test_Array[i].target = SetGoal;
	p->pid_test_Array[i].feedback =KalmanFilter(&P_out_kalman.P_out_Kalman_Array[i], ActualVar+mpuerror);
	
//	p->pid_test_Array[i].error =KalmanFilter(&P_out_kalman.P_out_Kalman_Array[i], p->pid_test_Array[i].target - p->pid_test_Array[i].feedback );
	p->pid_test_Array[i].error = p->pid_test_Array[i].target - p->pid_test_Array[i].feedback;
	if(abs(p->pid_test_Array[i].error) <= p->pid_test_Array[i].error_limit)
		p->pid_test_Array[i].error = 0;

	p->pid_test_Array[i].integrate += p->pid_test_Array[i].error;
		if(p->pid_test_Array[i].integrate >= p->pid_test_Array[i].integrate_max)
		p->pid_test_Array[i].integrate = p->pid_test_Array[i].integrate_max;
	if(p->pid_test_Array[i].integrate <= - p->pid_test_Array[i].integrate_max)
		p->pid_test_Array[i].integrate = - p->pid_test_Array[i].integrate_max;

	p->pid_test_Array[i].Iout = p->pid_test_Array[i].integrate * p->pid_test_Array[i].Ki;
//	p->pid_test_Array[i].detaerror = mpuspeed+mpuerror;
	p->pid_test_Array[i].detaerror = p->pid_test_Array[i].error - p->pid_test_Array[i].pre_error;
	
	p->pid_test_Array[i].Pout =p->pid_test_Array[i].error * p->pid_test_Array[i].Kp;
	 
	p->pid_test_Array[i].Dout = (p->pid_test_Array[i].detaerror)*p->pid_test_Array[i].Kd;
	
	p->pid_test_Array[i].pre_error = p->pid_test_Array[i].error;
	
	p->pid_test_Array[i].out = p->pid_test_Array[i].Pout + 	p->pid_test_Array[i].Dout + p->pid_test_Array[i].Iout;
	p->pid_test_Array[i].out = myconstrain(p->pid_test_Array[i].out,-(p->pid_test_Array[i].out_max),(p->pid_test_Array[i].out_max));
	
	return (p->pid_test_Array[i].out);
}

float singlespeedPID(_Total_Pid* p,char i,float encoder_left,float remote)
{
	p->pid_test_Array[i].feedback =   - encoder_left;
	p->pid_test_Array[i].target = remote;
	p->pid_test_Array[i].error = p->pid_test_Array[i].target - p->pid_test_Array[i].feedback;
	if(abs(p->pid_test_Array[i].error) <= p->pid_test_Array[i].error_limit)
		p->pid_test_Array[i].error = 0;
	
	p->pid_test_Array[i].integrate += p->pid_test_Array[i].error;
	p->pid_test_Array[i].detaerror = p->pid_test_Array[i].error - p->pid_test_Array[i].pre_error;
	
	p->pid_test_Array[i].integrate = myconstrain(p->pid_test_Array[i].integrate,- (p->pid_test_Array[i].integrate_max),p->pid_test_Array[i].integrate_max);

	p->pid_test_Array[i].Pout = p->pid_test_Array[i].error * p->pid_test_Array[i].Kp;
	p->pid_test_Array[i].Iout = p->pid_test_Array[i].integrate * p->pid_test_Array[i].Ki;
	p->pid_test_Array[i].Dout = p->pid_test_Array[i].Kd * p->pid_test_Array[i].detaerror;
	
	p->pid_test_Array[i].pre_error = p->pid_test_Array[i].error;
	p->pid_test_Array[i].out = p->pid_test_Array[i].Pout + p->pid_test_Array[i].Iout + p->pid_test_Array[i].Dout;

	p->pid_test_Array[i].out = myconstrain(p->pid_test_Array[i].out,-(p->pid_test_Array[i].out_max),(p->pid_test_Array[i].out_max));
  return (p->pid_test_Array[i].out);

}



float balanturn(_Total_Pid* p,char i,float encoder_right,float encoder_left,float mpuspeed,float mpuerror,float remote)
{
	p->pid_test_Array[i].error += encoder_left - (-encoder_right);
	if(abs(p->pid_test_Array[i].error) <= p->pid_test_Array[i].error_limit)
		p->pid_test_Array[i].error = 0;

	p->pid_test_Array[i].detaerror = mpuspeed + mpuerror;
	
	p->pid_test_Array[i].integrate += -remote;
	p->pid_test_Array[i].Pout = p->pid_test_Array[i].error * p->pid_test_Array[i].Kp + p->pid_test_Array[i].integrate;
	p->pid_test_Array[i].Dout = p->pid_test_Array[i].detaerror * p->pid_test_Array[i].Kd;
	
	
//	p->pid_test_Array[i].Iout = p->pid_test_Array[i].integrate * p->pid_test_Array[i].Ki;
	
	//p->pid_test_Array[i].integrate = myconstrain(p->pid_test_Array[i].integrate,- (p->pid_test_Array[i].integrate_max),p->pid_test_Array[i].integrate_max);

	
	
	p->pid_test_Array[i].out = 
	p->pid_test_Array[i].Pout + 	p->pid_test_Array[i].Dout + p->pid_test_Array[i].Iout;
//	p->pid_test_Array[i].out += remote;
	p->pid_test_Array[i].out = myconstrain(p->pid_test_Array[i].out,-(p->pid_test_Array[i].out_max),(p->pid_test_Array[i].out_max));
	
  return (p->pid_test_Array[i].out);
}//转向1


float balanturn2(_Total_Pid* p,char i,float mpugoalangle,float mpuspeed,float mpuerror)
{
//	p->pid_test_Array[i].target = mpugoalangle;
//	p->pid_test_Array[i].feedback = mpuangle;
	p->pid_test_Array[i].error = mpugoalangle;
	if(abs(p->pid_test_Array[i].error) <= p->pid_test_Array[i].error_limit)
		p->pid_test_Array[i].error = 0;

	p->pid_test_Array[i].detaerror = mpuspeed + mpuerror;
	
	p->pid_test_Array[i].Pout = p->pid_test_Array[i].error * p->pid_test_Array[i].Kp;
	p->pid_test_Array[i].Dout = p->pid_test_Array[i].detaerror * p->pid_test_Array[i].Kd;
	p->pid_test_Array[i].out = p->pid_test_Array[i].Pout + 	p->pid_test_Array[i].Dout + p->pid_test_Array[i].Iout;
	p->pid_test_Array[i].out = myconstrain(p->pid_test_Array[i].out,-(p->pid_test_Array[i].out_max),(p->pid_test_Array[i].out_max));
  return (p->pid_test_Array[i].out);
}

float balanturn3(_Total_Pid* p,char i,float mpuspeed,float mpuerror)
{
	p->pid_test_Array[i].error = mpuspeed + mpuerror;
	if(abs(p->pid_test_Array[i].error) <= p->pid_test_Array[i].error_limit)
		p->pid_test_Array[i].error = 0;

	p->pid_test_Array[i].Pout = p->pid_test_Array[i].error * p->pid_test_Array[i].Kp;
	p->pid_test_Array[i].out = p->pid_test_Array[i].Pout;
	p->pid_test_Array[i].out = myconstrain(p->pid_test_Array[i].out,-(p->pid_test_Array[i].out_max),(p->pid_test_Array[i].out_max));
  return (p->pid_test_Array[i].out);
}



//float pid_handle2_test(_Total_Pid* p,char i,float err)
//{
////	float detaerror;没有KD
//	p->TurnTest[i].err = err;
//	if(abs(p->TurnTest[i].err) <= p->TurnTest[i].blind_limit)
//		p->TurnTest[i].err = 0;
//		
//	p->TurnTest[i].I_err += p->TurnTest[i].err;
//	if(p->TurnTest[i].I_err >= p->TurnTest[i].I_err_limit)
//		p->TurnTest[i].I_err = p->TurnTest[i].I_err_limit;
//	if(p->TurnTest[i].I_err <= - p->TurnTest[i].I_err_limit)
//		p->TurnTest[i].I_err = - p->TurnTest[i].I_err_limit;
//	
//	p->TurnTest[i].KPout = p->TurnTest[i].err * p->TurnTest[i].Kp;
//	p->TurnTest[i].KIout = p->TurnTest[i].I_err * p->TurnTest[i].Ki;
//	
//	p->TurnTest[i].nextSet = p->TurnTest[i].KPout + p->TurnTest[i].KIout;
//	if(p->TurnTest[i].nextSet >= p->TurnTest[i].nextSet_limit)
//		p->TurnTest[i].nextSet = p->TurnTest[i].nextSet_limit;
//	if(p->TurnTest[i].nextSet <= - p->TurnTest[i].nextSet_limit)
//		p->TurnTest[i].nextSet = - p->TurnTest[i].nextSet_limit;
//	
//	return(p->TurnTest[i].nextSet);

//}




/////////////////////////////////////////////////////////////////////////////////////
chassis_pid_t pid_chassis_Array[2]={
	{
		/*左轮*/
		.Speed.Kp = 1,
		.Speed.Ki = 0,
		.Speed.Kd = 0,	// 0.0112		0.0112
		.Speed.target = 0,
		.Speed.feedback = 0,
		.Speed.error = 0,
		.Speed.pre_error = 0,
		.Speed.integrate = 0,
		.Speed.integrate_max = 18000,
		.Speed.Pout = 0,
		.Speed.Iout = 0,
		.Speed.Dout = 0,
		.Speed.out = 0,
		.Speed.error_limit = 0,
		.Angle.Kp = 1,
		.Angle.Ki = 0,
		.Angle.Kd = 0,	// 0.0112		0.0112
		.Angle.target = 0,
		.Angle.feedback = 0,
		.Angle.error = 0,
		.Angle.pre_error = 0,
		.Angle.integrate = 0,
		.Angle.integrate_max = 18000,
		.Angle.Pout = 0,
		.Angle.Iout = 0,
		.Angle.Dout = 0,
		.Angle.out = 0,
		.Angle.error_limit = 0,
	},
	{
		/*右轮*/
		.Speed.Kp = 1,
		.Speed.Ki = 0,
		.Speed.Kd = 0,	// 0.0112		0.0112
		.Speed.target = 0,
		.Speed.feedback = 0,
		.Speed.error = 0,
		.Speed.pre_error = 0,
		.Speed.integrate = 0,
		.Speed.integrate_max = 18000,
		.Speed.Pout = 0,
		.Speed.Iout = 0,
		.Speed.Dout = 0,
		.Speed.out = 0,
		.Speed.error_limit = 0,
		.Angle.Kp = 1,
		.Angle.Ki = 0,
		.Angle.Kd = 0,	// 0.0112		0.0112
		.Angle.target = 0,
		.Angle.feedback = 0,
		.Angle.error = 0,
		.Angle.pre_error = 0,
		.Angle.integrate = 0,
		.Angle.integrate_max = 18000,
		.Angle.Pout = 0,
		.Angle.Iout = 0,
		.Angle.Dout = 0,
		.Angle.out = 0,
		.Angle.error_limit = 0,
	}
};

gimble_pid_t pid_gimble_Array[2] = {
	{
		/*yaw*/
		.Speed.Kp = 1,
		.Speed.Ki = 0,
		.Speed.Kd = 0,	// 0.0112		0.0112
		.Speed.target = 0,
		.Speed.feedback = 0,
		.Speed.error = 0,
		.Speed.pre_error = 0,
		.Speed.integrate = 0,
		.Speed.integrate_max = 18000,
		.Speed.Pout = 0,
		.Speed.Iout = 0,
		.Speed.Dout = 0,
		.Speed.out = 0,
		.Speed.error_limit = 0,
		.Angle.Kp = 1,
		.Angle.Ki = 0,
		.Angle.Kd = 0,	// 0.0112		0.0112
		.Angle.target = 0,
		.Angle.feedback = 0,
		.Angle.error = 0,
		.Angle.pre_error = 0,
		.Angle.integrate = 0,
		.Angle.integrate_max = 18000,
		.Angle.Pout = 0,
		.Angle.Iout = 0,
		.Angle.Dout = 0,
		.Angle.out = 0,
		.Angle.error_limit = 0,
	},
	{
		/*pitch*/
		.Speed.Kp = 1,
		.Speed.Ki = 0,
		.Speed.Kd = 0,	// 0.0112		0.0112
		.Speed.target = 0,
		.Speed.feedback = 0,
		.Speed.error = 0,
		.Speed.pre_error = 0,
		.Speed.integrate = 0,
		.Speed.integrate_max = 18000,
		.Speed.Pout = 0,
		.Speed.Iout = 0,
		.Speed.Dout = 0,
		.Speed.out = 0,
		.Speed.error_limit = 0,
		.Angle.Kp = 1,
		.Angle.Ki = 0,
		.Angle.Kd = 0,	// 0.0112		0.0112
		.Angle.target = 0,
		.Angle.feedback = 0,
		.Angle.error = 0,
		.Angle.pre_error = 0,
		.Angle.integrate = 0,
		.Angle.integrate_max = 18000,
		.Angle.Pout = 0,
		.Angle.Iout = 0,
		.Angle.Dout = 0,
		.Angle.out = 0,
		.Angle.error_limit = 0,
	}
};



