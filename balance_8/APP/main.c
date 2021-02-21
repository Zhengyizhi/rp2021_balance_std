#include "system.h"

uint32_t Remote_time = 0;
extern int SystemMonitor;
float see1,see2,see3,see4,see5,see6;
float mpulongtime,remoteturn;
float seePstand,seeDstand,seePspeed,seeIspeed,seePturn,seeDturn,seeIstand,seeoutstand,seeoutspeed,seeoutturn;
float seeerror,seefeedback,seetarget,seedetaerror,seeerror,seeout,seeintegrate,seedetaerror;
extern float stand_Speed1,speedcircle_speed1,turn_Speed1;
float Store_NextSpeed12;
extern short int Store_NextSpeed1,Store_NextSpeed2;
extern float gravity;
extern int choose_i,choose_j;
void see11(void)
{
		RP_SendToPc(choose_i,choose_j,pitch,gravity,mpu_data_Array[0].longtimeangle,see1);
}
void see22(void)
{
	RP_SendToPc(Total_pid.pid_test_Array[4].target,Total_pid.pid_test_Array[4].feedback,
	            Total_pid.pid_test_Array[4].Pout,Total_pid.pid_test_Array[4].Dout,seePspeed,seePturn);
}
void see33(void)
{
//	Store_NextSpeed12 = Store_NextSpeed1;
	RP_SendToPc(see6,seeoutstand,seeoutspeed,seeoutturn,
	            see1,Remote_Target_Array[Left_remote_cross]);
}

int main(void)
{
	System_Init();
	PID_Init (&Total_pid);
	control_circle_init();
	Init_kal();
//	control_circle_init();
	Remote_time = micros();
//  Wait_stand_init();
	while(1)
	{
		Loop();
		see1 = Chassis_Moto_Info_Array[1].NowSpeed;
		see2 = Total_pid.pid_test_Array[21].Ki;
		
	//	ff = KalmanFilter(&P_out_kalman.P_out_Kalman_Array[5],mpu_data_Array[0].longtimeangle);
		
		see3 = Total_pid.pid_test_Array[21].Kp;
		see4 = -pitch;
		
		mpulongtime= mpu_data_Array[0].longtimeangle;
		remoteturn = Total_pid.pid_test_Array[5].integrate;
		
		see5 = Total_pid.pid_test_Array[0].out;
		see6 = Chassis_Moto_Info_Array[0].NowSpeed;//you
		
		seeoutstand = Total_pid.pid_test_Array[16].out;
		seePstand = Total_pid.pid_test_Array[16].Pout;
		seeDstand = Total_pid.pid_test_Array[16].Dout;
		seeIstand = Total_pid.pid_test_Array[16].Iout;
		seedetaerror = Total_pid.pid_test_Array[21].detaerror;
		
		seePspeed = -Total_pid.pid_test_Array[3].Pout ;
		seeIspeed = -Total_pid.pid_test_Array[3].Iout;
		seePturn =  Total_pid.pid_test_Array[4].Pout;
		seeDturn =  Total_pid.pid_test_Array[4].Dout;//youÂÖ
		
//		seeerror = Total_pid.pid_test_Array[4].error;
//		seefeedback = Total_pid.pid_test_Array[4].feedback;
//		seetarget = Total_pid.pid_test_Array[4].target ;
		seeoutspeed = Total_pid.pid_test_Array[3].out * Total_pid.pid_test_Array[0].Kp;;
		seeoutturn =	Total_pid.pid_test_Array[4].out;
		seeintegrate = Total_pid.pid_test_Array[3].integrate;
		
		
	}
}


