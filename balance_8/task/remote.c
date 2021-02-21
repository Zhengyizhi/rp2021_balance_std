#include "remote.h"
//14ms处理一次
float Gimbel_remotepitch_Speedsensitivity = 0.05;
float Gimbel_remotepitch_Anglesensitivity = 0.001;
float Gimbel_remoteyaw_Speedsensitivity = 0.1;
float Gimbel_remoteyaw_Anglesensitivity = 0.02;

float chassis_remoteforward = 0.025;//0.023
float chassis_remoteturn = 0.3;//0.9
//#define Right_remote_cross 0
//#define Right_remote_vertical 1
//#define Left_remote_cross 2
//#define Left_remote_vertical 3

/*遥控器灵敏度*/
float Remote_Target_Array[5];
//float centeryaw;

float Add_down_gimble;

/********刹车**********/
float brake;

Circle_condition Circle_analy;
_doone doone;


/*****正反馈的速度环*******/
int opentime;
/*****改变重心*******/
float gravity = 0;//-0.4/3.4
float gravity_2 = 0;
float nowturncenter,preturncenter;
void remote_calculate()
{
	if(Circle_analy.yawgimble_circle == speedset)
	{
//		Remote_Target_Array[Right_remote_cross] =  RC_Ctl.rc.ch0 * Gimbel_remoteyaw_Speedsensitivity;
//		//!!!!!!!!!!!!!!!!
//		Remote_Target_Array[Right_remote_cross] = myconstrain(Remote_Target_Array[Right_remote_cross] , -75,75);//!!!!!!!!!!!!!!
//		if(Gimbel_Moto_Info_Array[0].NowAngle <=1150 ||  Gimbel_Moto_Info_Array[0].NowAngle >= 5450)
//		{
//			Remote_Target_Array[Right_remote_cross] = 0;
//		}
		
	//	Remote_Target_Array[Right_remote_vertical] = RC_Ctl.rc.ch1 * Gimbel_remotepitch_sensitivity;
	}
	else if(Circle_analy.yawgimble_circle == angleset)
	{
//		Add_down_gimble += RC_Ctl.rc.ch0 * Gimbel_remoteyaw_Anglesensitivity;
//		
//		Remote_Target_Array[Right_remote_cross] = Add_down_gimble + 3375;

//		Remote_Target_Array[Right_remote_cross] = myconstrain(Remote_Target_Array[Right_remote_cross],1208,5504);
//		Add_down_gimble = myconstrain(Add_down_gimble,-2167,2129);
	}
	
	if(Circle_analy.pitchgimble_circle == speedset)
	{
//		Remote_Target_Array[Right_remote_vertical] = -RC_Ctl.rc.ch1 * Gimbel_remotepitch_Speedsensitivity;
//		Remote_Target_Array[Right_remote_vertical] = myconstrain(Remote_Target_Array[Right_remote_vertical],-35,35);
//		if(Gimbel_Moto_Info_Array[1].NowAngle <=3362 ||  Gimbel_Moto_Info_Array[0].NowAngle >= 4650)
//		{
//			Remote_Target_Array[Right_remote_vertical] = 0;
//		}
	}
	else if(Circle_analy.pitchgimble_circle == angleset)
	{
//		Remote_Target_Array[Right_remote_vertical] = RC_Ctl.rc.ch1 * Gimbel_remotepitch_Anglesensitivity;
	}
	
	//站起来
//	if(Circle_analy.control_FBwheel == ReSet)
//	{
//		Searchangle();
//	}
//	else if(Circle_analy.control_FBwheel == Set)
//	{}
	
	
	
	
	
	
	
	if(Circle_analy.Stand_circle == Set)//小车要先刹车再控制
	{
		Remote_Target_Array[Left_remote_vertical] = -RC_Ctl.rc.ch3 * chassis_remoteforward ;
		if(Chassis_Moto_Info_Array[0].NowSpeed >= 1100 && Chassis_Moto_Info_Array[1].NowSpeed <= -1100 && RC_Ctl.rc.ch3 >= 330)
		{
			AllFlag.Car_remote_warning = go_forward;//前进
		}
		else if(Chassis_Moto_Info_Array[1].NowSpeed >= 1100 && Chassis_Moto_Info_Array[0].NowSpeed <= -1100 && RC_Ctl.rc.ch3 <= -330)
		{
			AllFlag.Car_remote_warning = go_back;//后退预警
		}
		else if(abs(Chassis_Moto_Info_Array[1].NowSpeed)<=500 || abs(Chassis_Moto_Info_Array[0].NowSpeed)<=500)
		{
			AllFlag.Car_remote_warning = _clear_warning;
		}
		if(AllFlag.Car_remote_warning == go_forward)
		{
			if(RC_Ctl.rc.ch3 <= -50)
			{
				Remote_Target_Array[Left_remote_vertical] = 0;
			}
		}
		else if(AllFlag.Car_remote_warning == go_back)
		{
			if(RC_Ctl.rc.ch3 >= 50)
			{
				Remote_Target_Array[Left_remote_vertical] = 0;
			}
		}
	
	}
	if(Circle_analy.Speedrun_circle == Set)
	{
	////  Remote_Target_Array[Left_remote_vertical] = RC_Ctl.rc.ch3 * chassis_remoteforward ;
		if(abs(Remote_Target_Array[Left_remote_vertical])<=0.2f)//+-8
		{
			if(Total_pid.pid_test_Array[3].out_max <= 900)
				Total_pid.pid_test_Array[3].out_max += 2;
		}
		else 
		{
			if(Total_pid.pid_test_Array[3].out_max >= 800)
				Total_pid.pid_test_Array[3].out_max -= 2;
		}
	}
	if(Circle_analy.singlespeed == Set)
	{
//		Remote_Target_Array[Left_remote_vertical] = RC_Ctl.rc.ch3 * 12 ;
	}
	if(Circle_analy.Turnaround_circle == SetTurn_Pencoder_Dz)
	{
		Remote_Target_Array[Left_remote_cross] = RC_Ctl.rc.ch2 * chassis_remoteturn;	
	}
	
	
////////		if(abs(RC_Ctl.rc.ch3 )>=5)
////////		{
////////			Circle_analy.Speedrun_circle = ReSet;//reset
////////			clear_PID(&Total_pid,3);
////////			speedcircle_speed1 = speedcircle_speed2 =0;
////////		}
////////		else 
////////		{
////////			Circle_analy.Speedrun_circle = Set;//set
////////		}
		
		
		
//		Remote_Target_Array[Left_remote_vertical] =  RC_Ctl.rc.ch3 * chassis_remoteforward;
	
	
{
//	if(RC_Ctl.rc.s2 == 1)
//	{
//		Circle_analy.Turnaround_circle = turn_center;
//	}
//	else
//	{
//		Circle_analy.Turnaround_circle = SetTurn_Pencoder_Dz;
//	}
	
//	if(Circle_analy.Turnaround_circle == turn_center)//5
//	{
//		static int i;
//		nowturncenter = average_add(&_Aver_Filter,0,KalmanFilter(&P_out_kalman.P_out_Kalman_Array[5],mpu_data_Array[0].longtimeangle));
//		i++;
//		if(i>100)
//		{
//			if(abs(nowturncenter - preturncenter)<=0.5f)
//				gravity = nowturncenter;
//			preturncenter = nowturncenter;
//		}
//		Remote_Target_Array[Right_remote_cross] = RC_Ctl.rc.ch0 * 5;
//	}
	
}
//	if(Circle_analy.Turnaround_circle == turn_center)
//	{
//	
//	}
	
}

void control_circle_init()
{
	Circle_analy.yawgimble_circle = angleset;
	Circle_analy.pitchgimble_circle = Set;
	Circle_analy.Stand_circle = Set;
	Circle_analy.Speedrun_circle =Set;//choose.h
	Circle_analy.singlespeed = ReSet;
	
	Circle_analy.Turnaround_circle = SetTurn_Pencoder_Dz;
	
	
	//动量轮
	Circle_analy.momentum_wheel = Set;
	Ordor_Condition.momentum_order = level_0;
	//站起
	Circle_analy.front_wheel = _not;
	Circle_analy.back_wheel = _not;
	Circle_analy.control_FBwheel = None;
	AllFlag.mpu_Flag = mpu_un;
	AllFlag.Stand_Flag = _none;
//	AllFlag.Stand_Flag = _normal2;
}


