#include "chassis.h"
#include "my_include.h"

////char test[6];//最大5,"0正确，2有问题"
/////*test [0]环的名字*/
//Circle_control Circle_startSet (uint8_t circle_Flag,uint8_t status)
//{
//	Circle_control circlestatus  = None;
//	
//	assert_param(IS_Circle_GET_FLAG(circle_Flag));//调试的时候可以试试
//	if(status == Set)
//	{
//		circlestatus = Set;
//	}
//	else if(status == ReSet)
//	{
//		circlestatus = ReSet;
//	}	
//	return circlestatus;
//}
//站起来
float Front_wheel_angle,Back_wheel_angle;
float Front_wheel_change,Back_wheel_change;
short int Store_NextSpeed1;//暂时储存，可以有正负，以数组的形式输出，所以short int
short int Store_NextAngle1;
short int Store_NextSpeed2;
short int Store_NextAngle2;
short int Momentum_wheel_1,Momentum_wheel_2;//1_动量左轮，2_动量右轮
//short int Store_NextSpeed3;
//short int Store_NextSpeed4;
_Receive mpu_data_Array[6];
extern float gravity_2;
#define  center_Pitch 179
void wheel_Initanaly()
{
	if(mpu_data_Array[0].longtimeangle<=-29.5)
	{
		Circle_analy.front_wheel_initial = inside;
		Circle_analy.back_wheel_initial = outside;
	}
	else if(mpu_data_Array[0].longtimeangle>=36)
	{
		Circle_analy.back_wheel_initial = inside;
		Circle_analy.front_wheel_initial = outside;
	}
	else if(mpu_data_Array[0].longtimeangle<=30 || mpu_data_Array[0].longtimeangle >= -25)
	{
		Circle_analy.back_wheel_initial = outside;
		Circle_analy.front_wheel_initial = outside;
	}
}

float ttr;
void Wait_stand_init()
{
	static int i;
	if(pitch>=-10&&pitch<=10)
	{
		AllFlag.mpu_Flag = mpu_OK;
//		AllFlag.Car_level = one;
		ttr=1;
	}
	if(Circle_analy.control_FBwheel == None && AllFlag.mpu_Flag == mpu_OK)
	{
		Circle_analy.control_FBwheel = ReSet;
		AllFlag.Stand_Flag = _normal;//只进一次也必进一次，也就是一定要找到零点。
		AllFlag.Car_level = two;
		ttr=2;
	}
	if(Circle_analy.control_FBwheel == Set && AllFlag.Stand_Flag == _normal && AllFlag.mpu_Flag == mpu_OK)//
	{
		Front_wheel_change = 32000;
		Back_wheel_change = 43000;
		AllFlag.Stand_Flag = _normal2;
		ttr=3;
	}
//	if(abs(mpu_data_Array[0].longtimeangle)<=6)
//	{
//		i++;
//		if(i>=125)
//		{
//		AllFlag.Stand_Flag = _normal2;
//		}
//	}
	if(AllFlag.Stand_Flag == _normal2)
	{
		if(abs(mpu_data_Array[0].longtimeangle)<=6)
		{
			i++;
			if(i>=80)
			{
				AllFlag.Stand_Flag = _normal3;
				Front_wheel_change = 0;
				Back_wheel_change = 0;
			}
		}
	}
//	else
//		AllFlag.Stand_Flag = _error;
//	mpu_data_Array[4].preAngle = -170;
//	mpu_data_Array[num].longtimeangle
}


void mpu_AngleAnaly(char num,float jump,float center,float degree,float Nowangle)//4
{	
	if(center - Nowangle>200 && Nowangle < 0)//越界
	{
		mpu_data_Array[num].longtimeangle = (-360 + center - Nowangle);//+
	}
	else
	{
		mpu_data_Array[num].longtimeangle = center - Nowangle;//-
	}
//
//	mpu_data_Array[num].longtimeangle = -degree * mpu_data_Array[num].longtimecircle + ( center - Nowangle );	
//mpu_data_Array[num].preAngle =  Nowangle;
}

void mpu_AngleAnaly2(char num,float jump,float center,float degree,float Nowangle)
{
	mpu_data_Array[num].NowAngle = Nowangle;
	mpu_data_Array[num].detaNowAngle = mpu_data_Array[num].NowAngle - mpu_data_Array[num].preAngle;
	if(mpu_data_Array[num].detaNowAngle <= -jump)
		mpu_data_Array[num].longtimecircle ++;
	else if(mpu_data_Array[num].detaNowAngle >= jump)//反转
		mpu_data_Array[num].longtimecircle --;
	mpu_data_Array[num].longtimeangle = -degree * mpu_data_Array[num].longtimecircle + (center - mpu_data_Array[num].NowAngle);
	mpu_data_Array[num].preAngle = mpu_data_Array[num].NowAngle;
}

extern float gravity;
float softtry;
float stand_angle,stand_Speed1,stand_Speed2;//
float speedcircle_speed1,speedcircle_speed2,singlespeed1,singlespeed2;
float alpha_1,alpha_2;
float turn_Speed1,turn_Speed2;
float centeryaw;
float stand_speedproportion1,speed_speedproportion1;

float qianjing;
float sss;
void chassis_control()
{
	if(AllFlag.Can1interrupt_Flag == _normal && (AllFlag.DMAinterrupt_Flag == _disconnected || AllFlag.RemoteData_Flag == _error))//只读取信息电机再can.c,遥控器突然失联
	{
		Store_NextSpeed1=Store_NextSpeed2=Front_wheel = Back_wheel= Momentum_wheel_1 = Momentum_wheel_2=0;
		mpu_AngleAnaly(0,200,0+gravity,360,pitch);
//		PID_Init (&Total_pid);
//		if(AllFlag.Stand_Flag == _normal)
//		{
//			centeryaw = yaw;
//			mpu_data_Array[5].preAngle = centeryaw;
//		}
		if (Circle_analy.front_wheel == _OK)
		{
			AngleAnaly(0,5000,front_wheelmax,8192,Chassis_Moto_Info_Array[3].NowAngle);
		}
		if(Circle_analy.back_wheel == _OK)
		{
			AngleAnaly(1,5000,back_wheelmax,8192,Chassis_Moto_Info_Array[2].NowAngle);
		}
	}
	if(AllFlag.DMAinterrupt_Flag == _normal&& AllFlag.Can1interrupt_Flag == _normal)
	{
		if(Circle_analy.Stand_circle == Set)
		{
			mpu_AngleAnaly(0,200,0+gravity,360,pitch);
			stand_angle = pid_handle1_test(&Total_pid,1,mpu_data_Array[0].longtimeangle + Remote_Target_Array[Left_remote_vertical]);
			stand_Speed1 = (short int)(balanstand(&Total_pid,0,stand_angle,-gyroy,0));//左轮
//		stand_Speed1 = (short int)balanstand2(&Total_pid,2,0,-mpu_data_Array[0].longtimeangle,gyroy,0);

			stand_Speed2 = -stand_Speed1;//右轮
		}
		
		if(Circle_analy.Speedrun_circle == Set)
		{
			speedcircle_speed1 = (short int)((balanSpeed(&Total_pid,3,Chassis_Moto_Info_Array[0].NowSpeed,Chassis_Moto_Info_Array[1].NowSpeed,0))* Total_pid.pid_test_Array[0].Kp);
			//右轮
			speedcircle_speed2 = -speedcircle_speed1;
			//左轮
		}
		
////////////////////////////////////////////////////////////////////		
		if(Circle_analy.Turnaround_circle == SetTurn_Pencoder_Dz)
		{
			turn_Speed1 = (short int)(balanturn(&Total_pid,4,Chassis_Moto_Info_Array[1].NowSpeed,Chassis_Moto_Info_Array[0].NowSpeed,-gyroz,0,-Remote_Target_Array[Left_remote_cross]));
			//右轮
			turn_Speed2 = -turn_Speed1;
			//左轮
		}
//		else if(Circle_analy.Turnaround_circle == SetTurn_Pz_Dz)
//		{
//			mpu_AngleAnaly2(5,200,centeryaw,360,yaw);
//			turn_Speed1 = (short int)(balanturn2(&Total_pid,6,mpu_data_Array[5].longtimeangle,gyroz,0));
//			turn_Speed2 = -turn_Speed1;
//		
//		}
		
//		else if(Circle_analy.Turnaround_circle == turn_center)
//		{
//			turn_Speed1 = (short int)(balanturn(&Total_pid,5,Chassis_Moto_Info_Array[1].NowSpeed,Chassis_Moto_Info_Array[0].NowSpeed,-gyroz,0,Remote_Target_Array[Right_remote_cross]));
//			//右轮
//			turn_Speed2 = -turn_Speed1;
//			//左轮
//		}
		
		if(Circle_analy.singlespeed == Set)
		{
			singlespeed1 = (short int)(singlespeedPID(&Total_pid,6,Chassis_Moto_Info_Array[0].NowSpeed,Remote_Target_Array[Left_remote_vertical]));
			singlespeed2 = -(short int)(singlespeedPID(&Total_pid,15,-Chassis_Moto_Info_Array[1].NowSpeed,Remote_Target_Array[Left_remote_vertical]));	
		}
		
		if(Circle_analy.momentum_wheel == Set)
		{
			mpu_AngleAnaly(1,200,0+gravity_2,360,pitch);
			choose_feedback ();
//			alpha_1 = -(short int)(pid_handle3_test(&Total_pid,14,0,mpu_data_Array[1].longtimeangle ));//动量左轮角加速度
//			alpha_2 = (short int)(pid_handle3_test(&Total_pid,16,0,mpu_data_Array[1].longtimeangle ));//动量右轮角加速度
		}
		
	//	if(Circle_analy.Speedrun_circle == Set && Circle_analy.Stand_circle == Set)
		{
//			static float max,min;
			Store_NextSpeed1 = stand_Speed1 + speedcircle_speed1 - turn_Speed1 -singlespeed1;//右轮
			sss = Store_NextSpeed1;
			Store_NextSpeed2 = stand_Speed2 + speedcircle_speed2 + turn_Speed2 -singlespeed2;//左轮
			//前进
//			if(Chassis_Moto_Info_Array[0].NowSpeed >= 9000 && Chassis_Moto_Info_Array[1].NowSpeed <= -9000)
//			{
//				max = 1000;
//				min = 10000;
//			}
//			//后退
//			else if(Chassis_Moto_Info_Array[1].NowSpeed >= 9000 && Chassis_Moto_Info_Array[0].NowSpeed <= -9000)
//			{
//				max = 10000;
//				min = 1000;
//			}
//			else 
//			{
//				max = 10000;
//				min = 10000;
//			}
			
			
			Store_NextSpeed1 = myconstrain(Store_NextSpeed1,-10000,10000);//右
			Store_NextSpeed2 = myconstrain(Store_NextSpeed2,-10000,10000);
//			Store_NextSpeed1 = myconstrain(Store_NextSpeed1,-10000,10000);
//			Store_NextSpeed2 = myconstrain(Store_NextSpeed2,-10000,10000);
//			Momentum_wheel_1 = Momentum_wheel_1 + alpha_1;//动量左轮
//			Momentum_wheel_2 = Momentum_wheel_2 + alpha_2;//动量右轮
			Momentum_wheel_1 = myconstrain(Momentum_wheel_1,-8000,8000);
			Momentum_wheel_2 = myconstrain(Momentum_wheel_2,-8000,8000);
		}
		
		if(Circle_analy.control_FBwheel == ReSet)
			Searchangle();
		else if(Circle_analy.control_FBwheel == Set && AllFlag.Car_level == two)
		{
			AngleAnaly(0,5000,front_wheelmax,8192,Chassis_Moto_Info_Array[3].NowAngle);
			Front_wheel_angle = pid_handle3_test(&Total_pid,11,-Front_wheel_change,-wheel_data_Array[0].longtimeangle);
			Front_wheel = pid_handle3_test(&Total_pid,10,Front_wheel_angle,Chassis_Moto_Info_Array[3].NowSpeed);
			
			AngleAnaly(1,5000,back_wheelmax,8192,Chassis_Moto_Info_Array[2].NowAngle);
			Back_wheel_angle = pid_handle3_test(&Total_pid,13,-Back_wheel_change,-wheel_data_Array[1].longtimeangle);
			Back_wheel = pid_handle3_test(&Total_pid,12,Back_wheel_angle,Chassis_Moto_Info_Array[2].NowSpeed);
			ttr = 4;
		}
		

  }
	if(AllFlag.Stand_Flag == _normal || AllFlag.Stand_Flag == _normal2)
	{
//		Store_NextSpeed1 = Store_NextSpeed2 =0;
		if(RC_Ctl.rc.s2 == 3)
		{
		Front_wheel = Back_wheel = 0;//
		Momentum_wheel_1 =0;
		Momentum_wheel_2 = 0;
		}
		else if(RC_Ctl.rc.s2 == 2)
		{
		Front_wheel = Back_wheel = 0;//
//		Momentum_wheel_1 =0;
//		Momentum_wheel_2 = 0;
		}
		send_chassis_moto();
	}
	else if(AllFlag.Stand_Flag == _normal3)
	{
//		Store_NextSpeed1 = Store_NextSpeed2 =0;//
		if(RC_Ctl.rc.s2 == 3)
		{
		Front_wheel = Back_wheel = 0;//
		Momentum_wheel_1 =0;
		Momentum_wheel_2 = 0;
		}
		else if(RC_Ctl.rc.s2 == 2)
		{
		Front_wheel = Back_wheel = 0;//
//		Momentum_wheel_1 =0;
//		Momentum_wheel_2 = 0;
		}
		send_chassis_moto();
	}
		
}
u8 Send_chassis_Array[8];
u8 Send_chassis_Array_2[8];
void send_chassis_moto()
{
	Send_chassis_Array[0]=(Store_NextSpeed1>>8)&0xff;
	Send_chassis_Array[1]=Store_NextSpeed1&0xff;
	Send_chassis_Array[2] = (Store_NextSpeed2>>8)&0xff;//
	Send_chassis_Array[3] = Store_NextSpeed2&0xff;
	Send_chassis_Array[4] = (Momentum_wheel_1>>8)&0xff;
	Send_chassis_Array[5] = Momentum_wheel_1&0xff;
	Send_chassis_Array[6] = (Momentum_wheel_2>>8)&0xff;
	Send_chassis_Array[7] = Momentum_wheel_2&0xff;
	CAN1_Send_Msg_chassis(Send_chassis_Array,8);
	
	
	Send_chassis_Array_2[4] = (Front_wheel>>8)&0xff;
	Send_chassis_Array_2[5] = Front_wheel&0xff;
	Send_chassis_Array_2[6] = (Back_wheel>>8)&0xff;
	Send_chassis_Array_2[7] = Back_wheel&0xff;
	CAN1_Send_Msg_chassis_2(Send_chassis_Array_2,8);
}





