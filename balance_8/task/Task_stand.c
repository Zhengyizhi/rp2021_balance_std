#include "stand.h"
float front_wheelmin,front_wheelmax,back_wheelmin,back_wheelmax;
_Receive wheel_data_Array[2];//0-前轮，1-后轮

float detaerrorlimit = 5;
void AngleAnaly(char num,float jump,float center,float degree,float Nowangle)
{
	wheel_data_Array[num].NowAngle = Nowangle;
	wheel_data_Array[num].detaNowAngle = wheel_data_Array[num].NowAngle - wheel_data_Array[num].preAngle;
	if(wheel_data_Array[num].detaNowAngle <= -jump)
		wheel_data_Array[num].longtimecircle ++;
	else if(wheel_data_Array[num].detaNowAngle >= jump)//反转
		wheel_data_Array[num].longtimecircle --;
	wheel_data_Array[num].longtimeangle = -degree * wheel_data_Array[num].longtimecircle + (center - wheel_data_Array[num].NowAngle);
	wheel_data_Array[num].preAngle = wheel_data_Array[num].NowAngle;
}
short int Front_wheel,Back_wheel;

void Searchangle()
{
	static int i,j;
	//前轮弄好
	if(Circle_analy.front_wheel == _not)
	{
		Front_wheel = (short int)(pid_handle3_test(&Total_pid,10,3000,Chassis_Moto_Info_Array[3].NowSpeed));
		if(abs(Total_pid.pid_test_Array[10].detaerror)<=detaerrorlimit && abs(Front_wheel)>0)
		{
			i++;
			if(i>=70)
			{
				Circle_analy.front_wheel = _OK;
				front_wheelmax = Chassis_Moto_Info_Array[3].NowAngle;
				wheel_data_Array[0].preAngle = front_wheelmax;
			}
		}
	}
	else if (Circle_analy.front_wheel == _OK)
	{
		AngleAnaly(0,5000,front_wheelmax,8192,Chassis_Moto_Info_Array[3].NowAngle);
		clear_PID(&Total_pid,10);
	}
	//后轮弄好
	if(Circle_analy.back_wheel == _not)
	{
		Back_wheel = (short int)(pid_handle3_test(&Total_pid,12,3000,Chassis_Moto_Info_Array[2].NowSpeed));
		if(abs(Total_pid.pid_test_Array[12].detaerror)<=detaerrorlimit&& abs(Back_wheel)>0)
		{
			j++;
			if(j>=70)
			{
				Circle_analy.back_wheel = _OK;
				back_wheelmax = Chassis_Moto_Info_Array[2].NowAngle;
				wheel_data_Array[1].preAngle = back_wheelmax;
			}
		}
	}
	else if(Circle_analy.back_wheel == _OK)
	{
		AngleAnaly(1,5000,back_wheelmax,8192,Chassis_Moto_Info_Array[2].NowAngle);
		clear_PID(&Total_pid,12);
	}
	
	
	if(Circle_analy.back_wheel == _OK && Circle_analy.front_wheel == _OK)
	{
		Circle_analy.control_FBwheel = Set;
	}
}





