#include "Choose_task.h"


choose_Task Task_Choose;

int choose_i,choose_j;
float regis_grav;
void choose_task_1()
{
	if(AllFlag.DMAinterrupt_Flag == _normal&& AllFlag.Can1interrupt_Flag == _normal)
	{
		if(RC_Ctl.rc.s2 == 1)
		{
			Circle_analy.Speedrun_circle = ReSet;
			choose_task();
		}
		else 
		{
			choose_i = choose_j = 0;
			regis_grav = gravity;
			Circle_analy.Speedrun_circle = Set;
		}
	}
}


void choose_task()
{
	if (abs(RC_Ctl.rc.ch3)<=2 && abs(RC_Ctl.rc.ch2)<=2 && abs(pitch)<=30 )
	{
		choose_i++;
		if(choose_i>=80)
		{
			choose_i=0;
			Task_Choose.shift_grav_test = shift_gravity_open;//�ɼ������
		}
	}
	else
	{
		Task_Choose.shift_grav_test = shift_gravity_close;//���ɼ������
		choose_j=0;
	}
	
//	if(abs(gravity)>=10)
//	{
//		Task_Choose.shift_grav_test = shift_gravity_close;
//	}
	 if(Task_Choose.shift_grav_test == shift_gravity_open)
	{
		if(Chassis_Moto_Info_Array[1].NowSpeed >= -400 && Chassis_Moto_Info_Array[1].NowSpeed < -130 //����ǰƫ
		&& Chassis_Moto_Info_Array[0].NowSpeed <= 400 && Chassis_Moto_Info_Array[0].NowSpeed >130 )//1����ǰ��-��0����ǰ��+
		{
			choose_j++;
			if(choose_j>=15)
			{
				gravity += 0.007f;
			}
			if(choose_j>=30)
			{
				choose_j =30;
			}
		}
		else if (Chassis_Moto_Info_Array[1].NowSpeed >= -1300 && Chassis_Moto_Info_Array[1].NowSpeed < -400 //����ǰƫ
		&& Chassis_Moto_Info_Array[0].NowSpeed <= 1300 && Chassis_Moto_Info_Array[0].NowSpeed >400 )//1����ǰ��-��0����ǰ��+
		{
			choose_j++;
			if(choose_j>=10)
			{
				gravity += 0.08f;
			}
			if(choose_j>=30)
			{
				choose_j =30;
			}
		}//ǰƫ
		else if (Chassis_Moto_Info_Array[1].NowSpeed < -1300  //����ǰƫ
		&& Chassis_Moto_Info_Array[0].NowSpeed > 1300 )//1����ǰ��-��0����ǰ��+
		{
			choose_j++;
			if(choose_j>=5)
			{
				gravity += 0.13f;
			}
			if(choose_j>=30)
			{
				choose_j =30;
			}
		}//ǰƫ
		
		
		else if(Chassis_Moto_Info_Array[0].NowSpeed >= -400 && Chassis_Moto_Info_Array[0].NowSpeed < -130 //���ĺ�ƫ
		&& Chassis_Moto_Info_Array[1].NowSpeed <= 400 && Chassis_Moto_Info_Array[1].NowSpeed >130 )//1����ǰ��-��0����ǰ��+
		{
			choose_j--;
			if(choose_j<=-15)
			{
				gravity -= 0.007f;
			}
			if(choose_j<=-30)
			{
				choose_j = -30;
			}
		}
		else if (Chassis_Moto_Info_Array[0].NowSpeed >= -1300 && Chassis_Moto_Info_Array[0].NowSpeed < -400 //���ĺ�ƫ
		&& Chassis_Moto_Info_Array[1].NowSpeed <= 1300 && Chassis_Moto_Info_Array[1].NowSpeed >400 )//1����ǰ��-��0����ǰ��+
		{
			choose_j--;
			if(choose_j<=-10)
			{
				gravity -= 0.08f;
			}
			if(choose_j<=-30)
			{
				choose_j = -30;
			}
		}//��ƫ
		else if (Chassis_Moto_Info_Array[0].NowSpeed < -1300  //���ĺ�ƫ
		&& Chassis_Moto_Info_Array[1].NowSpeed > 1300 )//1����ǰ��-��0����ǰ��+
		{
			choose_j--;
			if(choose_j<=-5)
			{
				gravity -= 0.13f;
			}
			if(choose_j<=-30)
			{
				choose_j = -30;
			}
		}//��ƫ
		
//		else
//			Circle_analy.Speedrun_circle = Set;
	}



}

