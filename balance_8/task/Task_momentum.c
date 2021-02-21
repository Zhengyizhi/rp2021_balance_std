#include "momentum_wheel.h"




Condition_order Ordor_Condition;
choose_Fed Choose_Fed;
extern short int Momentum_wheel_1,Momentum_wheel_2;//1_动量左轮，2_动量右轮
extern float gravity_2;//初始为0 代表变化重心
float balance_gyroy = 0;
float zhongzhi=0;
float balance_angle_1=0;
float balance_angle_2=0;
float Speed_Speed=0;


float velocity_speed_1=0;
float velocity_speed_2=0;
void Decide_Condition ()
{
//	if(mpu_data_Array[1].longtimeangle<-15)//前倒为负数
//	{
//		forward_Stand();
//	}
//	else if(mpu_data_Array[1].longtimeangle>15)
//	{
//		backward_Stand();
//	}
//	else ;
		normal();
}

#if (zhuanzhi == 0)
float weizhi1=7.5;
float weizhi2 = 180;

void forward_Stand()
{
	if(Ordor_Condition.momentum_order == level_0)
		Ordor_Condition.momentum_order = level_1;
	if(Ordor_Condition.momentum_order == level_1)//读取稳定角度
	{
		static float angle_last;
		static float angle_deta;      //角度变化量
		angle_deta *= 0.8f;
		angle_deta += (angle_last - pitch) *0.2f;
		if(abs(angle_deta)<0.03f)
			Ordor_Condition.momentum_order = level_2;
		angle_last = pitch;
	}
	else if(Ordor_Condition.momentum_order == level_2)//根据倒下的角度确定速度
	{
		float speed_deta;
		Momentum_wheel_1 = Incremental_PI(&Total_pid,17,Chassis_Moto_Info_Array[1].NowSpeed,-mpu_data_Array[1].longtimeangle * weizhi1);//输出速度为正.实际右轮
		speed_deta = -mpu_data_Array[1].longtimeangle * weizhi1 - Chassis_Moto_Info_Array[1].NowSpeed;
		if(abs(speed_deta)<5)
		{
//			Ordor_Condition.momentum_order = level_3;
//			clear_PID(&Total_pid,17);
		}
	}
	else if(Ordor_Condition.momentum_order == level_3)
	{
		static int i=0;
		if(mpu_data_Array[1].longtimeangle<-10)//还是起不来
		{
			Momentum_wheel_1 = - Chassis_Moto_Info_Array[1].NowSpeed * weizhi2;
			i++;
			if(i>400)
			{
				i=0;
				Ordor_Condition.momentum_order = level_0;//起不来也跳出去重新来
			}
		}
		else //起来了
			Ordor_Condition.momentum_order = level_0;
	}
}


void backward_Stand()
{
	if(Ordor_Condition.momentum_order == level_0)
		Ordor_Condition.momentum_order = level_4;
	if(Ordor_Condition.momentum_order == level_4)//读取稳定角度
	{
		static float angle_last;
		static float angle_deta;      //角度变化量
		angle_deta *= 0.8f;
		angle_deta += (angle_last - pitch) *0.2f;
		if(abs(angle_deta)<0.03f)
			Ordor_Condition.momentum_order = level_5;
		angle_last = pitch;
	}
	else if(Ordor_Condition.momentum_order == level_5)//根据倒下的角度确定速度
	{
		float speed_deta;
		Momentum_wheel_1 = Incremental_PI(&Total_pid,17,Chassis_Moto_Info_Array[1].NowSpeed,-mpu_data_Array[1].longtimeangle * weizhi1);//输出速度为正.实际右轮
		speed_deta = -mpu_data_Array[1].longtimeangle * weizhi1 - Chassis_Moto_Info_Array[1].NowSpeed;
		if(abs(speed_deta)<5)
		{
			Ordor_Condition.momentum_order = level_6;
			clear_PID(&Total_pid,17);
		}
	}
	else if(Ordor_Condition.momentum_order == level_6)
	{
		static int i=0;
		if(mpu_data_Array[1].longtimeangle>10)//还是起不来
		{
			Momentum_wheel_1 = - Chassis_Moto_Info_Array[1].NowSpeed * weizhi2;
			i++;
			if(i>400)
			{
				i=0;
				Ordor_Condition.momentum_order = level_0;//起不来也跳出去重新来
			}
		}
		else //起来了
			Ordor_Condition.momentum_order = level_0;
	}
}

void normal(void)
{
//		if(Chassis_Moto_Info_Array[1].NowSpeed>=2000)
//			zhongzhi+=0.001f;
//		else if(Chassis_Moto_Info_Array[1].NowSpeed<=-2000)
//			zhongzhi-=0.001f;
//		if(Chassis_Moto_Info_Array[1].NowSpeed>=4000)
//			zhongzhi+=0.002f;
//		else if(Chassis_Moto_Info_Array[1].NowSpeed<=-4000)
//			zhongzhi-=0.002f;
//		if(Chassis_Moto_Info_Array[1].NowSpeed>=6000)
//			zhongzhi+=0.005f;
//		else if(Chassis_Moto_Info_Array[1].NowSpeed<=-6000)
//			zhongzhi-=0.005f;
		balance_angle = balance_momentum(&Total_pid,18,pitch,gyroy,zhongzhi);
		velocity_speed = velocity_momentum(&Total_pid,19,Chassis_Moto_Info_Array[1].NowSpeed);
	Momentum_wheel_1 = balance_angle+velocity_speed;
	Momentum_wheel_2 = -balance_angle;
}

#elif (zhuanzhi ==1)


void normal(void)
{
	balance_angle_1 = -balance_momentum(&Total_pid,16,pitch,-gyroy,zhongzhi);
  Momentum_wheel_1 = balance_angle_1;
	Momentum_wheel_2 = -balance_angle_1;
}

#elif (zhuanzhi ==2)

void normal(void)
{

//	if(abs(zhongzhi - pitch)>=5)
//	{
		Speed_Speed = -balance_momentum(&Total_pid,16,pitch,-gyroy,zhongzhi);
		clear_PID(&Total_pid,21);
		clear_PID(&Total_pid,20);
		Momentum_wheel_1 = Speed_Speed;
		Momentum_wheel_2 = -Momentum_wheel_1;
//	}
//	else 
//	{
//		balance_angle_1 = pid_handle3_test(&Total_pid,20,zhongzhi,pitch);
//		velocity_speed_1 = balanstand_momentum(&Total_pid,21,balance_angle_1,gyroy,0,zhongzhi,-30,30);//+gyro负反馈

//		clear_PID(&Total_pid,16);
//		Momentum_wheel_1 = velocity_speed_1 ;
//		Momentum_wheel_2 = -velocity_speed_1;
//	}
}
int tttt;
void choose_feedback()
{
	//纵
	Choose_Fed.goal_angle = RC_Ctl.rc.ch3 * chassis_remoteforward;
	zhongzhi = - RC_Ctl.rc.ch3 * chassis_remoteforward;
	Choose_Fed.now_angle = -pitch;
	if(abs(Choose_Fed.goal_angle)<=2 && abs(Choose_Fed.now_angle)<=5 && abs(RC_Ctl.rc.ch2)<=2)
	{
			none_feedback();//原地静止
	}
	else if(abs(Choose_Fed.goal_angle)<=2 && abs(Choose_Fed.now_angle)<=2 && abs(RC_Ctl.rc.ch2)>2)
	{
			negative_feedback();//原地旋转
	}
	else if(Choose_Fed.goal_angle - pitch < -7)
	{
		if(gyroy >20)
		{
			positive_feedback();
		}
		else if(gyroy < -1300)
		{
			negative_feedback();
		}
//		else 
//		{
//			none_feedback();
//		}
	}
	else if(Choose_Fed.goal_angle - pitch > 7)
	{
		if(gyroy < -20)
		{
			positive_feedback();
		}
		else if(gyroy > 1300)
		{
			negative_feedback();
		}
//		else 
//		{
//			none_feedback();
//		}
	}
	else if (abs (Choose_Fed.goal_angle - pitch)<=7)
	{
		negative_feedback();
	}
		



}
void  none_feedback()
{
	Momentum_wheel_1 = Momentum_wheel_2 = 0;
	tttt = 0;
	clear_PID(&Total_pid,16);
	clear_PID(&Total_pid,21);
	clear_PID(&Total_pid,20);
}

void negative_feedback()
{
	velocity_speed_1 = balanstand_momentum(&Total_pid,21,balance_angle_1,gyroy,0,zhongzhi,-30,30);//+gyro负反馈
	tttt = 1;
//	clear_PID(&Total_pid,16);
	Momentum_wheel_1 = velocity_speed_1 ;
	Momentum_wheel_2 = -velocity_speed_1;
}
void positive_feedback()
{
	Speed_Speed = -balance_momentum(&Total_pid,16,pitch,-gyroy,zhongzhi);
	tttt = 2;
//	clear_PID(&Total_pid,21);
//	clear_PID(&Total_pid,20);
	Momentum_wheel_1 = Speed_Speed;
	Momentum_wheel_2 = -Momentum_wheel_1;

}



#endif



