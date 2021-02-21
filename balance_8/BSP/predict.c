#include "math.h"
#include "system.h"
void kalmanCreate2(kalman *p,float T_Q,float T_R)
{
    //kalman* p = ( kalman*)malloc(sizeof( kalman));
    p->X_last = (float)0;
    p->P_last = 0;
    p->Q = T_Q;
    p->R = T_R;
    p->A = 1;
    p->H = 1;
    p->X_mid = p->X_last;
    //return p;
}
float KalmanFilter(kalman* p,float dat)
{
    p->X_mid =p->A*p->X_last;                     //x(k|k-1) = AX(k-1|k-1)+BU(k)
    p->P_mid = p->A*p->P_last+p->Q;               //p(k|k-1) = Ap(k-1|k-1)A'+Q
    p->kg = p->P_mid/(p->P_mid+p->R);             //kg(k) = p(k|k-1)H'/(Hp(k|k-1)'+R)
    p->X_now = p->X_mid+p->kg*(dat-p->X_mid);     //x(k|k) = X(k|k-1)+kg(k)(Z(k)-HX(k|k-1))
    p->P_now = (1-p->kg)*p->P_mid;                //p(k|k) = (I-kg(k)H)P(k|k-1)
    p->P_last = p->P_now;                         //状态更新
    p->X_last = p->X_now;
    return p->X_now;
}

//void KalmanFilter(void *struct,float input)
//{
//	struct = struct;
//	/*一阶卡尔曼滤波*/
//}

typedef struct
{
	uint16_t nowLength;
	uint16_t queueLength;
	//长度
	float queue[100];
	//指针

}QueueObj;
//队列对象


QueueObj target_speed=
{
	.nowLength =0,
	.queueLength = 100,
	.queue = {0}
};

/**
* @brief 获取目标的速度
* @param void
* @return void
*	以队列的逻辑
*/
float Get_Target_Speed(uint8_t queue_len,float angle)
{
	float sum=0;
	float tmp=0;
	
	if(queue_len>target_speed.queueLength)
		queue_len=target_speed.queueLength;
	//防止溢出
	
	
	if(target_speed.nowLength<queue_len)
	{
		//队列未满，只进不出
		target_speed.queue[target_speed.nowLength] = angle;
		target_speed.nowLength++;
	}
	else
	{
		//队列已满，FIFO。
		for(uint16_t i=0;i<queue_len-1;i++)
		{
			target_speed.queue[i] = target_speed.queue[i+1];
			//更新队列
		
		}
		target_speed.queue[queue_len-1] = angle;
	}
	
	//更新完队列
	
	
	for(uint16_t j=0;j<queue_len;j++)
	{
		sum+=target_speed.queue[j];
	}
	tmp = sum/(queue_len/1.f);
	
	tmp = (angle - tmp);	
	
	return tmp;
}



QueueObj target_accel=
{
	.nowLength =0,
	.queueLength = 100,
	.queue = {0}
};

/**
* @brief 获取目标的加速度
* @param void
* @return void
*	以队列的逻辑
*/
float Get_Target_Accel(uint8_t queue_len,float speed)
{
	float sum=0;
	float tmp=0;
	
	if(queue_len>target_accel.queueLength)
		queue_len=target_accel.queueLength;
	//防止溢出
	
	
	if(target_accel.nowLength<queue_len)
	{
		//队列未满，只进不出
		target_accel.queue[target_accel.nowLength] = speed;
		target_accel.nowLength++;
	}
	else
	{
		//队列已满，FIFO。
		for(uint16_t i=0;i<queue_len-1;i++)
		{
			target_accel.queue[i] = target_accel.queue[i+1];
			//更新队列
		
		}
		target_accel.queue[queue_len-1] = speed;
	}
	
	//更新完队列
	
	
	for(uint16_t j=0;j<queue_len;j++)
	{
		sum+=target_accel.queue[j];
	}
	tmp = sum/(queue_len/1.f);
	
	tmp = (speed - tmp);	
	
	return tmp;
}



void Vision_Task(void)
{
	static uint16_t ramp_cnt=0;/*斜坡计数 -- 用于缓和掉帧影响*/			static bool judge_flag=false;/*丢失标志位激活标志位,用于一次性操作*/
	
	static uint16_t predict_delay=0;/*预测的延时函数*/		static uint16_t active_cnt=0,lost_cnt=0;/*激活计数/丢失计数--用于识别和未识别到相互切换的过程*/
	
	static uint32_t vision_this_time,vision_last_time;/*视觉接受的时间*/					static uint8_t last_identify_flag=0;/*上一次的接收标志位*/
	
	static float now_vision_yaw=0;/*当前获得的视觉yaw角度--相对坐标角度*/				uint32_t outline_cnt=0;	/*离线计数*/
	
	static uint16_t last_mech_yaw=0,now_mech_yaw=0;/*当前和上次的机械yaw角度 -- 云台角度*/

	static bool init=true;/*初始化标志位*/						
	
	if(init == true)
	{
		start_mech_yaw = sensor.yaw.mech_angle;
		last_mech_yaw = start_mech_yaw;
		init = false;
		//初始化过程
	}
	/*对变量进行初始化赋值，不加也影响不大*/ 
	
	now_mech_yaw = sensor.yaw.mech_angle; /*当前的yaw轴机械角度值*/
	/*这里的sensor是传感器数据的结构体，这里主要是为了更新yaw轴云台电机的机械角度*/ 

	delta_mech_yaw += now_mech_yaw - last_mech_yaw; 	/*计算两次采样的机械角度差值*/
	/*这里是计算两次采样的机械角度之间的差值*/ 
	
	cloud_yaw_raw = delta_mech_yaw;  /*进入卡尔曼滤波*/
	/*这里是为了将差值进行卡尔曼滤波*/
	 
	last_mech_yaw = now_mech_yaw; /*记录上一次的角度值*/
	/*更新记录上一次采样的机械角度*/ 
	
	cloud_yaw_kf = KalmanFilter(&kalman_cloudYaw,cloud_yaw_raw)+start_mech_yaw;	/*滤波*/       cloud_degree=cloud_yaw_kf/22.253f;  
	/*这里是将前后两次采样的差值进行卡尔曼滤波后再加上起始角度，来作为滤波后的云台机械角度*/ 
	
	/*
		这里这么做是为了避开机械角度的边界问题，这个方法同样可以解决陀螺仪产生的边界而导致无法直接对角度数据进行滤波的问题。 你细细品 
	*/	
	
	/*以上是对视觉原始数据的处理*/
	
	if(vision_update_flag == true)	/*视觉更新了数据*/
	{
		/*当视觉数据更新后进来*/		
		
		if(last_identify_flag != vision_data.identifyTarget)	/*识别的状态发生了切换*/
		{
			/*若发生了掉帧或刚识别到目标或丢失了目标，则进到这里*/ 
			
			ramp_cnt = RAMP_MAX_CNT;	/*切换后斜坡过渡*/
			/*这里原本的ramp_cnt为0.现在赋值为不为0的数，表示要对数据进行斜坡处理*/ 
			
			judge_flag = true;			/*进入判定过程*/
			/*触发识别状态的判定条件，即当出现识别模式的切换时，开始进入判断过程，判断是否真的跟丢或者只是掉帧或者刚识别到目标*/ 
			
			
			//掉帧后重新判断识别状态
		}
		
		if(judge_flag == true)		/*识别到了目标*/
		{
			/*
				这里的内容是哨兵侦察和识别之间的切换代码，实际上其他兵种可以不看，根据需要写上自己的切换过程代码。不过也可以看一下 参考一下思路，你细细品。 
			*/ 
			
			if(vision_data.identifyTarget == 1)		/*表示为发生了掉帧重新进入状态判定过程--这里不影响继续跟踪识别-因为模式没有改变*/
			{
				active_cnt++; 	/*活跃计数*/
				
				if(active_cnt >= ACTIVE_MAX_CNT) /*达到阈值，认定为识别到*/
				{
					judge_flag = false;
					
					gimbal_mode = ATTACK;
				
					predict_delay = PREDICT_DELAY_CNT;  /*用于延迟超前角的加入时间*/
					
					active_cnt = 0;
					
					lost_cnt = 0;
					/*重新确认进来的判断*/
				}	
			}
			else
			{
				lost_cnt++;	 		/*丢失计数*/
				
				if(lost_cnt >= LOST_MAX_CNT) /*达到阈值，认定为丢失*/
				{
					judge_flag = false;
					
					gimbal_mode = SCOUT; 	
					
					active_cnt = 0;
					
					lost_cnt = 0;					
					/*侦察模式的切换*/
				}
			}
			/*进入判定后不侦察*/	
			 
			/*到这里是侦察和识别的切换*/ 
		}
	
	
		/*↑↑↑ 模式判定过程 ↑↑↑*/
	
		/*
			下面就是斜坡的过程了
			如果发生了切换的情况，则这里会进入斜坡并持续一小段时间，来缓和由于掉帧带来的冲击。具体的根据实际情况采用，可不用， 
		*/ 
		if(ramp_cnt > 0)
		{
			/*斜坡过渡*/
			now_vision_yaw = RampFloat(abs(vision_data.yawAngle - now_vision_yaw)/ramp_cnt,vision_data.yawAngle,now_vision_yaw);
			ramp_cnt--;
		}
		else
			now_vision_yaw = vision_data.yawAngle;                          vision_degree = vision_data.yawAngle;
		/* ↑↑↑获取当前的视觉角度↑↑↑ */ 
		
		update_cloud_yaw = cloud_yaw_raw+start_mech_yaw; 			/*视觉数据更新时的云台角度*/
		/*视觉数据更新时的云台数据*/ 
		
		vision_yaw_raw = now_vision_yaw * 22.253f;	/*变换为机械角度的尺度*/    
		/*视觉的最新数据，转换成和机械角度的同一个尺度*/ 
		
		
		vision_yaw_kf = KalmanFilter(&kalman_visionYaw,vision_yaw_raw); 	/*对视觉角度数据做卡尔曼滤波*/  
		/*视觉数据的卡尔曼滤波*/ 
		
		
		vision_dis_raw = vision_data.distance/1000.f;	/*计算距离*/
		/*获取距离数据*/  
	
	
		vision_dis_kf = KalmanFilter(&kalman_dist,vision_dis_raw);	/*滤波*/
		/*距离数据卡尔曼滤波器*/ 
		
		
		vision_this_time = millis();		/*获取当前时间*/
		/*更新视觉更新的时间 -- 在最后有用到*/ 
		
		vision_update_flag = false;		/*清除标志位*/	
		
		/*之所以将数据更新的内容放在更新标志位里，是为了保证预测数据可以不受云台自身运动的影响。
		仔细说了就是因为数据的更新频率远小于云台的角度数据，在视觉没更新的时间点里，云台的数据一直在更新。
		这导致了云台的运动会给预测的速度带来扰动数据。这里的处理就是为了尽可能地避免这里地噪声 
		*/ 
		
	}
	/*↑↑↑数据更新的处理↑↑↑*/
		
	target_yaw_raw = -vision_yaw_kf*k_scale_vision + cloud_yaw_kf;		/*对目标的期望角度*/
	/*目标的位置的yaw角度预测，这里的符号是因为器件安装包括云台和摄像头的安装之间的关系。具体看自己的安装关系。*/ 


	target_yaw_kf = KalmanFilter(&kalman_targetYaw,target_yaw_raw);		/*对目标的yaw角度进行定位*/   target_degree=target_yaw_kf/22.253f;  
	/*对目标角度进行滤波*/ 
	
	
	now_cal_yaw = -vision_yaw_raw*k_scale_vision + update_cloud_yaw;	 /*目标的角度 -- 用于计算目标速度*/
	/*开始预测速度，这里的k_scale_vision是摄像头角度变化的转化系数。
	
		举个例子，摄像头的视觉数据变化了10°，但是实际上云台转过了15°，那这个时候就需要给视觉的数据乘上一个1.5，此时的k_scale_vision就为1.5 
	*/ 
	
	
	target_speed_raw = Get_Target_Speed(queue_speed_length,now_cal_yaw)*k_speed; 	/*计算出对目标速度的预测*/
	/*开始对目标的速度进行预测，这个函数直接参考我给的代码。k_speed是缩放系数，因为得到的速度数据比较小，因此放大后放到卡尔曼比较合适*/ 
	
	
	if(vision_data.identifyTarget == 1)				/*计算速度值*/
		target_speed_kf = KalmanFilter(&kalman_speedYaw,target_speed_raw);
	else
		target_speed_kf = KalmanFilter(&kalman_speedYaw,0);			
	
	/*参考去年的做法。*/ 
	
	
	target_accel_raw = Get_Target_Accel(10,target_speed_kf);	 /*获取加速度*/
	
	target_accel_raw = myDeathZoom(0,0.1,target_accel_raw);		/*死区处理 - 滤除0点附近的噪声*/
	
	target_accel_kf = KalmanFilter(&kalman_accel,target_accel_raw);		/*卡尔曼滤波*/

	/*上面三行是解算加速度的数据，跟速度的解算原理一样，不多做解释*/		
		
	if(predict_delay>0)
	{
		/*预测的延时*/
		predict_delay--;
		predict_angle_raw=0;
		
		/*这里predict的延时跟去年的代码有点像，都是为了让云台先摆到目标附近在进行预测用的，
		否则会导致云台晃动，原因就是因为云台晃动会给预测的速度带来影响，从而形成正反馈的数据震荡过程。 */ 
	}
	else
	{
		/*延时结束后，进入预测过程*/
		float tmp1;	 /*临时变量*/
		
		if((target_accel_kf*target_speed_kf)>=0)
		{
			/*加速度和速度同向时*/
			tmp1 = 0.8f; 
		}
		else
		{
			tmp1 = -1.2f;
		}
		
		/*这里用来判断加速度和速度关系，来决定加速度得作用方向
			
			例如 加速度和速度反向时，说明减速运动
				加速度和速度同向时，说明加速运动。 
		*/ 
		
		feedforward_angle = k_ff * target_accel_kf; 	/*计算前馈角*/
		
		feedforward_angle = constrain(feedforward_angle,-4.f,4.f);	 /*前馈角*/
		
		/*前馈角度时通过加速度来对目标运动的提前反应。*/ 
		
		predict_angle_raw = k_pre*target_speed_kf*vision_dis_kf + tmp1*feedforward_angle*use_ff;	/*计算预测角度*/
	}
	predict_angle_raw = constrain(predict_angle_raw,-8,8);	/*限幅*/
	
	predict_angle = RampFloat((abs(predict_angle_raw - predict_angle)/predict_ramp_cnt),predict_angle_raw,predict_angle); 	/*斜坡处理*/
	
	/*这里是对预测的滤波*/ 
	
	attack_pitch_offset = k_pitch * vision_dis_kf; 	/*抬头距离补偿*/
	/*对于距离的pitch角度抬头，但是没有加入对pitch方向的预测。后续补充上。*/ 
	
	
	/*以上是控制所需要的角度计算过程*/
	
	outline_cnt = millis() - vision_this_time; 	/*离线计数*/
	/*计算视觉最近一帧的更新时间，来判断是否已经离线*/ 
	
	
	if(outline_cnt >= 1000)
	{
		/*离线*/
		vision_data.identifyTarget = 0;
		vision_data.yawAngle = 0;
		vision_data.pitchAngle = 0;
		vision_data.distance = 0;
		gimbal_mode = SCOUT;
		vision_online_flag = false;		
	}
	else
	{
		vision_online_flag=true;
	}
	/*判定是否离线*/
	/*离线要有对应的处理，否则云台暴走。*/ 
	
	
	if(vision_this_time != vision_last_time)
	{
		/*计算帧率*/
		vision_update_fps = vision_this_time - vision_last_time;
		
		vision_last_time = vision_this_time;
		
		/*计算帧率*/ 
	}
	
	last_identify_flag = vision_data.identifyTarget;
	//记录上一次标志位
}
