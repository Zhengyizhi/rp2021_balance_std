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
    p->P_last = p->P_now;                         //״̬����
    p->X_last = p->X_now;
    return p->X_now;
}

//void KalmanFilter(void *struct,float input)
//{
//	struct = struct;
//	/*һ�׿������˲�*/
//}

typedef struct
{
	uint16_t nowLength;
	uint16_t queueLength;
	//����
	float queue[100];
	//ָ��

}QueueObj;
//���ж���


QueueObj target_speed=
{
	.nowLength =0,
	.queueLength = 100,
	.queue = {0}
};

/**
* @brief ��ȡĿ����ٶ�
* @param void
* @return void
*	�Զ��е��߼�
*/
float Get_Target_Speed(uint8_t queue_len,float angle)
{
	float sum=0;
	float tmp=0;
	
	if(queue_len>target_speed.queueLength)
		queue_len=target_speed.queueLength;
	//��ֹ���
	
	
	if(target_speed.nowLength<queue_len)
	{
		//����δ����ֻ������
		target_speed.queue[target_speed.nowLength] = angle;
		target_speed.nowLength++;
	}
	else
	{
		//����������FIFO��
		for(uint16_t i=0;i<queue_len-1;i++)
		{
			target_speed.queue[i] = target_speed.queue[i+1];
			//���¶���
		
		}
		target_speed.queue[queue_len-1] = angle;
	}
	
	//���������
	
	
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
* @brief ��ȡĿ��ļ��ٶ�
* @param void
* @return void
*	�Զ��е��߼�
*/
float Get_Target_Accel(uint8_t queue_len,float speed)
{
	float sum=0;
	float tmp=0;
	
	if(queue_len>target_accel.queueLength)
		queue_len=target_accel.queueLength;
	//��ֹ���
	
	
	if(target_accel.nowLength<queue_len)
	{
		//����δ����ֻ������
		target_accel.queue[target_accel.nowLength] = speed;
		target_accel.nowLength++;
	}
	else
	{
		//����������FIFO��
		for(uint16_t i=0;i<queue_len-1;i++)
		{
			target_accel.queue[i] = target_accel.queue[i+1];
			//���¶���
		
		}
		target_accel.queue[queue_len-1] = speed;
	}
	
	//���������
	
	
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
	static uint16_t ramp_cnt=0;/*б�¼��� -- ���ڻ��͵�֡Ӱ��*/			static bool judge_flag=false;/*��ʧ��־λ�����־λ,����һ���Բ���*/
	
	static uint16_t predict_delay=0;/*Ԥ�����ʱ����*/		static uint16_t active_cnt=0,lost_cnt=0;/*�������/��ʧ����--����ʶ���δʶ���໥�л��Ĺ���*/
	
	static uint32_t vision_this_time,vision_last_time;/*�Ӿ����ܵ�ʱ��*/					static uint8_t last_identify_flag=0;/*��һ�εĽ��ձ�־λ*/
	
	static float now_vision_yaw=0;/*��ǰ��õ��Ӿ�yaw�Ƕ�--�������Ƕ�*/				uint32_t outline_cnt=0;	/*���߼���*/
	
	static uint16_t last_mech_yaw=0,now_mech_yaw=0;/*��ǰ���ϴεĻ�еyaw�Ƕ� -- ��̨�Ƕ�*/

	static bool init=true;/*��ʼ����־λ*/						
	
	if(init == true)
	{
		start_mech_yaw = sensor.yaw.mech_angle;
		last_mech_yaw = start_mech_yaw;
		init = false;
		//��ʼ������
	}
	/*�Ա������г�ʼ����ֵ������ҲӰ�첻��*/ 
	
	now_mech_yaw = sensor.yaw.mech_angle; /*��ǰ��yaw���е�Ƕ�ֵ*/
	/*�����sensor�Ǵ��������ݵĽṹ�壬������Ҫ��Ϊ�˸���yaw����̨����Ļ�е�Ƕ�*/ 

	delta_mech_yaw += now_mech_yaw - last_mech_yaw; 	/*�������β����Ļ�е�ǶȲ�ֵ*/
	/*�����Ǽ������β����Ļ�е�Ƕ�֮��Ĳ�ֵ*/ 
	
	cloud_yaw_raw = delta_mech_yaw;  /*���뿨�����˲�*/
	/*������Ϊ�˽���ֵ���п������˲�*/
	 
	last_mech_yaw = now_mech_yaw; /*��¼��һ�εĽǶ�ֵ*/
	/*���¼�¼��һ�β����Ļ�е�Ƕ�*/ 
	
	cloud_yaw_kf = KalmanFilter(&kalman_cloudYaw,cloud_yaw_raw)+start_mech_yaw;	/*�˲�*/       cloud_degree=cloud_yaw_kf/22.253f;  
	/*�����ǽ�ǰ�����β����Ĳ�ֵ���п������˲����ټ�����ʼ�Ƕȣ�����Ϊ�˲������̨��е�Ƕ�*/ 
	
	/*
		������ô����Ϊ�˱ܿ���е�Ƕȵı߽����⣬�������ͬ�����Խ�������ǲ����ı߽�������޷�ֱ�ӶԽǶ����ݽ����˲������⡣ ��ϸϸƷ 
	*/	
	
	/*�����Ƕ��Ӿ�ԭʼ���ݵĴ���*/
	
	if(vision_update_flag == true)	/*�Ӿ�����������*/
	{
		/*���Ӿ����ݸ��º����*/		
		
		if(last_identify_flag != vision_data.identifyTarget)	/*ʶ���״̬�������л�*/
		{
			/*�������˵�֡���ʶ��Ŀ���ʧ��Ŀ�꣬���������*/ 
			
			ramp_cnt = RAMP_MAX_CNT;	/*�л���б�¹���*/
			/*����ԭ����ramp_cntΪ0.���ڸ�ֵΪ��Ϊ0��������ʾҪ�����ݽ���б�´���*/ 
			
			judge_flag = true;			/*�����ж�����*/
			/*����ʶ��״̬���ж���������������ʶ��ģʽ���л�ʱ����ʼ�����жϹ��̣��ж��Ƿ���ĸ�������ֻ�ǵ�֡���߸�ʶ��Ŀ��*/ 
			
			
			//��֡�������ж�ʶ��״̬
		}
		
		if(judge_flag == true)		/*ʶ����Ŀ��*/
		{
			/*
				������������ڱ�����ʶ��֮����л����룬ʵ�����������ֿ��Բ�����������Ҫд���Լ����л����̴��롣����Ҳ���Կ�һ�� �ο�һ��˼·����ϸϸƷ�� 
			*/ 
			
			if(vision_data.identifyTarget == 1)		/*��ʾΪ�����˵�֡���½���״̬�ж�����--���ﲻӰ���������ʶ��-��Ϊģʽû�иı�*/
			{
				active_cnt++; 	/*��Ծ����*/
				
				if(active_cnt >= ACTIVE_MAX_CNT) /*�ﵽ��ֵ���϶�Ϊʶ��*/
				{
					judge_flag = false;
					
					gimbal_mode = ATTACK;
				
					predict_delay = PREDICT_DELAY_CNT;  /*�����ӳٳ�ǰ�ǵļ���ʱ��*/
					
					active_cnt = 0;
					
					lost_cnt = 0;
					/*����ȷ�Ͻ������ж�*/
				}	
			}
			else
			{
				lost_cnt++;	 		/*��ʧ����*/
				
				if(lost_cnt >= LOST_MAX_CNT) /*�ﵽ��ֵ���϶�Ϊ��ʧ*/
				{
					judge_flag = false;
					
					gimbal_mode = SCOUT; 	
					
					active_cnt = 0;
					
					lost_cnt = 0;					
					/*���ģʽ���л�*/
				}
			}
			/*�����ж������*/	
			 
			/*������������ʶ����л�*/ 
		}
	
	
		/*������ ģʽ�ж����� ������*/
	
		/*
			�������б�µĹ�����
			����������л������������������б�²�����һС��ʱ�䣬���������ڵ�֡�����ĳ��������ĸ���ʵ��������ã��ɲ��ã� 
		*/ 
		if(ramp_cnt > 0)
		{
			/*б�¹���*/
			now_vision_yaw = RampFloat(abs(vision_data.yawAngle - now_vision_yaw)/ramp_cnt,vision_data.yawAngle,now_vision_yaw);
			ramp_cnt--;
		}
		else
			now_vision_yaw = vision_data.yawAngle;                          vision_degree = vision_data.yawAngle;
		/* ��������ȡ��ǰ���Ӿ��Ƕȡ����� */ 
		
		update_cloud_yaw = cloud_yaw_raw+start_mech_yaw; 			/*�Ӿ����ݸ���ʱ����̨�Ƕ�*/
		/*�Ӿ����ݸ���ʱ����̨����*/ 
		
		vision_yaw_raw = now_vision_yaw * 22.253f;	/*�任Ϊ��е�Ƕȵĳ߶�*/    
		/*�Ӿ����������ݣ�ת���ɺͻ�е�Ƕȵ�ͬһ���߶�*/ 
		
		
		vision_yaw_kf = KalmanFilter(&kalman_visionYaw,vision_yaw_raw); 	/*���Ӿ��Ƕ��������������˲�*/  
		/*�Ӿ����ݵĿ������˲�*/ 
		
		
		vision_dis_raw = vision_data.distance/1000.f;	/*�������*/
		/*��ȡ��������*/  
	
	
		vision_dis_kf = KalmanFilter(&kalman_dist,vision_dis_raw);	/*�˲�*/
		/*�������ݿ������˲���*/ 
		
		
		vision_this_time = millis();		/*��ȡ��ǰʱ��*/
		/*�����Ӿ����µ�ʱ�� -- ��������õ�*/ 
		
		vision_update_flag = false;		/*�����־λ*/	
		
		/*֮���Խ����ݸ��µ����ݷ��ڸ��±�־λ���Ϊ�˱�֤Ԥ�����ݿ��Բ�����̨�����˶���Ӱ�졣
		��ϸ˵�˾�����Ϊ���ݵĸ���Ƶ��ԶС����̨�ĽǶ����ݣ����Ӿ�û���µ�ʱ������̨������һֱ�ڸ��¡�
		�⵼������̨���˶����Ԥ����ٶȴ����Ŷ����ݡ�����Ĵ������Ϊ�˾����ܵر������������ 
		*/ 
		
	}
	/*���������ݸ��µĴ��������*/
		
	target_yaw_raw = -vision_yaw_kf*k_scale_vision + cloud_yaw_kf;		/*��Ŀ��������Ƕ�*/
	/*Ŀ���λ�õ�yaw�Ƕ�Ԥ�⣬����ķ�������Ϊ������װ������̨������ͷ�İ�װ֮��Ĺ�ϵ�����忴�Լ��İ�װ��ϵ��*/ 


	target_yaw_kf = KalmanFilter(&kalman_targetYaw,target_yaw_raw);		/*��Ŀ���yaw�ǶȽ��ж�λ*/   target_degree=target_yaw_kf/22.253f;  
	/*��Ŀ��ǶȽ����˲�*/ 
	
	
	now_cal_yaw = -vision_yaw_raw*k_scale_vision + update_cloud_yaw;	 /*Ŀ��ĽǶ� -- ���ڼ���Ŀ���ٶ�*/
	/*��ʼԤ���ٶȣ������k_scale_vision������ͷ�Ƕȱ仯��ת��ϵ����
	
		�ٸ����ӣ�����ͷ���Ӿ����ݱ仯��10�㣬����ʵ������̨ת����15�㣬�����ʱ�����Ҫ���Ӿ������ݳ���һ��1.5����ʱ��k_scale_vision��Ϊ1.5 
	*/ 
	
	
	target_speed_raw = Get_Target_Speed(queue_speed_length,now_cal_yaw)*k_speed; 	/*�������Ŀ���ٶȵ�Ԥ��*/
	/*��ʼ��Ŀ����ٶȽ���Ԥ�⣬�������ֱ�Ӳο��Ҹ��Ĵ��롣k_speed������ϵ������Ϊ�õ����ٶ����ݱȽ�С����˷Ŵ��ŵ��������ȽϺ���*/ 
	
	
	if(vision_data.identifyTarget == 1)				/*�����ٶ�ֵ*/
		target_speed_kf = KalmanFilter(&kalman_speedYaw,target_speed_raw);
	else
		target_speed_kf = KalmanFilter(&kalman_speedYaw,0);			
	
	/*�ο�ȥ���������*/ 
	
	
	target_accel_raw = Get_Target_Accel(10,target_speed_kf);	 /*��ȡ���ٶ�*/
	
	target_accel_raw = myDeathZoom(0,0.1,target_accel_raw);		/*�������� - �˳�0�㸽��������*/
	
	target_accel_kf = KalmanFilter(&kalman_accel,target_accel_raw);		/*�������˲�*/

	/*���������ǽ�����ٶȵ����ݣ����ٶȵĽ���ԭ��һ��������������*/		
		
	if(predict_delay>0)
	{
		/*Ԥ�����ʱ*/
		predict_delay--;
		predict_angle_raw=0;
		
		/*����predict����ʱ��ȥ��Ĵ����е��񣬶���Ϊ������̨�Ȱڵ�Ŀ�긽���ڽ���Ԥ���õģ�
		����ᵼ����̨�ζ���ԭ�������Ϊ��̨�ζ����Ԥ����ٶȴ���Ӱ�죬�Ӷ��γ��������������𵴹��̡� */ 
	}
	else
	{
		/*��ʱ�����󣬽���Ԥ�����*/
		float tmp1;	 /*��ʱ����*/
		
		if((target_accel_kf*target_speed_kf)>=0)
		{
			/*���ٶȺ��ٶ�ͬ��ʱ*/
			tmp1 = 0.8f; 
		}
		else
		{
			tmp1 = -1.2f;
		}
		
		/*���������жϼ��ٶȺ��ٶȹ�ϵ�����������ٶȵ����÷���
			
			���� ���ٶȺ��ٶȷ���ʱ��˵�������˶�
				���ٶȺ��ٶ�ͬ��ʱ��˵�������˶��� 
		*/ 
		
		feedforward_angle = k_ff * target_accel_kf; 	/*����ǰ����*/
		
		feedforward_angle = constrain(feedforward_angle,-4.f,4.f);	 /*ǰ����*/
		
		/*ǰ���Ƕ�ʱͨ�����ٶ�����Ŀ���˶�����ǰ��Ӧ��*/ 
		
		predict_angle_raw = k_pre*target_speed_kf*vision_dis_kf + tmp1*feedforward_angle*use_ff;	/*����Ԥ��Ƕ�*/
	}
	predict_angle_raw = constrain(predict_angle_raw,-8,8);	/*�޷�*/
	
	predict_angle = RampFloat((abs(predict_angle_raw - predict_angle)/predict_ramp_cnt),predict_angle_raw,predict_angle); 	/*б�´���*/
	
	/*�����Ƕ�Ԥ����˲�*/ 
	
	attack_pitch_offset = k_pitch * vision_dis_kf; 	/*̧ͷ���벹��*/
	/*���ھ����pitch�Ƕ�̧ͷ������û�м����pitch�����Ԥ�⡣���������ϡ�*/ 
	
	
	/*�����ǿ�������Ҫ�ĽǶȼ������*/
	
	outline_cnt = millis() - vision_this_time; 	/*���߼���*/
	/*�����Ӿ����һ֡�ĸ���ʱ�䣬���ж��Ƿ��Ѿ�����*/ 
	
	
	if(outline_cnt >= 1000)
	{
		/*����*/
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
	/*�ж��Ƿ�����*/
	/*����Ҫ�ж�Ӧ�Ĵ���������̨���ߡ�*/ 
	
	
	if(vision_this_time != vision_last_time)
	{
		/*����֡��*/
		vision_update_fps = vision_this_time - vision_last_time;
		
		vision_last_time = vision_this_time;
		
		/*����֡��*/ 
	}
	
	last_identify_flag = vision_data.identifyTarget;
	//��¼��һ�α�־λ
}
