#ifndef __MY_include_H
#define __MY_include_H

#define myconstrain(x, min, max)	((x>max)?max:(x<min?min:x))
#define myabs(x) 					((x)>0? (x):(-(x)))


typedef enum{
	None = 0,
	Set = 1,
	ReSet = 2,
	
	SetTurn_Pencoder_Dz = 6,//����Ӱ���û����飬�������Գ��ֻ����޷���⣬�����ǵ�Ư��
	SetTurn_Pz_Dz = 7,//������Ư��
	SetTurn_Pz = 8,//�����ǶԸ�Ƶ�źŲ���ʧ��
	
	speedset = 3,//��̨ר��
	angleset =4,
	
	turn_center = 5,
	singlespeed = 6,
	
	momentum = 15,//������
	//ֱ��ר��
	_OK = 11,
	_not =12,
	inside = 13,
	outside =14
}Circle_control;


typedef struct{
	Circle_control Stand_circle;
	Circle_control Speedrun_circle;
	Circle_control Turnaround_circle;//���̴�
	
	Circle_control pitchgimble_circle;
	Circle_control yawgimble_circle;//��̨��
	
	Circle_control singlespeed;
	Circle_control momentum_wheel;//������
//	Circle_control turncentre;//������
	
	//վ����
	Circle_control front_wheel;
	Circle_control back_wheel;
	Circle_control control_FBwheel;
	
	Circle_control front_wheel_initial;
	Circle_control back_wheel_initial;
	
}Circle_condition;

extern  Circle_condition Circle_analy;


typedef enum{
	open =1,
	close =2
}_doone;

extern _doone doone;

#define 	nonecircle  ((uint8_t)0x0000)       
#define 	Standcircle  ((uint8_t)0x0001)               
#define 	Speedruncircle ((uint8_t)0x0002)            
#define 	Turnaroundcircle ((uint8_t)0x0003)
//Circle_name;




#endif
