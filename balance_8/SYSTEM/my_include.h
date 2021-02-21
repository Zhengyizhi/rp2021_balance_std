#ifndef __MY_include_H
#define __MY_include_H

#define myconstrain(x, min, max)	((x>max)?max:(x<min?min:x))
#define myabs(x) 					((x)>0? (x):(-(x)))


typedef enum{
	None = 0,
	Set = 1,
	ReSet = 2,
	
	SetTurn_Pencoder_Dz = 6,//积分影响用户体验，编码器对车轮滑动无法检测，陀螺仪的漂移
	SetTurn_Pz_Dz = 7,//陀螺仪漂移
	SetTurn_Pz = 8,//陀螺仪对高频信号采样失真
	
	speedset = 3,//云台专用
	angleset =4,
	
	turn_center = 5,
	singlespeed = 6,
	
	momentum = 15,//动量轮
	//直立专用
	_OK = 11,
	_not =12,
	inside = 13,
	outside =14
}Circle_control;


typedef struct{
	Circle_control Stand_circle;
	Circle_control Speedrun_circle;
	Circle_control Turnaround_circle;//底盘大环
	
	Circle_control pitchgimble_circle;
	Circle_control yawgimble_circle;//云台环
	
	Circle_control singlespeed;
	Circle_control momentum_wheel;//动量轮
//	Circle_control turncentre;//调重心
	
	//站起来
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
