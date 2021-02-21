#ifndef __2ms_analy_H
#define __2ms_analy_H
#include "system.h"


typedef enum{
	_none = 1,
	_normal =2,
	_disconnected = 3,
	_error = 4,
	_normal2 = 5,
	_normal3 = 6
}interrupt_Flag;//interrupt


typedef enum{

	remote_control =1,
	un_control =2,
	
	falling =3,
	standing =4,
	mpu_OK =5,
	mpu_un = 6,
	one = 7,
	two = 8,
	three =9,
	
	
	
	go_forward = 10,
	go_back = 11,//Ô¤¾¯
	stop_forward = 12,
	stop_back = 13,//´íÎóÉ²³µ
	_clear_warning = 14
	
}car_Flag;

typedef struct{
	interrupt_Flag DMAinterrupt_Flag;
	interrupt_Flag Can1interrupt_Flag;
	interrupt_Flag RemoteData_Flag;
	interrupt_Flag Stand_Flag;
	
	car_Flag mpu_Flag;
	
	car_Flag Car_Flag;
	car_Flag Car_transit_Flag;
	car_Flag Car_level;
	car_Flag Car_remote_warning;
	car_Flag hand_remote_warning;
}conditionFlag;//

extern conditionFlag AllFlag;
void analy_Flag(void);


#endif

