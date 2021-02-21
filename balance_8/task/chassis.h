#ifndef __Chassis_H
#define __Chassis_H
#include "system.h"
#include "my_include.h"
//#define IS_Circle_GET_FLAG(Circle_FLAG) (((Circle_FLAG) == Standcircle) || \
//                                   ((Circle_FLAG) == Speedcircle) || \
//                                   ((Circle_FLAG) == Turnaroundcircle))
////·ûºÏ¾Í·µ»Ø1

void send_chassis_moto(void);
void chassis_control(void);
void mpu_AngleAnaly(char num,float jump,float center,float degree,float Nowangle);
void mpu_AngleAnaly2(char num,float jump,float center,float degree,float Nowangle);
extern float stand_angle,stand_Speed1,stand_Speed2,singlespeed1,singlespeed2;//
extern float speedcircle_speed1,speedcircle_speed2;
void wheel_Initanaly(void);
#endif
