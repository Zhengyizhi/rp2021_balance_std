#ifndef __momentum_wheel_H
#define __momentum_wheel_H

#include "system.h"
#define zhuanzhi 2//0∑¥÷√




typedef enum
{
	level_0=0,
	level_1=1,
	level_2=2,
	level_3=3,
	level_4=4,
	level_5=5,
	level_6=6,
	level_7=7
}Con_level;

typedef struct Condition_order
{
	Con_level momentum_order;
}Condition_order;

typedef enum
{
	_blank_ = 0,
	_negative = 1,
	_positive_1 = 2,
	_positive_2 = 3,
}_remote;

typedef struct choose_Fed
{
//	_remote remote_;
	float goal_angle;
	float now_angle;
	_remote CHOOSE;

}choose_Fed;

extern choose_Fed Choose_Fed;
extern Condition_order Ordor_Condition;
extern float chassis_remoteforward;
void Decide_Condition (void);
void forward_Stand(void);
void backward_Stand(void);
void normal(void);


void  none_feedback(void);
void negative_feedback(void);
void positive_feedback(void);
void choose_feedback(void);

#endif
