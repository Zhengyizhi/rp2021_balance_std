#ifndef __Choose_task_H
#define __Choose_task_H
#include "system.h"

typedef enum{
	//是否可检测重心
	shift_gravity_open = 0,
	shift_gravity_close = 2,
	//重心是否偏移
	shift_gravity_Yes = 1,
	shift_gravity_No = 3,
	
}task__Choose;

typedef struct{
	task__Choose shift_grav_test;//是否可检测重心
	task__Choose shift_grav_analysis;//重心是否偏移
	
}choose_Task;
void choose_task(void);
void choose_task_1(void);
extern choose_Task Task_Choose;
extern  float gravity;
#endif


