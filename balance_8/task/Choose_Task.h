#ifndef __Choose_task_H
#define __Choose_task_H
#include "system.h"

typedef enum{
	//�Ƿ�ɼ������
	shift_gravity_open = 0,
	shift_gravity_close = 2,
	//�����Ƿ�ƫ��
	shift_gravity_Yes = 1,
	shift_gravity_No = 3,
	
}task__Choose;

typedef struct{
	task__Choose shift_grav_test;//�Ƿ�ɼ������
	task__Choose shift_grav_analysis;//�����Ƿ�ƫ��
	
}choose_Task;
void choose_task(void);
void choose_task_1(void);
extern choose_Task Task_Choose;
extern  float gravity;
#endif


