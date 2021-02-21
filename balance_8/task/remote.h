#ifndef __remote_H
#define __remote_H
#include "system.h"
#define Right_remote_cross 0
#define Right_remote_vertical 1
#define Left_remote_cross 2
#define Left_remote_vertical 3






extern float Remote_Target_Array[5];
void control_circleinit(void);
void remote_calculate(void);
void control_circle_init(void);

#endif
