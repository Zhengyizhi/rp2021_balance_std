#ifndef __SYSTEM_H
#define __SYSTEM_H

#include <stdio.h>
#include "stdbool.h"
#include "stm32f4xx_can.h"
#include "stm32f4xx.h"
#include "stdint.h"
#include "sys.h" 
#include "usart.h"
#include "string.h"

//#include "inv_mpu.h"
//#include "inv_mpu_dmp_motion_driver.h"
#include "myiic.h" 
//#include "mpu6050.h"
#include "bmi.h"
#include "bmi2.h"
#include "bmi2_common.h"
#include "bmi270.h"



#include "can1.h"
#include "can2.h"
#include "led.h"
#include "usart2.h"
#include "control.h"
#include "crc.h"
#include "anoc.h"

/*以下自己写的*/
#include "my_include.h"
#include "remote.h"
#include "chassis.h"
#include "gimble.h"
#include "pid.h"
#include "2ms_analy.h"
#include "kalman.h"
#include "stand.h"
#include "momentum_wheel.h"
#include "choose_Task.h"


#define abs(x) ((x)>0? (x):(-(x)))

extern volatile uint32_t sysTickUptime;



float constrain(float amt, float low, float high);
int32_t constrain_int32(int32_t amt, int32_t low, int32_t high);
int16_t constrain_int16(int16_t amt, int16_t low, int16_t high);

uint32_t micros(void);
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);

uint32_t millis(void);
void Parameter_Init(void);
void System_Init(void);
void Loop(void);
void Stop(void);
extern float roll,pitch,yaw;
extern short gyrox,gyroy,gyroz;	//陀螺仪原始数据

void see11(void);
void see22(void);
void see33(void);


#endif


