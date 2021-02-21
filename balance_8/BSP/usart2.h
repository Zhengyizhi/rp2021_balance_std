#ifndef __USART2_H
#define __USART2_H

#include "system.h"

/* ----------------------- RC Channel Definition---------------------------- */
#define RC_CH_VALUE_MIN ((uint16_t)364 )
#define RC_CH_VALUE_OFFSET ((uint16_t)1024)
#define RC_CH_VALUE_MAX ((uint16_t)1684)
/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP ((uint16_t)1)
#define RC_SW_MID ((uint16_t)3)
#define RC_SW_DOWN ((uint16_t)2)
/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W ((uint16_t)0x01<<0)
#define KEY_PRESSED_OFFSET_S ((uint16_t)0x01<<1)
#define KEY_PRESSED_OFFSET_A ((uint16_t)0x01<<2)
#define KEY_PRESSED_OFFSET_D ((uint16_t)0x01<<3)
#define KEY_PRESSED_OFFSET_Q ((uint16_t)0x01<<4)
#define KEY_PRESSED_OFFSET_E ((uint16_t)0x01<<5)
#define KEY_PRESSED_OFFSET_SHIFT ((uint16_t)0x01<<6)
#define KEY_PRESSED_OFFSET_CTRL ((uint16_t)0x01<<7)


typedef union{
	struct 
	{
		struct
		{
			uint8_t ch0_h:8;
			
			uint8_t ch0_1:3;
			uint8_t ch1_h:5;
			
			uint8_t ch1_1:6;
			uint8_t ch2_h:2;
			
			uint8_t ch2_m:8;
			
			uint8_t ch2_1:1;
			uint8_t ch3_h:7;
			
			uint8_t ch3_1:4;
			uint8_t s1:2;
			uint8_t s2:2;	
			
//			uint8_t sw:8;
//			uint8_t sw:3;
		}rc;
		struct 
		{
			int16_t x; //!< Byte 6-7
			int16_t y; //!< Byte 8-9
			int16_t z; //!< Byte 10-11
			uint8_t press_l; //!< Byte 12
			uint8_t press_r; 
			
		}mouse;
		struct 
		{
			uint16_t v; //!< Byte 14-15
		}key;
		
		uint16_t resv; //!< Byte 16-17
	}data;
	uint8_t buf[18]; //!< Union --> Byte<0-17>
}RC_Ctl_Define_t;

typedef struct{
	struct{
		int16_t ch0;
		int16_t ch1;
		int16_t ch2;
		int16_t ch3;
		int8_t s1;
		int8_t s2;
		int16_t sw;
		
	}rc;
	struct{
		int16_t x;
		int16_t y;
		int16_t z;
		uint8_t press_l;
		uint8_t press_r;
	}mouse;
	struct{
		uint16_t v;
	}key;
}RC_Ctl_t;


extern RC_Ctl_t RC_Ctl;//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
extern int16_t System_Num;

#define RC_CH_VALUE_MIN ((uint16_t)364 )
#define RC_CH_VALUE_OFFSET ((uint16_t)1024)
#define RC_CH_VALUE_MAX ((uint16_t)1684)

#define KEY_PRESSED_OFFSET_W ((uint16_t)0x01<<0)
#define KEY_PRESSED_OFFSET_S ((uint16_t)0x01<<1)	

void RC_Init(void);
void Remote_ModeJudge(void);
//void Keyanaly(void);

#endif


