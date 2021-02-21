#include "2ms_analy.h"

conditionFlag AllFlag;


 static  float i=0,j=0;
 static  float record_CAN1pre=0;
 static float record_CAN1now=0;
static float record_DMApre=0;
static float record_DMAnow=0;

extern float DMAinterruptnumber;
extern float CAN1interruptnumber;
void analy_Flag(void)
{
	record_DMAnow=DMAinterruptnumber;
	record_CAN1now=CAN1interruptnumber;
	if(record_DMApre!=record_DMAnow)
	{
		i=0;
		AllFlag.DMAinterrupt_Flag = _normal;
	}
	if(record_DMApre==record_DMAnow)
	{
		i++;
		if(i>24)//Ò£¿ØÆ÷Ê§Áª
		{
			AllFlag.DMAinterrupt_Flag = _disconnected;
		}
	}
	record_DMApre=record_DMAnow;
	
	if(record_CAN1pre!=record_CAN1now)
	{
		j=0;
		AllFlag.Can1interrupt_Flag = _normal;
	}
	if(record_CAN1pre==record_CAN1now)
	{
		j++;
		if(j>10)
		{
			AllFlag.Can1interrupt_Flag = _disconnected;
		}
	}
	record_CAN1pre=record_CAN1now;
	
/*Ð¡³µ×´Ì¬¼ì²â*/	
//	if(abs(mpu_data_Array[4].longtimeangle)<=5 && )
	
	
}
	






