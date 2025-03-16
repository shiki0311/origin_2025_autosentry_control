#include "bsp_tim.h"

float fric_speed1=0,fric_speed2=0;
uint8_t if_encoder_first_run=1;

uint32_t tim14_cnt=0;
void encoder_solve(void)
{
	if(if_encoder_first_run)
	{
		TIM1->CNT=0;
		TIM8->CNT=0;
		fric_speed1=0;
		fric_speed2=0;
		if_encoder_first_run=0;
	}
	else
	{
		fric_speed1=(float)TIM1->CNT*500.0f/1000.0f;
		fric_speed2=(float)TIM8->CNT*500.0f/1000.0f;
		TIM1->CNT=0;
		TIM8->CNT=0;
		tim14_cnt++;
	}
}
