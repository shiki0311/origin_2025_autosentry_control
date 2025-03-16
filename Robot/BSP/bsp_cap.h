#ifndef BSP_CAP_H
#define BSP_CAP_H

#include "main.h"

typedef struct
{
	float cap_per;
	float chassis_power;
	
	float max_power;
	float actual_power;
	float buffer_power;
}cap_measure_t;


extern cap_measure_t cap_data;

void update_cap(uint8_t * data);

#endif
