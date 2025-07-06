#ifndef BSP_BUZZER_H
#define BSP_BUZZER_H
#include "struct_typedef.h"

typedef struct {
    uint16_t freq;  
    uint16_t duration; // 持续时间（ms）
		uint16_t pwm;
	
} Melody_TypeDef;

void buzzer_play_mao();
void buzzer_play_eva();


#endif
