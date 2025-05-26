#ifndef BSP_BUZZER_H
#define BSP_BUZZER_H
#include "struct_typedef.h"
extern void buzzer_on(uint16_t psc, uint16_t pwm);
extern void buzzer_off(void);

typedef struct {
    uint16_t freq;  
    uint16_t duration; // 持续时间（ms）
} Melody_TypeDef;

extern void buzzer_play_maomao(void);

#endif
