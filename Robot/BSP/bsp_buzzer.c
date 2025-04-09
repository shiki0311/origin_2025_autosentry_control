#include "bsp_buzzer.h"
#include "main.h"

#define TIM_CLK 84000000
#define TIM4_ARR 21000

extern TIM_HandleTypeDef htim4;
void buzzer_on(uint16_t freq, uint16_t pwm)
{
    uint16_t psc = (TIM_CLK/(freq*TIM4_ARR))-1;
    __HAL_TIM_PRESCALER(&htim4, psc);
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, pwm);

}
void buzzer_off(void)
{
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
}

static Melody_TypeDef melody[] = {
    {215, 35}, {959, 150}, {1085, 162}, {1186, 441},   
    {179, 46}, {1336, 23}, {1234, 58}, {1343, 208},  
    {1051, 208}, {145, 34}, {1062, 116},           
    {842, 545}, {152, 69},{0,0}
};

void buzzer_play_maomao(void)
{
    uint8_t i = 0;
    while(melody[i].duration != 0)
    {
        buzzer_on(melody[i].freq, 15000);
        HAL_Delay(melody[i].duration);
        i++;
    }
    buzzer_off();
}