#ifndef _LED_TASK
#define _LED_TASK

#include "main.h"
#include "referee_usart_task.h"
void LED_Task(void const * argument);
void VirCom_send(uint8_t data[], uint16_t len);
#endif
