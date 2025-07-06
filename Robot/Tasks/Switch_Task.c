#include "Switch_Task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "Shoot_Task.h"
#include "remote_control.h"
#include "referee.h"
#include "Cboard_To_Nuc_usbd_communication.h"

#define DIAL_SPEED_LOW 4500 // 3000
#define DIAL_SPEED_HIGH 5000

void Switch_Task(void const *argument)
{
	while (1)
	{
		vTaskDelay(2);
	}
}