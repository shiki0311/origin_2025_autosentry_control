#include "kalman_tracking_Task.h"
#include "main.h"
#include "Vofa_send.h"
#include "INS_Task.h"
#include "referee.h"

char data[512];

void kalman_tracking_task(void const * argument)
{
  /* USER CODE BEGIN kalman_tracking_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
	}
  /* USER CODE END kalman_tracking_task */
}