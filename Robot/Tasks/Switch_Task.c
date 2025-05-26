#include "Switch_Task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "Shoot_Task.h"
#include "remote_control.h"
#include "referee.h"
#include "Nmanifold_usbd_task.h"
#include "SolveTrajectory.h"

#define DIAL_SPEED_LOW 4500 // 3000
#define DIAL_SPEED_HIGH 5000

RC_ctrl_t rc_ctrl_last;
uint8_t dial_mode_last = 0;


void Switch_Task(void const *argument)
{
	while (1)
	{
		static uint32_t shoot_power_good_cnt = 0;

		if (((rc_ctrl.rc.s[0] == RC_SW_DOWN) && !(rc_ctrl.rc.s[1] == RC_SW_UP)) || rc_ctrl.rc.s[1] == RC_SW_DOWN || Game_Robot_State.power_management_shooter_output == 0x00)
		{
			shoot_power_good_cnt = 0;
			shoot_flag = 0;
		}

		if ((Game_Robot_State.power_management_shooter_output == 0x01) && shoot_power_good_cnt <= 20)
			shoot_power_good_cnt++; // delay

		if (((rc_ctrl.rc.s[1] == RC_SW_UP) || (rc_ctrl.rc.s[1] == RC_SW_MID)) && (shoot_power_good_cnt >= 20))
		{
			fric_state = 1;
		}
		else
		{
			fric_state = 0;
		}

		if (shoot_flag == 1)
		{ // dial
			if (dial_mode == 0)
			{
				if ((rc_ctrl.rc.s[0] == RC_SW_UP && rc_ctrl.rc.s[1] == RC_SW_MID) || (rc_ctrl.rc.s[1] == RC_SW_UP && AutoAim_Data_Receive.fire_or_not == 1 && Game_Status.game_progress == 4))
					dial_speed = DIAL_SPEED_HIGH;
				else
					dial_speed = 0;
			}
			else // 单发模式
			{
				static uint8_t autoaim_shoot_freq = 0;
				if ((!(rc_ctrl_last.rc.s[0] == RC_SW_MID) && (rc_ctrl.rc.s[0] == RC_SW_MID)) || (!(rc_ctrl_last.rc.s[0] == RC_SW_UP) && (rc_ctrl.rc.s[0] == RC_SW_UP)))
				{
					if (if_single_hit == 0) //
					{
						if_single_hit = 1;
						shoot_m2006[0].angle_set = (shoot_m2006[0].angle + 8192 * 45 / 10);
						shoot_m2006[1].angle_set = (shoot_m2006[1].angle + 8192 * 45 / 10);
					}
				}

				if (if_single_hit == 0)
				{
					if_single_hit = 1;
					shoot_m2006[0].angle_set = (shoot_m2006[0].angle + 8192 * 45 / 10 * (float)(autoaim_shoot_freq / 10));
					shoot_m2006[1].angle_set = (shoot_m2006[1].angle + 8192 * 45 / 10 * (float)(autoaim_shoot_freq / 10));
				}
			}
		}
		if (Autoaim_Mode == 0 || (Autoaim_Mode == 3))
			dial_mode = 0;
		else if (Autoaim_Mode == 1 || Autoaim_Mode == 2 || Autoaim_Mode == 3)
			dial_mode = 1;

		if (dial_mode == 1 && dial_mode_last == 0)
		{
			shoot_m2006[0].angle_set = shoot_m2006[0].angle;
			shoot_m2006[1].angle_set = shoot_m2006[1].angle;
		}

		rc_ctrl_last = rc_ctrl;
		dial_mode_last = dial_mode;
		vTaskDelay(2);
	}
}
