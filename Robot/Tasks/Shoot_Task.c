#include "Shoot_Task.h"
#include "Gimbal_Task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "remote_control.h"
#include "bsp_can.h"
#include "bsp_tim.h"
#include "usart.h"
#include <stdio.h>
#include <stdlib.h>
#include "referee.h"
#include "math.h"
#include "Cboard_To_Nuc_usbd_communication.h"
#include "Vofa_send.h"
#include "user_common_lib.h"

#define DIAL_MOTOR_SPEED_PID_KP 5.0f
#define DIAL_MOTOR_SPEED_PID_KI 0.01f
#define DIAL_MOTOR_SPEED_PID_KD 0.0f
#define DIAL_MOTOR_SPEED_PID_MAX_OUT 28000.0f
#define DIAL_MOTOR_SPEED_PID_MAX_IOUT 10000.0f

#define DIAL_MOTOR_ANGLE_PID_KP 0.3f
#define DIAL_MOTOR_ANGLE_PID_KI 0.0f
#define DIAL_MOTOR_ANGLE_PID_KD 0.2f
#define DIAL_MOTOR_ANGLE_PID_MAX_OUT 10000.0f
#define DIAL_MOTOR_ANGLE_PID_MAX_IOUT 10000.0f

#define SHOOT_MOTOR_3508_SPEED_PID_KP1 4.0f
#define SHOOT_MOTOR_3508_SPEED_PID_KI1 0.00007f
#define SHOOT_MOTOR_3508_SPEED_PID_KD1 0.0f

#define SHOOT_MOTOR_3508_SPEED_PID_KP2 4.0f
#define SHOOT_MOTOR_3508_SPEED_PID_KI2 0.00007f
#define SHOOT_MOTOR_3508_SPEED_PID_KD2 0.0f
#define SHOOT_MOTOR_3508_SPEED_PID_MAX_OUT 16000.0f
#define SHOOT_MOTOR_3508_SPEED_PID_MAX_IOUT 4000.0f

#define DIAL_SPEED_LOW 4500 // 3000
#define DIAL_SPEED_HIGH 5000

shoot_motor_t shoot_m2006[2] = {0};
shoot_motor_t shoot_motor_3508[2] = {0};
bool_t flag_cooling_limit[2] = {0};
uint16_t cooling_limit_cnt[2] = {0};
uint8_t fric_state = 0; // 0:off,1:on
motor_measure_t shoot_motor_measure[2];
// uint16_t fric_speed = 1005;  #pwm控制摩擦轮的寄存器目标值
uint16_t shoot_flag = 0;
int target_rpm_define = 6100;

/*****************************************************************根据裁判系统发射数据进行弹速闭环********************************************************************************/

#define USE_REFEREE_BULLET_SPEED_LOOP 1 // 1:对裁判系统传回的弹速闭环，外环控弹速（因为裁判系统传回的弹速数据是发射一发子弹传一次，频率不固定，所以只能用状态机控制，不用pid)，内环控摩擦轮3508转速
										// 0:仅对3508摩擦轮速度闭环，适用没有裁判系统的情况
#if USE_REFEREE_BULLET_SPEED_LOOP
// 弹速控制参数
#define TARGET_BULLET_SPEED 23.0f // 目标弹速
#define MIN_ADJUST_STEP 10		  // 弹速很接近目标弹速时每次调整的转速步长
#define MIDDLE_ADJUST_STEP 15	  // 弹速较为接近目标弹速时每次调整的转速步长
#define MAX_ADJUST_STEP 30		  // 弹速距离目标弹速偏差较大时每次调整的转速步长
#define MIN_RPM 5000			  // 摩擦轮最小转速
#define MAX_RPM 7000			  // 摩擦轮最大转速

uint8_t shoot_new_bullet = 0; // 检测发射机构是否发射新子弹

/**
 * @description:根据裁判系统弹速数据判断是否有子弹发射，发射则shoot_new_bullet = 1,只有在USE_REFEREE_BULLET_SPEED_LOOP为1时会被编译
 * @return 无
 */
void Shoot_Bullet_Update()
{
	static fp32 last_bullet_speed;
	if (Shoot_Data.bullet_speed != last_bullet_speed)
	{
		shoot_new_bullet = 1;
	}
	else
	{
		shoot_new_bullet = 0;
	}
	last_bullet_speed = Shoot_Data.bullet_speed;
}
/**
 * @description:选择弹速闭环时的改变摩擦轮转速的步长
 * @return 改变摩擦轮转速的步长
 */
uint8_t Choose_Bullet_Speed_Adjust_Step(float diff)
{
	if (diff > 0.3f)
	{
		if (diff <= 1.0f)
		{
			return MIN_ADJUST_STEP;
		}
		else if (diff <= 2.0f)
		{
			return MIDDLE_ADJUST_STEP;
		}
		else
		{
			return MAX_ADJUST_STEP;
		}
	}
	return 0;
}
/**
 * @description:根据裁判系统反馈弹速调整摩擦轮目标转速
 * @return 无
 */
void Fric_Motor_Speed_Control(void)
{
	static uint8_t bullet_count = 0;	  // 连续弹速计数，发射三发视为一轮
	static float bullet_speed_sum = 0.0f; // 弹速总和（用于计算平均值）
	if (shoot_new_bullet && Game_Robot_State.power_management_shooter_output == 0x01)
	{
		bullet_speed_sum += Shoot_Data.bullet_speed;
		bullet_count++;

		// 每3发子弹进行一次调速
		if (bullet_count >= 3)
		{
			float avg_speed = bullet_speed_sum / bullet_count;
			// 调速逻辑
			float diff = TARGET_BULLET_SPEED - avg_speed;
			uint8_t step = Choose_Bullet_Speed_Adjust_Step(my_fabsf(diff));
			target_rpm_define += my_sign(diff) * step;
			// 对目标转速进行约束
			target_rpm_define = limit(target_rpm_define, (float)MIN_RPM, (float)MAX_RPM);
			// 重置计数
			bullet_speed_sum = 0.0f;
			bullet_count = 0;
		}
	}
}
#endif
/********************************************************************************************************************************************************************************/
void Shoot_Motor_Init(void)
{
	const static fp32 shoot_motor_speed_pid[3] = {DIAL_MOTOR_SPEED_PID_KP, DIAL_MOTOR_SPEED_PID_KI, DIAL_MOTOR_SPEED_PID_KD};
	const static fp32 shoot_motor_angle_pid[3] = {DIAL_MOTOR_ANGLE_PID_KP, DIAL_MOTOR_ANGLE_PID_KI, DIAL_MOTOR_ANGLE_PID_KD};

	PID_init(&shoot_m2006[0].speed_pid, PID_POSITION, shoot_motor_speed_pid, DIAL_MOTOR_SPEED_PID_MAX_OUT, DIAL_MOTOR_SPEED_PID_MAX_IOUT);
	PID_init(&shoot_m2006[0].angle_pid, PID_POSITION, shoot_motor_angle_pid, DIAL_MOTOR_ANGLE_PID_MAX_OUT, DIAL_MOTOR_ANGLE_PID_MAX_IOUT);

	const static fp32 shoot_motor_3508_speed_pid1[3] = {SHOOT_MOTOR_3508_SPEED_PID_KP1, SHOOT_MOTOR_3508_SPEED_PID_KI1, SHOOT_MOTOR_3508_SPEED_PID_KD1};
	const static fp32 shoot_motor_3508_speed_pid2[3] = {SHOOT_MOTOR_3508_SPEED_PID_KP2, SHOOT_MOTOR_3508_SPEED_PID_KI2, SHOOT_MOTOR_3508_SPEED_PID_KD2};

	PID_init(&shoot_motor_3508[0].speed_pid, PID_POSITION, shoot_motor_3508_speed_pid1, SHOOT_MOTOR_3508_SPEED_PID_MAX_OUT, SHOOT_MOTOR_3508_SPEED_PID_MAX_IOUT);
	PID_init(&shoot_motor_3508[1].speed_pid, PID_POSITION, shoot_motor_3508_speed_pid2, SHOOT_MOTOR_3508_SPEED_PID_MAX_OUT, SHOOT_MOTOR_3508_SPEED_PID_MAX_IOUT);
}

/**
 * @description:更新波蛋盘2006电机和两个摩擦轮3508电机的数据
 * @return 无
 */
void Shoot_Motor_Data_Update(void)
{
	shoot_m2006[0].speed = motor_measure_shoot[2].speed_rpm;
	shoot_m2006[0].current = motor_measure_shoot[2].given_current;
	shoot_m2006[0].angle = dial_angle;
	for (int i = 0; i < 2; i++)
	{
		shoot_motor_3508[i].speed = motor_measure_shoot[i].speed_rpm;
		shoot_motor_3508[i].current = motor_measure_shoot[i].given_current;
	}
}

/**
 * @description:更新fric_state,fric_state为1时摩擦轮转动
 * @return 无
 */
void Fric_State_Update()
{
	if (((rc_ctrl.rc.s[1] == RC_SW_UP) || ((rc_ctrl.rc.s[1] == RC_SW_MID) && (rc_ctrl.rc.s[0] != RC_SW_DOWN))) && Game_Robot_State.power_management_shooter_output == 0x01)
	{
		fric_state = 1;
	}
	else
	{
		fric_state = 0;
	}
}
/**
 * @description:更新shoot_flag,shoot_flag为1时才能转波蛋盘，目的是确保在摩擦轮转速达到后才发弹
 * @return 无
 */
void Shoot_Flag_Update(void)
{
	if (((rc_ctrl.rc.s[0] == RC_SW_DOWN) && !(rc_ctrl.rc.s[1] == RC_SW_UP)) || rc_ctrl.rc.s[1] == RC_SW_DOWN || Game_Robot_State.power_management_shooter_output == 0x00)
	{
		shoot_flag = 0;
	}
	else if ((shoot_motor_3508[0].speed > target_rpm_define - 300) && (shoot_motor_3508[1].speed < -target_rpm_define + 300))
	{
		shoot_flag = 1;
	}
}

/**
 * @description:设置波蛋盘目标转速，在Dial_Motor_Control()中调用
 * @return 波蛋盘目标转速
 */
fp32 Dial_Speed_Set(void)
{
	bool_t remote_control_shoot = (rc_ctrl.rc.s[0] == RC_SW_UP && rc_ctrl.rc.s[1] == RC_SW_MID);
	bool_t autoaim_shoot = (rc_ctrl.rc.s[1] == RC_SW_UP && AutoAim_Data_Receive.fire_or_not == 1 && Game_Status.game_progress == 4);
	if (shoot_flag && (remote_control_shoot || autoaim_shoot))
	{
		return (float)DIAL_SPEED_HIGH;
	}
	else
		return 0.0f;
}

void Fric_Motor_Current_Control(void)
{
	if (fric_state) // fric_state
	{
		shoot_motor_3508[0].speed_set = target_rpm_define;
		shoot_motor_3508[1].speed_set = -target_rpm_define;
	}
	else
	{
		shoot_motor_3508[0].speed_set = 0;
		shoot_motor_3508[1].speed_set = 0;
	}

	PID_calc(&shoot_motor_3508[0].speed_pid, shoot_motor_3508[0].speed, shoot_motor_3508[0].speed_set);
	PID_calc(&shoot_motor_3508[1].speed_pid, shoot_motor_3508[1].speed, shoot_motor_3508[1].speed_set);

	shoot_motor_3508[0].target_current = shoot_motor_3508[0].speed_pid.out; // - dif_pid.out;
	shoot_motor_3508[1].target_current = shoot_motor_3508[1].speed_pid.out; // + dif_pid.out;
}

void Dial_Motor_Control(void)
{
	static uint32_t dial_stop_cnt = 0;
	if (Game_Robot_State.power_management_shooter_output == 0x01) // 卡弹保护，卡弹时波蛋盘反转
	{
		if (abs(shoot_m2006[0].target_current) > 18000 && fabs(shoot_m2006[0].speed) < 100)
		{
			dial_stop_cnt++;
			if (dial_stop_cnt > 200)
			{
				if (shoot_m2006[0].target_current < 0)
					shoot_m2006[0].target_current = 5000;
				else
					shoot_m2006[0].target_current = -5000;
				vTaskDelay(100);
				Shoot_Motor_Init();
				PID_clear(&shoot_m2006[0].speed_pid);
				PID_clear(&shoot_m2006[0].angle_pid);
				dial_stop_cnt = 0;
				shoot_m2006[0].angle_set = dial_angle;
			}
		}
	}
	shoot_m2006[0].speed_set = Dial_Speed_Set();
	PID_calc(&shoot_m2006[0].speed_pid, shoot_m2006[0].speed, shoot_m2006[0].speed_set);
	//	PID_calc(&shoot_m2006[1].speed_pid,shoot_m2006[1].speed,shoot_m2006[1].speed_set);

	shoot_m2006[0].target_current = shoot_m2006[0].speed_pid.out;
	//	shoot_m2006[1].give_current=shoot_m2006[1].speed_pid.out;
}

void Shoot_Power_Control()
{
	if (Power_Heat_Data.shooter_17mm_1_barrel_heat >= (Game_Robot_State.shooter_barrel_heat_limit - 60))
	{
		flag_cooling_limit[0] = 1;
	}
	if (Power_Heat_Data.shooter_17mm_2_barrel_heat >= (Game_Robot_State.shooter_barrel_heat_limit - 60))
	{
		flag_cooling_limit[1] = 1;
	}
	if (flag_cooling_limit[0] != 0 || flag_cooling_limit[1] != 0)
	{

		shoot_m2006[0].angle_set = shoot_m2006[0].angle;
		shoot_m2006[0].speed_set = 0;
		shoot_m2006[0].target_current = 0;

		PID_clear(&shoot_m2006[0].speed_pid);
		PID_clear(&shoot_m2006[0].angle_pid);

		cooling_limit_cnt[0]++;

		if (cooling_limit_cnt[0] >= 250)
		{
			cooling_limit_cnt[0] = 0;
			flag_cooling_limit[0] = 0;
			flag_cooling_limit[1] = 0;
		}
	}
}

void Shoot_Task(void const *argument)
{
	Shoot_Motor_Init();
	vTaskDelay(200);

	while (1)
	{
		Shoot_Motor_Data_Update();
		Fric_State_Update();
		Shoot_Flag_Update();

#if USE_REFEREE_BULLET_SPEED_LOOP
		Shoot_Bullet_Update();
		Fric_Motor_Speed_Control();
#endif

		Fric_Motor_Current_Control();
		Dial_Motor_Control();

		if (rc_ctrl.rc.s[1] == RC_SW_UP || AutoAim_Data_Receive.track || Game_Status.game_progress == 4)
		{
			Shoot_Power_Control();
		}

		vTaskDelay(2);
	}
}

// pwm控制摩擦轮
// void Fric_PWR(uint8_t power){
//	if(power){
//		if(TIM1->CCR1 < fric_speed){       //摩擦轮PWM值未到
//			TIM1->CCR1 += 3, TIM1->CCR2 += 3;
//       TIM1->CCR3 += 3, TIM1->CCR4 += 3;
//		}
//			shoot_flag=1;
//	}
//	else{
//		if(TIM1->CCR1 >= 1000){
//			TIM1->CCR1 -= 4,TIM1->CCR2 -= 4;
//       TIM1->CCR3 -= 4,TIM1->CCR4 -= 4;
//		}
//	}
// }