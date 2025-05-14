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
#include "Nmanifold_usbd_task.h"
#include "Vofa_send.h"

#define SHOOT_MOTOR_SPEED_PID_KP 5.0f
#define SHOOT_MOTOR_SPEED_PID_KI 0.01f
#define SHOOT_MOTOR_SPEED_PID_KD 0.0f
#define SHOOT_MOTOR_SPEED_PID_MAX_OUT 28000.0f
#define SHOOT_MOTOR_SPEED_PID_MAX_IOUT 10000.0f

#define SHOOT_MOTOR_ANGLE_PID_KP 0.3f
#define SHOOT_MOTOR_ANGLE_PID_KI 0.0f
#define SHOOT_MOTOR_ANGLE_PID_KD 0.2f
#define SHOOT_MOTOR_ANGLE_PID_MAX_OUT 10000.0f
#define SHOOT_MOTOR_ANGLE_PID_MAX_IOUT 10000.0f

#define SHOOT_MOTOR_3508_SPEED_PID_KP1 4.0f
#define SHOOT_MOTOR_3508_SPEED_PID_KI1 0.00007f
#define SHOOT_MOTOR_3508_SPEED_PID_KD1 0.0f

#define SHOOT_MOTOR_3508_SPEED_PID_KP2 4.0f
#define SHOOT_MOTOR_3508_SPEED_PID_KI2 0.00007f
#define SHOOT_MOTOR_3508_SPEED_PID_KD2 0.0f
#define SHOOT_MOTOR_3508_SPEED_PID_MAX_OUT 3000.0f
#define SHOOT_MOTOR_3508_SPEED_PID_MAX_IOUT 2000.0f

shoot_motor_t shoot_m2006[2] = {0};
shoot_motor_t shoot_motor_3508[2] = {0};
bool_t flag_cooling_limit[2] = {0};
uint16_t cooling_limit_cnt[2] = {0};
uint8_t fric_state = 0;					  // 0:off,1:on
uint8_t dial_mode = 0, if_single_hit = 0; // mode:0 continue,1 single
int16_t dial_speed = 0;
motor_measure_t shoot_motor_measure[2];
// uint16_t fric_speed = 1005;  #pwm控摩擦轮用的参数
uint16_t shoot_flag = 0, shoot_cnt = 0, safe_flag = 0;

const int target_rpm_define = 6300; // Ħ����Ŀ��ת��6400

void Shoot_Motor_Init(void)
{
	const static fp32 shoot_motor_speed_pid[3] = {SHOOT_MOTOR_SPEED_PID_KP, SHOOT_MOTOR_SPEED_PID_KI, SHOOT_MOTOR_SPEED_PID_KD};
	const static fp32 shoot_motor_angle_pid[3] = {SHOOT_MOTOR_ANGLE_PID_KP, SHOOT_MOTOR_ANGLE_PID_KI, SHOOT_MOTOR_ANGLE_PID_KD};

	PID_init(&shoot_m2006[0].speed_pid, PID_POSITION, shoot_motor_speed_pid, SHOOT_MOTOR_SPEED_PID_MAX_OUT, SHOOT_MOTOR_SPEED_PID_MAX_IOUT);
	PID_init(&shoot_m2006[0].angle_pid, PID_POSITION, shoot_motor_angle_pid, SHOOT_MOTOR_ANGLE_PID_MAX_OUT, SHOOT_MOTOR_ANGLE_PID_MAX_IOUT);

	const static fp32 shoot_motor_3508_speed_pid1[3] = {SHOOT_MOTOR_3508_SPEED_PID_KP1, SHOOT_MOTOR_3508_SPEED_PID_KI1, SHOOT_MOTOR_3508_SPEED_PID_KD1};
	const static fp32 shoot_motor_3508_speed_pid2[3] = {SHOOT_MOTOR_3508_SPEED_PID_KP2, SHOOT_MOTOR_3508_SPEED_PID_KI2, SHOOT_MOTOR_3508_SPEED_PID_KD2};

	PID_init(&shoot_motor_3508[0].speed_pid, PID_POSITION, shoot_motor_3508_speed_pid1, SHOOT_MOTOR_SPEED_PID_MAX_OUT, SHOOT_MOTOR_SPEED_PID_MAX_IOUT);
	PID_init(&shoot_motor_3508[1].speed_pid, PID_POSITION, shoot_motor_3508_speed_pid2, SHOOT_MOTOR_SPEED_PID_MAX_OUT, SHOOT_MOTOR_SPEED_PID_MAX_IOUT);
}

void CAN_Shoot_CMD(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4) //-30000,+30000
{
	CAN_TxHeaderTypeDef shoot_tx_message;
	uint8_t shoot_can_send_data[8];
	uint32_t send_mail_box;
	shoot_tx_message.StdId = CAN_SHOOT_ALL_ID;
	shoot_tx_message.IDE = CAN_ID_STD;
	shoot_tx_message.RTR = CAN_RTR_DATA;
	shoot_tx_message.DLC = 0x08;
	shoot_can_send_data[0] = motor1 >> 8;
	shoot_can_send_data[1] = motor1;
	shoot_can_send_data[2] = motor2 >> 8;
	shoot_can_send_data[3] = motor2;
	shoot_can_send_data[4] = motor3 >> 8;
	shoot_can_send_data[5] = motor3;
	shoot_can_send_data[6] = motor4 >> 8;
	shoot_can_send_data[7] = motor4;

	HAL_CAN_AddTxMessage(&SHOOT_CAN, &shoot_tx_message, shoot_can_send_data, &send_mail_box);
}

void Shoot_Motor_Data_Update(void) // ֻ�õ���shoot_m2006[0]
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

// extern ext_shoot_data_t shoot_data_t;
// float bullet_speed_max=0,bullet_speed_min=30;

void Fric_Motor_Control(void)
{
	if (fric_state)
	{
		shoot_motor_3508[0].speed_set = target_rpm_define;
		shoot_motor_3508[1].speed_set = -target_rpm_define;

		if ((shoot_motor_3508[0].speed > 6000) && (shoot_motor_3508[1].speed < -6000))
		{
			shoot_flag = 1;
		}
	}
	if (fric_state == 0)
	{
		shoot_motor_3508[0].speed_set = 0;
		shoot_motor_3508[1].speed_set = 0;
	}

	PID_calc(&shoot_motor_3508[0].speed_pid, shoot_motor_3508[0].speed, shoot_motor_3508[0].speed_set);
	PID_calc(&shoot_motor_3508[1].speed_pid, shoot_motor_3508[1].speed, shoot_motor_3508[1].speed_set);

	shoot_motor_3508[0].target_current = shoot_motor_3508[0].speed_pid.out; // - dif_pid.out;
	shoot_motor_3508[1].target_current = shoot_motor_3508[1].speed_pid.out; // + dif_pid.out;
}

uint16_t dial_single_cnt = 0;
uint32_t dial_stop_cnt = 0;

void Dial_Motor_Control(void)
{
	if (Game_Robot_State.power_management_shooter_output == 0x01)
	{
		if (abs(shoot_m2006[0].target_current) > 18000 && fabs(shoot_m2006[0].speed) < 100) // ��ת�Լ� ��ת������
		{
			dial_stop_cnt++;
			if (dial_stop_cnt > 250)
			{
				if (shoot_m2006[0].target_current < 0)
					shoot_m2006[0].target_current = 5000;
				else
					shoot_m2006[0].target_current = -5000;
				vTaskDelay(300);
				Shoot_Motor_Init();
				PID_clear(&shoot_m2006[0].speed_pid);
				PID_clear(&shoot_m2006[0].angle_pid);
				dial_stop_cnt = 0;
				shoot_m2006[0].angle_set = dial_angle;
			}
		}
	}

	if (shoot_flag == 1) // Ħ�����ѿ� �����̿���ת
	{
		if (dial_mode == 0) // continue ����ģʽ
		{
			shoot_m2006[0].speed_set = dial_speed; // ��switch_task�и�ֵ
		}
		else // single ����ģʽ
		{
			if (if_single_hit == 1) // һС��ʱ���if_single_hit �� 1 �� 0
			{
				dial_single_cnt++;
				if (dial_single_cnt > 50)
				{
					if_single_hit = 0;
					dial_single_cnt = 0;
				}
			}

			PID_calc(&shoot_m2006[0].angle_pid, shoot_m2006[0].angle, shoot_m2006[0].angle_set);
			shoot_m2006[0].speed_set = shoot_m2006[0].angle_pid.out;
		}
	}
	else // Ħ����û��
	{
		shoot_m2006[0].speed_set = 0;
	}
	PID_calc(&shoot_m2006[0].speed_pid, shoot_m2006[0].speed, shoot_m2006[0].speed_set);
	//	PID_calc(&shoot_m2006[1].speed_pid,shoot_m2006[1].speed,shoot_m2006[1].speed_set);

	shoot_m2006[0].target_current = shoot_m2006[0].speed_pid.out;
	//	shoot_m2006[1].give_current=shoot_m2006[1].speed_pid.out;
}

void Shoot_Power_Control() // ���ݲ���ϵͳ���ص������ж��Ƿ���
{
	if (Power_Heat_Data.shooter_17mm_1_barrel_heat >= (Game_Robot_State.shooter_barrel_heat_limit - 60)) // ǹ��1��ȴֵ����
	{
		flag_cooling_limit[0] = 1;
	}
	if (Power_Heat_Data.shooter_17mm_2_barrel_heat >= (Game_Robot_State.shooter_barrel_heat_limit - 60))
	{
		flag_cooling_limit[1] = 1;
	}
	if (flag_cooling_limit[0] != 0 || flag_cooling_limit[1] != 0) // ������1ͣת �Ȱ���     /////�в���ϵͳ�ǵ�ע�ͻ���
	{

		shoot_m2006[0].angle_set = shoot_m2006[0].angle;
		shoot_m2006[0].speed_set = 0;
		shoot_m2006[0].target_current = 0;

		PID_clear(&shoot_m2006[0].speed_pid);
		PID_clear(&shoot_m2006[0].angle_pid);

		cooling_limit_cnt[0]++;

		if (cooling_limit_cnt[0] >= 250) // 500ms �Ȱ����ٴ�
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

		Fric_Motor_Control(); // Ħ����Ĭ�Ͽ�������
		Dial_Motor_Control();
		if (rc_ctrl.rc.s[1] == RC_SW_UP || AutoAim_Data_Receive.track)
		{
			Shoot_Power_Control();
		}

		vTaskDelay(2);
	}
}

// Ħ���ֻ�����
// void Fric_PWR(uint8_t power){
//	if(power){
//		if(TIM1->CCR1 < fric_speed){       //Ħ����PWMֵδ�ﵽ
//			TIM1->CCR1 += 3, TIM1->CCR2 += 3;
//       TIM1->CCR3 += 3, TIM1->CCR4 += 3;		//PWMֵ��������Ħ���ֹ���������
//		}
//			shoot_flag=1;
//	}
//	else{
//		if(TIM1->CCR1 >= 1000){					//��֤Ħ����ת���ڻ�û׼��������Ħ����ʱΪ0
//			TIM1->CCR1 -= 4,TIM1->CCR2 -= 4;
//       TIM1->CCR3 -= 4,TIM1->CCR4 -= 4;			//PWMֵ������С��Ħ���ֹ����𽥼�С
//		}
//	}
// }