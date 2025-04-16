#include "Chassis_Task.h"
#include "Gimbal_Task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "can.h"
#include "remote_control.h"
#include "bsp_can.h"
#include "arm_math.h"
#include "referee.h"
#include "bsp_cap.h"
#include "Nmanifold_usbd_task.h"
#include "detect_task.h"

#define NORMAL_VX_MAX 4000
#define NORMAL_VY_MAX 4000
#define ROTATE_VX_MAX 3000
#define ROTATE_VY_MAX 3000
// #define ROTATE_WZ_MAX 25000
#define ROTATE_WZ_MAX 22000
#define ROTATE_WZ_MIN -10000
#define ROTATE_WEAK 0.5f
#define CHASSIS_FOLLOW_GIMBAL_ANGLE_ZERO 4450
#define CHASSIS_FOLLOW_GIMBAL_ANGLE_LEFT_ZERO 6498
#define CHASSIS_FOLLOW_GIMBAL_ANGLE_RIGHT_ZERO 2402
#define CHASSIS_FOLLOW_GIMBAL_ANGLE_BACK_ZERO 8544
#define NO_JUDGE_TOTAL_CURRENT_LIMIT 64000.0f
// #define BUFFER_TOTAL_CURRENT_LIMIT      16000.0f
#define BUFFER_TOTAL_CURRENT_LIMIT 30000.0f
#define POWER_TOTAL_CURRENT_LIMIT 30000.0f
#define WARNING_POWER_BUFF 60.0f

#define M3505_MOTOR_SPEED_PID_KP 12.0f
#define M3505_MOTOR_SPEED_PID_KI 0.008f
#define M3505_MOTOR_SPEED_PID_KD 0.0f
#define M3505_MOTOR_SPEED_PID_MAX_OUT 16000.0f
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 2000.0f

#define CHASSIS_FOLLOW_GIMBAL_PID_KP 450.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KI 0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KD 5000.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT 20000.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT 1500.0f

#define CHASSIS_POWER_PID_KP 0.03f
#define CHASSIS_POWER_PID_KI 0.00f
#define CHASSIS_POWER_PID_KD 0.0f
#define CHASSIS_POWER_PID_MAX_OUT 1.0f
#define CHASSIS_POWER_PID_MAX_IOUT 1000.0f

// ??????? rp/m->m/s
#define M3508_MOTOR_RPM_TO_VECTOR 0.03547934768011173503001388518321f
// ??????? rp->m
#define M3508_MOTOR_ECD_TO_DISTANCE 0.00000415809748903494517209f

/********************?????????????**********************/
const fp32 toque_coefficient = 1.99688994e-6f; //
const fp32 k1 = 1.23e-07;					   // k1
const fp32 k2 = 1.453e-07;					   // k2
const fp32 constant = 4.081f;
/*****************************************************/

/*************************?§Ö?????§Ý?§³????????2025???????????????—¥****************************/
typedef enum
{
	HEALTH_NORMAL, // ??????
	HEALTH_HURT	   // ?????????
} health_state_t;

health_state_t health_state = HEALTH_NORMAL;
/*****************************************************/

chassis_motor_t chassis_m3508[4] = {0};
chassis_control_t chassis_control;
uint8_t chassis_follow_gimbal_zerochange_flag = 0;
fp32 chassis_follow_gimbal_zero_actual = CHASSIS_FOLLOW_GIMBAL_ANGLE_ZERO;
fp32 chassis_power_limit = 0, chassis_power_buffer = 0;
double init_chassis_power = 0.0;
const fp32 factor[3] = {1.54, 1.54, 1.54};

static void CAN_Chassis_CMD(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4) //-16384,+16384
{
	static CAN_TxHeaderTypeDef chassis_tx_message;
	static uint8_t chassis_can_send_data[8];
	uint32_t send_mail_box;
	chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID;
	chassis_tx_message.IDE = CAN_ID_STD;
	chassis_tx_message.RTR = CAN_RTR_DATA;
	chassis_tx_message.DLC = 0x08;
	chassis_can_send_data[0] = motor1 >> 8;
	chassis_can_send_data[1] = motor1;
	chassis_can_send_data[2] = motor2 >> 8;
	chassis_can_send_data[3] = motor2;
	chassis_can_send_data[4] = motor3 >> 8;
	chassis_can_send_data[5] = motor3;
	chassis_can_send_data[6] = motor4 >> 8;
	chassis_can_send_data[7] = motor4;

	HAL_StatusTypeDef status;
	status = HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
	if (status != HAL_OK)
	{
		// ???????????
		CAN_TxQueue_Push(&chassis_tx_message, chassis_can_send_data);
	}
}
	void CAN_Cap_CMD(float data1, float data2, float data3, float data4)
	{
		CAN_TxHeaderTypeDef cap_tx_message;
		uint8_t cap_can_send_data[8];
		uint32_t send_mail_box;
		cap_tx_message.StdId = CAN_CAP_TX_ID;
		cap_tx_message.IDE = CAN_ID_STD;
		cap_tx_message.RTR = CAN_RTR_DATA;
		cap_tx_message.DLC = 0x08;

		uint16_t temp;

		temp = data1 * 100;
		cap_can_send_data[0] = temp;
		cap_can_send_data[1] = temp >> 8;
		temp = data2 * 100;
		cap_can_send_data[2] = temp;
		cap_can_send_data[3] = temp >> 8;
		temp = data3 * 100;
		cap_can_send_data[4] = temp;
		cap_can_send_data[5] = temp >> 8;
		temp = data4 * 100;
		cap_can_send_data[6] = temp;
		cap_can_send_data[7] = temp >> 8;

		HAL_CAN_AddTxMessage(&CHASSIS_CAN, &cap_tx_message, cap_can_send_data, &send_mail_box);
	}

	void Chassis_Motor_Init(void)
	{
		const static fp32 motor_speed_pid[3] = {M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD};
		const static fp32 chassis_follow_gimbal_pid[3] = {CHASSIS_FOLLOW_GIMBAL_PID_KP, CHASSIS_FOLLOW_GIMBAL_PID_KI, CHASSIS_FOLLOW_GIMBAL_PID_KD};

		for (uint8_t i = 0; i < 4; i++)
		{
			PID_init(&chassis_m3508[i].pid, PID_POSITION, motor_speed_pid, M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT);
		}
		PID_init(&chassis_control.chassis_follow_gimbal_pid, PID_POSITION, chassis_follow_gimbal_pid, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT);
	}

	void power_control()
	{
		fp32 scaled_give_power[4];
		uint8_t chassis_power_limit_rmul = Game_Robot_State.chassis_power_limit;

		init_chassis_power = 0;

		static float initial_give_power[4];
		for (uint8_t i = 0; i < 4; i++)
		{
			initial_give_power[i] = toque_coefficient * chassis_m3508[i].give_current * chassis_m3508[i].speed // ???????????RM2023-?????????????????????????????????
									+ k1 * chassis_m3508[i].give_current * chassis_m3508[i].give_current + k2 * chassis_m3508[i].speed * chassis_m3508[i].speed + constant;
			if (initial_give_power[i] < 0) // ????????????????????????
				continue;
			init_chassis_power += initial_give_power[i];
		}

		if (init_chassis_power > chassis_power_limit_rmul)
		{
			fp32 power_scale = chassis_power_limit_rmul / init_chassis_power;
			for (uint8_t i = 0; i < 4; i++)
			{
				scaled_give_power[i] = initial_give_power[i] * power_scale; // ?????????????????
				if (scaled_give_power[i] < 0)
				{
					continue;
				}
				fp32 b = toque_coefficient * chassis_m3508[i].speed;
				fp32 c = k2 * chassis_m3508[i].speed * chassis_m3508[i].speed - scaled_give_power[i] + constant;

				if (chassis_m3508[i].give_current > 0) // ????????????????pid????????????????????
				{
					fp32 temp = (-b + sqrt(b * b - 4 * k1 * c)) / (2 * k1);
					if (temp > 16000)
					{
						chassis_m3508[i].give_current = 16000;
					}
					else
						chassis_m3508[i].give_current = temp;
				}
				else
				{
					fp32 temp = (-b - sqrt(b * b - 4 * k1 * c)) / (2 * k1);
					if (temp < -16000)
					{
						chassis_m3508[i].give_current = -16000;
					}
					else
						chassis_m3508[i].give_current = temp;
				}
			}
		}
	}

	void Chassis_Motor_Data_Update(void)
	{
		static float lpf_ratio = 0.0f;
		static fp32 chassis_m3508_last_speed[4];
		for (uint8_t i = 0; i < 4; i++)
		{
			chassis_m3508[i].speed = motor_measure_chassis[i].speed_rpm;

			chassis_m3508[i].speed = (chassis_m3508[i].speed) * (1 - lpf_ratio) + chassis_m3508_last_speed[i] * lpf_ratio;
			chassis_m3508_last_speed[i] = motor_measure_chassis[i].speed_rpm;
		}
		//	chassis_power_limit=Game_Robot_State.chassis_power_limit; // ???????????
		chassis_power_buffer = Power_Heat_Data.buffer_energy; // ???????????
	}

	static void chassis_vector_to_mecanum_wheel_speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set, fp32 *wheel0, fp32 *wheel1, fp32 *wheel2, fp32 *wheel3)
	{
		// because the gimbal is in front of chassis, when chassis rotates, wheel 0 and wheel 1 should be slower and wheel 2 and wheel 3 should be faster
		// ???????? ????????????????????????? 0 ??1 ????????????? ???????? 2,3 ??????????
		*wheel0 = (+vx_set + vy_set) - MOTOR_DISTANCE_TO_CENTER * wz_set;
		*wheel1 = (+vx_set - vy_set) - MOTOR_DISTANCE_TO_CENTER * wz_set;
		*wheel2 = (-vx_set - vy_set) - MOTOR_DISTANCE_TO_CENTER * wz_set;
		*wheel3 = (-vx_set + vy_set) - MOTOR_DISTANCE_TO_CENTER * wz_set;
	}

	void Health_Monitor_Update(void)
	{

		static uint32_t hurt_start_time = 0;
		static uint16_t last_health = 0;
		uint16_t current_health = Game_Robot_State.current_HP; // ?????????

		switch (health_state)
		{
		case HEALTH_NORMAL:
			if (current_health < last_health && Robot_Hurt.hurt_type == 0 && Robot_Hurt.armor_type != 0) // ????????Ûš??
			{
				health_state = HEALTH_HURT;
				hurt_start_time = xTaskGetTickCount();
			}
			break;

		case HEALTH_HURT:
			if (current_health < last_health && Robot_Hurt.hurt_type == 0 && Robot_Hurt.armor_type != 0) // ??????????
			{
				hurt_start_time = xTaskGetTickCount(); // ???¨¹??
			}
			else if (xTaskGetTickCount() - hurt_start_time > pdMS_TO_TICKS(3000))
			{
				health_state = HEALTH_NORMAL;
			}
			break;
		}

		last_health = current_health; // ??????????
	}

	float Limit_To_180(float in)
	{
		while (in < -180 || in > 180)
		{					   // ?????????????????????[-??,??]????
			if (in < -180)	   // ?????????§³??-??
				in = in + 360; // ????2??
			else if (in > 180) // ?????????????
				in = in - 360; // ??§³2??
		}
		return in;
	}

	fp32 Angle_Z_Suit_ZERO_Get(fp32 Target_Angle, fp32 Target_Speed)
	{
		float angle_front_err = Limit_To_180((float)(Target_Angle - CHASSIS_FOLLOW_GIMBAL_ANGLE_ZERO) / 8192.0f * 360.0f); // ?????????????????????[-??,??]????

		// ????????§³???????
		if (angle_front_err > 135 || angle_front_err <= -135)
			return CHASSIS_FOLLOW_GIMBAL_ANGLE_BACK_ZERO;
		if (angle_front_err > 45 && angle_front_err <= 135)
			return CHASSIS_FOLLOW_GIMBAL_ANGLE_LEFT_ZERO;
		if (angle_front_err > -45 && angle_front_err <= 45)
			return CHASSIS_FOLLOW_GIMBAL_ANGLE_ZERO;
		if (angle_front_err > -135 && angle_front_err <= -45)
			return CHASSIS_FOLLOW_GIMBAL_ANGLE_RIGHT_ZERO;
	}

	void chassis_vector_set(void)
	{

		if (rc_ctrl.rc.s[1] == RC_SW_DOWN) // stop
		{
			chassis_control.vx = 0;
			chassis_control.vy = 0;
			chassis_control.wz = 0;
			chassis_follow_gimbal_zerochange_flag = 1; // ??????0 ?????????§Ý???????? ??????????????????????
		}
		else if ((rc_ctrl.rc.s[1] == RC_SW_MID) && rc_ctrl.rc.ch[4] < 500 && rc_ctrl.rc.ch[4] > -500) // normal move ??????? ??????????
		{
			if (chassis_follow_gimbal_zerochange_flag == 1)
			{
				// ?????? ??????yaw??????????
				chassis_follow_gimbal_zero_actual = Angle_Z_Suit_ZERO_Get(gimbal_m6020[0].ENC_angle, gimbal_m6020[0].ENC_speed); // ?????????§Ý?
				chassis_follow_gimbal_zerochange_flag = 0;
			}
			chassis_control.chassis_follow_gimbal_angle = (float)(((uint16_t)gimbal_m6020[0].ENC_angle + (8192 - (uint16_t)chassis_follow_gimbal_zero_actual)) % 8192) / 8192.0f * 360.0f;
			//      chassis_control.chassis_follow_gimbal_angle=(Angle_Z_Suit_Err_Get((float)(gimbal_m6020[0].ENC_angle)/8192.0f*360,gimbal_m6020[0].ENC_speed))/(2*PI)*360;
			if (chassis_control.chassis_follow_gimbal_angle > 180)
			{
				chassis_control.chassis_follow_gimbal_angle -= 360;
			}
			PID_calc(&chassis_control.chassis_follow_gimbal_pid, chassis_control.chassis_follow_gimbal_angle, 0);
			chassis_control.wz = -chassis_control.chassis_follow_gimbal_pid.out;

			fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;
			fp32 vx, vy;
			float sin_yaw_rad;

			vx = ramp_control(0, rc_ctrl.rc.ch[2] * 30, 0.3f);
			vy = ramp_control(0, rc_ctrl.rc.ch[3] * 30, 0.3f);

			sin_yaw_rad = -(chassis_control.chassis_follow_gimbal_angle + Limit_To_180((float)(chassis_follow_gimbal_zero_actual - CHASSIS_FOLLOW_GIMBAL_ANGLE_ZERO) / 8192.0f * 360.0f)) / 180.0f * 3.14159f;
			sin_yaw = arm_sin_f32(sin_yaw_rad);
			cos_yaw = arm_cos_f32(sin_yaw_rad);
			chassis_control.vx = cos_yaw * vx + sin_yaw * vy;
			chassis_control.vy = -sin_yaw * vx + cos_yaw * vy;
		}
		else if (rc_ctrl.rc.s[1] == RC_SW_MID) // rotate mode §³????
		{
			// ????????????????chassis_follow_gimbal_angle????§³?????????????????????
			chassis_control.chassis_follow_gimbal_angle = (float)((uint16_t)((uint16_t)gimbal_m6020[0].ENC_angle + (8192 - CHASSIS_FOLLOW_GIMBAL_ANGLE_ZERO) - chassis_control.wz * 0.012) % 8192) / 8192.0f * 360.0f;
			if (chassis_control.chassis_follow_gimbal_angle > 180)
			{
				chassis_control.chassis_follow_gimbal_angle -= 360;
			}

			fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;
			fp32 vx, vy;
			static fp32 vx_last_rotate, vy_last_rotate;
			vx = ramp_control(0, rc_ctrl.rc.ch[2] * 12, 0.5f);
			vy = ramp_control(0, rc_ctrl.rc.ch[3] * 12, 0.5f);
			vx = 0.5 * vx + 0.5 * vx_last_rotate;
			vy = 0.5 * vy + 0.5 * vy_last_rotate;
			vx_last_rotate = ramp_control(0, rc_ctrl.rc.ch[2] * 8, 0.4f);
			vy_last_rotate = ramp_control(0, rc_ctrl.rc.ch[3] * 8, 0.4f);

			sin_yaw = arm_sin_f32(-(chassis_control.chassis_follow_gimbal_angle) / 180.0f * 3.14159f);
			cos_yaw = arm_cos_f32(-(chassis_control.chassis_follow_gimbal_angle) / 180.0f * 3.14159f);
			chassis_control.vx = cos_yaw * vx + sin_yaw * vy;
			chassis_control.vy = -sin_yaw * vx + cos_yaw * vy;

			if (rc_ctrl.rc.ch[4] <= -500)
			{
				chassis_control.wz = ROTATE_WZ_MAX;
			}
			else if (rc_ctrl.rc.ch[4] >= 500)
			{
				chassis_control.wz = ROTATE_WZ_MIN;
			}

			chassis_follow_gimbal_zerochange_flag = 1; // ???? ???§³?????§Ý??????????????????
		}
		else if (rc_ctrl.rc.s[1] == RC_SW_UP) // automatic mode NUC?????????
		{
			fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;
			fp32 vx, vy;

			vx = -(float)Chassis_Data_Receive.vy * factor[0] * 800;
			vy = (float)Chassis_Data_Receive.vx * factor[1] * 800;

			if (Rotate_Data_Receive.rotate == 0) // ??????????
			{
				chassis_control.chassis_follow_gimbal_angle = (float)(((uint16_t)gimbal_m6020[0].ENC_angle + (8192 - (uint16_t)chassis_follow_gimbal_zero_actual)) % 8192) / 8192.0f * 360.0f;
				if (chassis_control.chassis_follow_gimbal_angle > 180.0f)
				{
					chassis_control.chassis_follow_gimbal_angle -= 360.0f;
				}
				PID_calc(&chassis_control.chassis_follow_gimbal_pid, chassis_control.chassis_follow_gimbal_angle, 0);
				chassis_control.wz = -chassis_control.chassis_follow_gimbal_pid.out;

				sin_yaw = arm_sin_f32(-(chassis_control.chassis_follow_gimbal_angle + Limit_To_180((float)(chassis_follow_gimbal_zero_actual - CHASSIS_FOLLOW_GIMBAL_ANGLE_ZERO) / 8192.0f * 360.0f)) / 180.0f * 3.14159f);
				cos_yaw = arm_cos_f32(-(chassis_control.chassis_follow_gimbal_angle + Limit_To_180((float)(chassis_follow_gimbal_zero_actual - CHASSIS_FOLLOW_GIMBAL_ANGLE_ZERO) / 8192.0f * 360.0f)) / 180.0f * 3.14159f);
				chassis_control.vx = cos_yaw * vx + sin_yaw * vy;
				chassis_control.vy = -sin_yaw * vx + cos_yaw * vy;
			}
			else // §³????
			{
				chassis_control.chassis_follow_gimbal_angle = (float)((uint16_t)((uint16_t)gimbal_m6020[0].ENC_angle + (8192 - CHASSIS_FOLLOW_GIMBAL_ANGLE_ZERO) - chassis_control.wz * 0.01) % 8192) / 8192.0f * 360.0f;
				if (chassis_control.chassis_follow_gimbal_angle > 180)
				{
					chassis_control.chassis_follow_gimbal_angle -= 360;
				}
				sin_yaw = arm_sin_f32(-(chassis_control.chassis_follow_gimbal_angle) / 180.0f * 3.14159f);
				cos_yaw = arm_cos_f32(-(chassis_control.chassis_follow_gimbal_angle) / 180.0f * 3.14159f);
				chassis_control.vx = cos_yaw * vx + sin_yaw * vy;
				chassis_control.vy = -sin_yaw * vx + cos_yaw * vy;

				static float rotate_sine_angle = 0;
				static float rotate_sine_T = 1;

				//			rotate_sine_angle+=(0.001/rotate_sine_T*360);
				//			rotate_sine_angle=Limit_To_180(rotate_sine_angle);
				if (health_state == HEALTH_HURT)
				{
					chassis_control.wz = -(float)Rotate_Data_Receive.rotate;
				}
				else
				{
					chassis_control.wz = -(float)Rotate_Data_Receive.rotate * ROTATE_WEAK;
				}
				chassis_follow_gimbal_zerochange_flag = 1;
			}
		}
	}

	void chassis_power_cap_control(void) // ??????¨®???
	{
		float total_current_limit = 0.0f;
		float total_current = 0.0f;

		if (cap_data.cap_per == 0)
		{
			total_current_limit = NO_JUDGE_TOTAL_CURRENT_LIMIT;
		}
		else
		{
			if (cap_data.cap_per < 0.3f)
			{
				float power_scale;

				power_scale = cap_data.cap_per / 0.3f;
				total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT * power_scale;
			}
			else
			{
				total_current_limit = NO_JUDGE_TOTAL_CURRENT_LIMIT;
				//			if(cap_data.chassis_power > chassis_power_limit*0.75f)
				//			{
				//				float power_scale;
				//				if(cap_data.chassis_power < chassis_power_limit)
				//				{
				//					power_scale = (chassis_power_limit - cap_data.chassis_power) / (chassis_power_limit - chassis_power_limit*0.75f);
				//				}
				//				else
				//				{
				//					power_scale = 0.0f;
				//				}
				//
				//				total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT + POWER_TOTAL_CURRENT_LIMIT * power_scale;
				//			}
				//			else
				//			{
				//				total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT + POWER_TOTAL_CURRENT_LIMIT;
				//			}
			}
		}
		total_current = 0.0f;
		for (uint8_t i = 0; i < 4; i++)
		{
			total_current += fabs((float)chassis_m3508[i].give_current);
		}

		if (total_current > total_current_limit)
		{
			float current_scale = total_current_limit / total_current;
			chassis_m3508[0].give_current *= current_scale;
			chassis_m3508[1].give_current *= current_scale;
			chassis_m3508[2].give_current *= current_scale;
			chassis_m3508[3].give_current *= current_scale;
		}
	}

	extern fp32 INS_angle[3];
	void chassis_feedback_update(void)
	{
		for (uint8_t i = 0; i < 4; i++)
		{
			//		fp32 temp_angle=motor_measure_chassis[i].ecd-motor_measure_chassis[i].last_ecd;
			//		if(temp_angle>=4096)
			//		{
			//			temp_angle=(8192.0f-motor_measure_chassis[i].ecd)+motor_measure_chassis[i].last_ecd;
			//		}
			//		else if(temp_angle<=-4096)
			//		{
			//			temp_angle=(8192.0f-motor_measure_chassis[i].last_ecd)+motor_measure_chassis[i].ecd;
			//		}
			// ?????????????????????PID???
			//		chassis_m3508[i].speed = M3508_MOTOR_RPM_TO_VECTOR * motor_measure_chassis[i].speed_rpm; // rp/s -> m/s
		}
		Chassis_Data_Tramsit.y = (-motor_measure_chassis[0].code + motor_measure_chassis[1].code + motor_measure_chassis[2].code - motor_measure_chassis[3].code) * 0.70710678f / 4.0f * M3508_MOTOR_ECD_TO_DISTANCE;
		Chassis_Data_Tramsit.x = (-motor_measure_chassis[0].code - motor_measure_chassis[1].code + motor_measure_chassis[2].code + motor_measure_chassis[3].code) * 0.70710678f / 4.0f * M3508_MOTOR_ECD_TO_DISTANCE;
		Chassis_Data_Tramsit.z = INS_angle[0] / 57.18f;
		Chassis_Data_Tramsit.vy = (-chassis_m3508[0].speed + chassis_m3508[1].speed + chassis_m3508[2].speed - chassis_m3508[3].speed) * 0.70710678f / 4.0f;
		Chassis_Data_Tramsit.vx = (-chassis_m3508[0].speed - chassis_m3508[1].speed + chassis_m3508[2].speed + chassis_m3508[3].speed) * 0.70710678f / 4.0f;
		Chassis_Data_Tramsit.wz = (-chassis_m3508[0].speed - chassis_m3508[1].speed - chassis_m3508[2].speed - chassis_m3508[3].speed) / 4.0f;
	}

	/**
	 * @brief  ???¦Ï???
	 * @param  ???????Ú…???????
	 * @retval ????
	 */
	float ramp_control(float ref, float set, float accel)
	{
		fp32 ramp = limit(accel, 0, 1) * (set - ref);
		return ref + ramp;
	}

	/**
	 * @brief  ?????¦¶????
	 * @param  ????????,??§³?,????
	 * @retval ???????
	 */
	fp32 limit(float data, float min, float max)
	{
		if (data >= max)
			return max;
		if (data <= min)
			return min;
		return data;
	}

	void Chassis_Task(void const *argument)
	{
		Chassis_Motor_Init();

		vTaskDelay(200);

		while (1)
		{
			Chassis_Motor_Data_Update();

			Health_Monitor_Update();

			chassis_vector_set();

			chassis_vector_to_mecanum_wheel_speed(chassis_control.vx, chassis_control.vy, chassis_control.wz, &chassis_m3508[0].speed_set, &chassis_m3508[1].speed_set, &chassis_m3508[2].speed_set, &chassis_m3508[3].speed_set);

			for (uint8_t i = 0; i < 4; i++)
			{

				PID_calc(&chassis_m3508[i].pid, chassis_m3508[i].speed, chassis_m3508[i].speed_set);

				chassis_m3508[i].give_current = chassis_m3508[i].pid.out;
			}

			//				if(rc_ctrl.rc.s[1]==RC_SW_DOWN||toe_is_error(DBUS_TOE)||(Game_Status.game_progress!=4 && rc_ctrl.rc.s[1]==RC_SW_UP)) //?????detect_task
			if (rc_ctrl.rc.s[1] == RC_SW_DOWN || toe_is_error(DBUS_TOE))
				CAN_Chassis_CMD(0, 0, 0, 0);
			else
			{
				//				chassis_feedback_update();
				power_control();
				//				chassis_power_cap_control();
				CAN_Chassis_CMD(chassis_m3508[0].give_current, chassis_m3508[1].give_current, chassis_m3508[2].give_current, chassis_m3508[3].give_current);
			}

			vTaskDelay(2);
		}
	}
