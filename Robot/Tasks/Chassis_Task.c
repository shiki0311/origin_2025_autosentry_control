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
#define ROTATE_WEAK 0.3f
#define CHASSIS_FOLLOW_GIMBAL_ANGLE_ZERO 4450
#define CHASSIS_FOLLOW_GIMBAL_ANGLE_LEFT_ZERO 6498
#define CHASSIS_FOLLOW_GIMBAL_ANGLE_RIGHT_ZERO 2402
#define CHASSIS_FOLLOW_GIMBAL_ANGLE_BACK_ZERO 8544
#define NO_JUDGE_TOTAL_CURRENT_LIMIT 64000.0f
#define BUFFER_TOTAL_CURRENT_LIMIT 30000.0f
#define POWER_TOTAL_CURRENT_LIMIT 30000.0f
#define WARNING_POWER_BUFF 60.0f

#define M3505_MOTOR_SPEED_PID_KP 10.0f
#define M3505_MOTOR_SPEED_PID_KI 0.006f
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

#define NAV_SPEED_FAST 800.0f  //导航发过来的速度乘以的系数，非上坡用
#define NAV_SPEED_SLOW 400.0f  //导航发过来的速度乘以的系数，非上坡用


//  rp/m->m/s
#define M3508_MOTOR_RPM_TO_VECTOR 0.03547934768011173503001388518321f
//  rp->m
#define M3508_MOTOR_ECD_TO_DISTANCE 0.00000415809748903494517209f

/********************电机功率控制参数**********************/
const fp32 toque_coefficient = 1.99688994e-6f;
const fp32 k1 = 1.23e-07;  // k1
const fp32 k2 = 1.453e-07; // k2
const fp32 constant = 4.081f;
/*****************************************************/

/*************************中弹掉血切换小陀螺模式（2025赛季节省功率特供版）****************************/
typedef enum
{
	HEALTH_NORMAL, // 正常模式
	HEALTH_HURT	   // 受伤旋转模式
} health_state_t;

health_state_t health_state = HEALTH_NORMAL;
/*****************************************************/

/*************************底盘功率上限枚举体，不同的值对应不同的底盘功率上限****************************/
typedef enum
{
	REMOTE_CONTROL, //
	NAV_NORMAL_MODE,
	HURT,
	UPHILL_START,
	ON_HILL
} chassis_max_power_control_t;

chassis_max_power_control_t chassis_max_power_control_flag = NAV_NORMAL_MODE;
/*****************************************************/
chassis_motor_t chassis_m3508[4] = {0};
chassis_control_t chassis_control;
uint8_t chassis_follow_gimbal_zerochange_flag = 0;
fp32 chassis_follow_gimbal_zero_actual = CHASSIS_FOLLOW_GIMBAL_ANGLE_ZERO;
fp32 chassis_power_limit = 0, chassis_power_buffer = 0;
fp32 init_chassis_power = 0.0;
const fp32 factor[3] = {1.54, 1.54, 1.54};

static void CAN_Chassis_CMD(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4) //-16384,+16384
{
	CAN_TxHeaderTypeDef chassis_tx_message;
	uint8_t chassis_can_send_data[8];
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
		// can发送失败送入can发送队列中
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

	HAL_StatusTypeDef status;
	status = HAL_CAN_AddTxMessage(&CHASSIS_CAN, &cap_tx_message, cap_can_send_data, &send_mail_box);
	if (status != HAL_OK)
	{
		// can发送失败送入can发送队列中
		CAN_TxQueue_Push(&cap_tx_message, cap_can_send_data);
	}
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

static void Chassis_Max_Power_Update() // 根据不同模式选择不同底盘功率上限
{
	if (!cap_recieve_flag)
	{
		chassis_power_limit = Game_Robot_State.chassis_power_limit - 5;
		return;
	}
	switch (rc_ctrl.rc.s[1])
	{
	case RC_SW_MID:
		chassis_max_power_control_flag = REMOTE_CONTROL;
		break;
	case RC_SW_UP:
		if (AutoAim_Data_Receive.uphill_flag == 1)
			chassis_max_power_control_flag = UPHILL_START;
		else if (AutoAim_Data_Receive.uphill_flag == 2)
			chassis_max_power_control_flag = ON_HILL;
		else if (health_state == HEALTH_HURT)
			chassis_max_power_control_flag = HURT;
		else if (health_state == HEALTH_NORMAL)
			chassis_max_power_control_flag = NAV_NORMAL_MODE;
		break;
	}
	if (cap_data.cap_per > 0.3f) // 超电还没榨干就再压榨一下
	{
		switch (chassis_max_power_control_flag)
		{
		case REMOTE_CONTROL:
			chassis_power_limit = Game_Robot_State.chassis_power_limit + cap_data.cap_per * 100;
			break;
		case NAV_NORMAL_MODE:
			chassis_power_limit = Game_Robot_State.chassis_power_limit - 5;
			break;
		case HURT:
			chassis_power_limit = Game_Robot_State.chassis_power_limit + cap_data.cap_per * 70;
			break;
		case UPHILL_START:
			chassis_power_limit = Game_Robot_State.chassis_power_limit + cap_data.cap_per * 100;
			break;
		case ON_HILL:
			chassis_power_limit = Game_Robot_State.chassis_power_limit + 100;
			break;
		}
	}
	else
	{
		chassis_power_limit = Game_Robot_State.chassis_power_limit - (1 - cap_data.cap_per) * 10;
	}
}

static float Nav_Chassis_Rotate_Set() // 导航底盘旋转速度控制
{
	fp32 nav_wz;
	
	if (AutoAim_Data_Receive.uphill_flag == 2)
		nav_wz = 0;

	else if (health_state == HEALTH_HURT)
	{
		nav_wz = -(float)AutoAim_Data_Receive.rotate;
	}
	else
	{
		nav_wz = -(float)AutoAim_Data_Receive.rotate * ROTATE_WEAK;
	}
	return nav_wz;
}

void power_control()
{
	fp32 scaled_give_power[4];
	fp32 initial_give_power[4];
	fp32 final_give_power[4];

	init_chassis_power = 0;

	Chassis_Max_Power_Update();

	for (uint8_t i = 0; i < 4; i++)
	{
		initial_give_power[i] = toque_coefficient * chassis_m3508[i].give_current * chassis_m3508[i].speed + k1 * chassis_m3508[i].give_current * chassis_m3508[i].give_current + k2 * chassis_m3508[i].speed * chassis_m3508[i].speed + constant;
		if (initial_give_power[i] < 0)
			continue;
		init_chassis_power += initial_give_power[i];
	}

	if (init_chassis_power > chassis_power_limit)
	{
		fp32 power_scale = chassis_power_limit / init_chassis_power;
		for (uint8_t i = 0; i < 4; i++)
		{
			scaled_give_power[i] = initial_give_power[i] * power_scale;
			if (scaled_give_power[i] < 0)
			{
				continue;
			}
			fp32 b = toque_coefficient * chassis_m3508[i].speed;
			fp32 c = k2 * chassis_m3508[i].speed * chassis_m3508[i].speed - scaled_give_power[i] + constant;

			if (chassis_m3508[i].give_current > 0)
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
	//	chassis_power_limit=Game_Robot_State.chassis_power_limit; //
	chassis_power_buffer = Power_Heat_Data.buffer_energy; //
}

static void chassis_vector_to_mecanum_wheel_speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set, fp32 *wheel0, fp32 *wheel1, fp32 *wheel2, fp32 *wheel3)
{
	*wheel0 = (+vx_set + vy_set) - MOTOR_DISTANCE_TO_CENTER * wz_set;
	*wheel1 = (+vx_set - vy_set) - MOTOR_DISTANCE_TO_CENTER * wz_set;
	*wheel2 = (-vx_set - vy_set) - MOTOR_DISTANCE_TO_CENTER * wz_set;
	*wheel3 = (-vx_set + vy_set) - MOTOR_DISTANCE_TO_CENTER * wz_set;
}

void Health_Monitor_Update(void)
{
	if (rc_ctrl.rc.s[1] != RC_SW_UP)
		return;

	static uint32_t hurt_start_time = 0;
	static uint16_t last_health = 0;
	uint16_t current_health = Game_Robot_State.current_HP; // ?????????

	switch (health_state)
	{
	case HEALTH_NORMAL:
		if (current_health < last_health && Robot_Hurt.hurt_type == 0 && Robot_Hurt.armor_type != 0) // ????????蹥??
		{
			health_state = HEALTH_HURT;
			hurt_start_time = xTaskGetTickCount();
		}
		break;

	case HEALTH_HURT:
		if (current_health < last_health && Robot_Hurt.hurt_type == 0 && Robot_Hurt.armor_type != 0) // ??????????
		{
			hurt_start_time = xTaskGetTickCount();
		}
		else if (xTaskGetTickCount() - hurt_start_time > pdMS_TO_TICKS(3000))
		{
			health_state = HEALTH_NORMAL;
		}
		break;
	}
	last_health = current_health;
}

float Limit_To_180(float in)
{
	while (in < -180 || in > 180)
	{					   // ?????????????????????[-??,??]????
		if (in < -180)	   // ?????????锟斤拷??-??
			in = in + 360; // ????2??
		else if (in > 180) // ?????????????
			in = in - 360; // ??锟斤拷2??
	}
	return in;
}

fp32 Angle_Z_Suit_ZERO_Get(fp32 Target_Angle, fp32 Target_Speed)
{
	float angle_front_err = Limit_To_180((float)(Target_Angle - CHASSIS_FOLLOW_GIMBAL_ANGLE_ZERO) / 8192.0f * 360.0f); // ?????????????????????[-??,??]????

	// ????????锟斤拷???????
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
	static fp32 vx, vy;
	fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;

	if (rc_ctrl.rc.s[1] == RC_SW_DOWN) // stop
	{
		chassis_follow_gimbal_zerochange_flag = 1; // ??????0 ?????????锟斤拷???????? ??????????????????????
	}
	else if ((rc_ctrl.rc.s[1] == RC_SW_MID) && rc_ctrl.rc.ch[4] < 500 && rc_ctrl.rc.ch[4] > -500) // normal move
	{
		if (chassis_follow_gimbal_zerochange_flag == 1)
		{
			// ?????? ??????yaw??????????
			chassis_follow_gimbal_zero_actual = Angle_Z_Suit_ZERO_Get(gimbal_m6020[0].ENC_angle, gimbal_m6020[0].ENC_speed); // ?????????锟斤拷?
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

		fp32 sin_yaw_rad;

		vx = ramp_control(vx, rc_ctrl.rc.ch[2] * 9, 0.7f);
		vy = ramp_control(vy, rc_ctrl.rc.ch[3] * 9, 0.7f);

		sin_yaw_rad = -(chassis_control.chassis_follow_gimbal_angle + Limit_To_180((float)(chassis_follow_gimbal_zero_actual - CHASSIS_FOLLOW_GIMBAL_ANGLE_ZERO) / 8192.0f * 360.0f)) / 180.0f * 3.14159f;
		sin_yaw = arm_sin_f32(sin_yaw_rad);
		cos_yaw = arm_cos_f32(sin_yaw_rad);
		chassis_control.vx = cos_yaw * vx + sin_yaw * vy;
		chassis_control.vy = -sin_yaw * vx + cos_yaw * vy;
	}
	else if (rc_ctrl.rc.s[1] == RC_SW_MID) // rotate mode
	{
		// ????????????????chassis_follow_gimbal_angle
		chassis_control.chassis_follow_gimbal_angle = (float)((uint16_t)((uint16_t)gimbal_m6020[0].ENC_angle + (8192 - CHASSIS_FOLLOW_GIMBAL_ANGLE_ZERO) - chassis_control.wz * 0.012) % 8192) / 8192.0f * 360.0f;
		if (chassis_control.chassis_follow_gimbal_angle > 180)
		{
			chassis_control.chassis_follow_gimbal_angle -= 360;
		}

		vx = ramp_control(vx, rc_ctrl.rc.ch[2] * 9, 0.7f);
		vy = ramp_control(vy, rc_ctrl.rc.ch[3] * 9, 0.7f);

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

		chassis_follow_gimbal_zerochange_flag = 1; // ???? ???锟斤拷?????锟斤拷??????????????????
	}
	else if (rc_ctrl.rc.s[1] == RC_SW_UP) // automatic mode NUC?????????
	{
//		if(AutoAim_Data_Receive.uphill_flag == 2)
//		{
//		vx = -ramp_control(vx, (float)AutoAim_Data_Receive.vy * factor[0] * NAV_SPEED_SLOW, 0.3f);
//		vy = ramp_control(vy, (float)AutoAim_Data_Receive.vx * factor[1] * NAV_SPEED_SLOW, 0.3f);
//		}
		vx = -ramp_control(vx, (float)AutoAim_Data_Receive.vy * factor[0] * NAV_SPEED_FAST, 0.9f);
		vy = ramp_control(vy, (float)AutoAim_Data_Receive.vx * factor[1] * NAV_SPEED_FAST, 0.9f);

		if (AutoAim_Data_Receive.rotate == 0) // ??????????
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
		}
		else // 锟斤拷????
		{
			chassis_control.chassis_follow_gimbal_angle = (float)((uint16_t)((uint16_t)gimbal_m6020[0].ENC_angle + (8192 - CHASSIS_FOLLOW_GIMBAL_ANGLE_ZERO) - chassis_control.wz * 0.01) % 8192) / 8192.0f * 360.0f;
			if (chassis_control.chassis_follow_gimbal_angle > 180)
			{
				chassis_control.chassis_follow_gimbal_angle -= 360;
			}
			sin_yaw = arm_sin_f32(-(chassis_control.chassis_follow_gimbal_angle) / 180.0f * 3.14159f);
			cos_yaw = arm_cos_f32(-(chassis_control.chassis_follow_gimbal_angle) / 180.0f * 3.14159f);

			static float rotate_sine_angle = 0;
			static float rotate_sine_T = 1;

			chassis_control.wz = Nav_Chassis_Rotate_Set();
			//			rotate_sine_angle+=(0.001/rotate_sine_T*360);
			//			rotate_sine_angle=Limit_To_180(rotate_sine_angle);

			chassis_follow_gimbal_zerochange_flag = 1;
		}
		chassis_control.vx = cos_yaw * vx + sin_yaw * vy;
		chassis_control.vy = -sin_yaw * vx + cos_yaw * vy;
	}
}

extern fp32 INS_angle[3];
void chassis_feedback_update(void)
{
	Chassis_Data_Tramsit.vy = (-chassis_m3508[0].speed + chassis_m3508[1].speed + chassis_m3508[2].speed - chassis_m3508[3].speed) * 0.70710678f * M3508_MOTOR_RPM_TO_VECTOR / 4.0f;
	Chassis_Data_Tramsit.vx = (-chassis_m3508[0].speed - chassis_m3508[1].speed + chassis_m3508[2].speed + chassis_m3508[3].speed) * 0.70710678f * M3508_MOTOR_RPM_TO_VECTOR / 4.0f;
	Chassis_Data_Tramsit.wz = (-chassis_m3508[0].speed - chassis_m3508[1].speed - chassis_m3508[2].speed - chassis_m3508[3].speed) * M3508_MOTOR_RPM_TO_VECTOR / 4.0f;
}

/**
 * @brief  ???锟斤拷???
 * @param  ???????趨???????
 * @retval ????
 */
float ramp_control(float ref, float set, float accel)
{
	fp32 ramp = limit(accel, 0, 1) * (set - ref);
	return ref + ramp;
}

/**
 * @brief  ?????锟斤拷????
 * @param  ????????,??锟斤拷?,????
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

	 
		CAN_Cap_CMD(Game_Robot_State.chassis_power_limit - 5, 0, Power_Heat_Data.buffer_energy, 0);
		power_control();
//		if (rc_ctrl.rc.s[1] == RC_SW_DOWN || toe_is_error(DBUS_TOE))
		if(rc_ctrl.rc.s[1]==RC_SW_DOWN||toe_is_error(DBUS_TOE)||(Game_Status.game_progress!=4 && rc_ctrl.rc.s[1]==RC_SW_UP))
			CAN_Chassis_CMD(0, 0, 0, 0);
		else
		{
			//				chassis_feedback_update();
			CAN_Chassis_CMD(chassis_m3508[0].give_current, chassis_m3508[1].give_current, chassis_m3508[2].give_current, chassis_m3508[3].give_current);
		}
		vTaskDelay(2);
	}
}
