#include "Gimbal_Task.h"
#include "INS_Task.h"
#include "Shoot_Task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "can.h"
#include "remote_control.h"
#include "bsp_can.h"
#include "Chassis_Task.h"
#include "main.h"
#include "arm_math.h"
#include "usart.h"
#include "string.h"
#include "Nmanifold_usbd_task.h"
#include "referee.h"
#include "Vofa_send.h"

#define ANGLE_TO_RAD 0.01745f
#define RAD_TO_ANGLE 57.295779f

#define PITCH_ECD_ANGLE_MAX 27500 // 27800
#define PITCH_ECD_ANGLE_MIN 24850 // 25000

#define YAW_MOTOR_SPEED_PID_KP 600.0f
#define YAW_MOTOR_SPEED_PID_KI 1.1f // 80.0f
#define YAW_MOTOR_SPEED_PID_KD 0.00f
#define YAW_MOTOR_SPEED_PID_MAX_OUT 30000.0f
#define YAW_MOTOR_SPEED_PID_MAX_IOUT 10000.0f

#define YAW_MOTOR_ANGLE_PID_KP 24.80f
#define YAW_MOTOR_ANGLE_PID_KI 0.00013113f
#define YAW_MOTOR_ANGLE_PID_KD 200.3f
#define YAW_MOTOR_ANGLE_PID_MAX_OUT 1200.0f
#define YAW_MOTOR_ANGLE_PID_MAX_IOUT 50.0f

#define YAW_MOTOR_AUTO_AIM_PID_KP 8.0f
#define YAW_MOTOR_AUTO_AIM_PID_KI 0.0001f
#define YAW_MOTOR_AUTO_AIM_PID_KD 50.0f
#define YAW_MOTOR_AUTO_AIM_PID_MAX_OUT 800.0f
#define YAW_MOTOR_AUTO_AIM_PID_MAX_IOUT 0.0f

#define PITCH_MOTOR_SPEED_PID_KP 4.5f
#define PITCH_MOTOR_SPEED_PID_KI 0.0002f
#define PITCH_MOTOR_SPEED_PID_KD 0.0f
#define PITCH_MOTOR_SPEED_PID_MAX_OUT 10.0f
#define PITCH_MOTOR_SPEED_PID_MAX_IOUT 5.0f

#define PITCH_MOTOR_ANGLE_PID_KP 0.05f // 0.2f
#define PITCH_MOTOR_ANGLE_PID_KI 0.001f
#define PITCH_MOTOR_ANGLE_PID_KD 20.0f // 3.0f
#define PITCH_MOTOR_ANGLE_PID_MAX_OUT 4.5f
#define PITCH_MOTOR_ANGLE_PID_MAX_IOUT 1.0f

#define PITCH_MOTOR_AUTO_AIM_PID_KP 0.7f
#define PITCH_MOTOR_AUTO_AIM_PID_KI 0.00000f // 0.0005f
#define PITCH_MOTOR_AUTO_AIM_PID_KD 10.0f
#define PITCH_MOTOR_AUTO_AIM_PID_MAX_OUT 20.0f
#define PITCH_MOTOR_AUTO_AIM_PID_MAX_IOUT 0.0f

gimbal_motor_t gimbal_m6020[2] = {0};

uint8_t auto_aim_tracking = 0;
uint8_t auto_aim_tracking_last = 0;
float yaw_angle_err = 0;
float pitch_angle_err = 0;

static void CAN_Gimbal_CMD(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4) //-30000,+30000
{
	static CAN_TxHeaderTypeDef gimbal_tx_message;
	static uint8_t gimbal_can_send_data[8];
	uint32_t send_mail_box;
	gimbal_tx_message.StdId = CAN_GIMBAL_ALL_ID;
	gimbal_tx_message.IDE = CAN_ID_STD;
	gimbal_tx_message.RTR = CAN_RTR_DATA;
	gimbal_tx_message.DLC = 0x08;
	gimbal_can_send_data[0] = motor1 >> 8;
	gimbal_can_send_data[1] = motor1;
	gimbal_can_send_data[2] = motor2 >> 8;
	gimbal_can_send_data[3] = motor2;
	gimbal_can_send_data[4] = motor3 >> 8;
	gimbal_can_send_data[5] = motor3;
	gimbal_can_send_data[6] = motor4 >> 8;
	gimbal_can_send_data[7] = motor4;

	HAL_StatusTypeDef status;
	status = HAL_CAN_AddTxMessage(&GIMBAL_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
//	if (status != HAL_OK)
//	{
//		// ����ʧ��ʱ���
//		CAN_TxQueue_Push(&gimbal_tx_message, gimbal_can_send_data);
//	}
}

static float angle_error_calc(float target, float current)
{
	float err = target - current;
	if (err > 180)
		err -= 360;
	else if (err < -180)
		err += 360;
	return err;
}
void Gimbal_Motor_Init(void)
{
	const static fp32 yaw_motor_speed_pid[3] = {YAW_MOTOR_SPEED_PID_KP, YAW_MOTOR_SPEED_PID_KI, YAW_MOTOR_SPEED_PID_KD};
	const static fp32 yaw_motor_angle_pid[3] = {YAW_MOTOR_ANGLE_PID_KP, YAW_MOTOR_ANGLE_PID_KI, YAW_MOTOR_ANGLE_PID_KD};
	const static fp32 yaw_motor_auto_aim_pid[3] = {YAW_MOTOR_AUTO_AIM_PID_KP, YAW_MOTOR_AUTO_AIM_PID_KI, YAW_MOTOR_AUTO_AIM_PID_KD};

	const static fp32 pitch_motor_speed_pid[3] = {PITCH_MOTOR_SPEED_PID_KP, PITCH_MOTOR_SPEED_PID_KI, PITCH_MOTOR_SPEED_PID_KD};
	const static fp32 pitch_motor_angle_pid[3] = {PITCH_MOTOR_ANGLE_PID_KP, PITCH_MOTOR_ANGLE_PID_KI, PITCH_MOTOR_ANGLE_PID_KD};
	const static fp32 pitch_motor_auto_aim_pid[3] = {PITCH_MOTOR_AUTO_AIM_PID_KP, PITCH_MOTOR_AUTO_AIM_PID_KI, PITCH_MOTOR_AUTO_AIM_PID_KD};

	PID_init(&gimbal_m6020[0].speed_pid, PID_POSITION, yaw_motor_speed_pid, YAW_MOTOR_SPEED_PID_MAX_OUT, YAW_MOTOR_SPEED_PID_MAX_IOUT);
	PID_init(&gimbal_m6020[0].angle_pid, PID_POSITION, yaw_motor_angle_pid, YAW_MOTOR_ANGLE_PID_MAX_OUT, YAW_MOTOR_ANGLE_PID_MAX_IOUT);
	PID_init(&gimbal_m6020[0].auto_aim_pid, PID_POSITION, yaw_motor_auto_aim_pid, YAW_MOTOR_AUTO_AIM_PID_MAX_OUT, YAW_MOTOR_AUTO_AIM_PID_MAX_IOUT);

	PID_init(&DM_pitch_motor_data.speed_pid, PID_POSITION, pitch_motor_speed_pid, PITCH_MOTOR_SPEED_PID_MAX_OUT, PITCH_MOTOR_SPEED_PID_MAX_IOUT);
	PID_init(&DM_pitch_motor_data.angle_pid, PID_POSITION, pitch_motor_angle_pid, PITCH_MOTOR_ANGLE_PID_MAX_OUT, PITCH_MOTOR_ANGLE_PID_MAX_IOUT);
	PID_init(&DM_pitch_motor_data.auto_aim_pid, PID_POSITION, pitch_motor_auto_aim_pid, PITCH_MOTOR_AUTO_AIM_PID_MAX_OUT, PITCH_MOTOR_AUTO_AIM_PID_MAX_IOUT);
}

void Gimbal_Motor_Data_Update(void)
{
	gimbal_m6020[0].INS_speed_last = gimbal_m6020[0].INS_speed;
	gimbal_m6020[0].INS_speed = bmi088_real_data.gyro[2] * RAD_TO_ANGLE;
	gimbal_m6020[0].INS_angle = INS_angle_deg[0];
	gimbal_m6020[0].ENC_angle = motor_measure_gimbal[0].ecd;
	gimbal_m6020[0].ENC_speed = motor_measure_gimbal[0].speed_rpm;

	DM_pitch_motor_data.INS_speed_last = DM_pitch_motor_data.INS_speed;
	DM_pitch_motor_data.INS_speed = bmi088_real_data.gyro[1];
	DM_pitch_motor_data.INS_angle = INS_angle_deg[2];
}
void Yaw_Motor_Control(void)
{
	static uint8_t yaw_mode = 0, yaw_mode_last = 0; // 0:speed,1:angle
	auto_aim_tracking = AutoAim_Data_Receive.track;

	if (AutoAim_Data_Receive.yaw_aim != 0 || AutoAim_Data_Receive.pitch_aim != 0) // ������
	{
		yaw_angle_err = angle_error_calc(AutoAim_Data_Receive.yaw_aim, gimbal_m6020[0].INS_angle);
		PID_calc(&gimbal_m6020[0].auto_aim_pid, yaw_angle_err, 0);
		gimbal_m6020[0].INS_speed_set = (-gimbal_m6020[0].auto_aim_pid.out) + (gimbal_m6020[0].INS_speed - gimbal_m6020[0].INS_speed_last) * 3.0; // ��0.8��Ŀ��yaw���ٶȡ�ǰ��
		gimbal_m6020[0].INS_angle_set = AutoAim_Data_Receive.yaw_aim;
		yaw_mode = yaw_mode_last = 1;
	}

	// ң�������Կ���
	else if (rc_ctrl.rc.s[1] == RC_SW_MID)
	{
		yaw_mode_last = yaw_mode;
		if (rc_ctrl.rc.ch[0] > 10 || rc_ctrl.rc.ch[0] < -10)
		{
			yaw_mode = 0; // 0�ٶȻ�
		}
		else
		{
			yaw_mode = 1; // 1λ�û�
		}

		if (yaw_mode == 0)
		{
			gimbal_m6020[0].INS_speed_set = -(float)rc_ctrl.rc.ch[0] / 660.0f * 5.0f * RAD_TO_ANGLE;
		}
		else if (yaw_mode == 1 && yaw_mode_last == 0) // yaw�ᶨ����״̬�л��ĽǶ�
		{
			gimbal_m6020[0].INS_angle_set = gimbal_m6020[0].INS_angle;
		}

		if (yaw_mode == 1)
		{
			yaw_angle_err = angle_error_calc(gimbal_m6020[0].INS_angle_set, gimbal_m6020[0].INS_angle);
			PID_calc(&gimbal_m6020[0].angle_pid, yaw_angle_err, 0);
			gimbal_m6020[0].INS_speed_set = -gimbal_m6020[0].angle_pid.out;
		}
	}
	else if (rc_ctrl.rc.s[1] == RC_SW_UP) // ��ʧĿ��yawͣ����
	{
		static uint32_t zero_speed_start_time = 0;
		static uint8_t zero_speed_flag = 0;

		if (auto_aim_tracking == 0 && auto_aim_tracking_last == 1)
		{
			zero_speed_start_time = xTaskGetTickCount();
			zero_speed_flag = 1;
		}
		if (zero_speed_flag && (xTaskGetTickCount() - zero_speed_start_time <= pdMS_TO_TICKS(2000)))
		{
			gimbal_m6020[0].INS_speed_set = 0;
		}
		else
		{
			zero_speed_flag = 0; //
			gimbal_m6020[0].INS_speed_set = Chassis_Data_Receive.yaw_speed * RAD_TO_ANGLE;
		}
	}

	PID_calc(&gimbal_m6020[0].speed_pid, gimbal_m6020[0].INS_speed, gimbal_m6020[0].INS_speed_set);
	gimbal_m6020[0].give_current = gimbal_m6020[0].speed_pid.out;
	auto_aim_tracking_last = auto_aim_tracking;
}

void Pitch_Motor_Control(void)
{
	static uint8_t pitch_mode = 0, pitch_mode_last = 0; // 0:speed,1:angle

	// ������
	if (AutoAim_Data_Receive.yaw_aim != 0 || AutoAim_Data_Receive.pitch_aim != 0)
	{

		pitch_angle_err = (-AutoAim_Data_Receive.pitch_aim) - DM_pitch_motor_data.INS_angle;
		PID_calc(&DM_pitch_motor_data.auto_aim_pid, pitch_angle_err, 0);

		DM_pitch_motor_data.INS_speed_set = (-DM_pitch_motor_data.auto_aim_pid.out) + (DM_pitch_motor_data.INS_speed - DM_pitch_motor_data.INS_speed_last) * 0.2;
		DM_pitch_motor_data.INS_angle_set = AutoAim_Data_Receive.pitch_aim;

		pitch_mode = pitch_mode_last = 1;
	}

	// ң��������
	else if (rc_ctrl.rc.s[1] == RC_SW_MID)
	{
		pitch_mode_last = pitch_mode;
		if ((rc_ctrl.rc.ch[1] > 5 || rc_ctrl.rc.ch[1] < -5))
		{
			pitch_mode = 0;
		}
		else
		{
			pitch_mode = 1;
		}

		if (pitch_mode == 0)
		{
			DM_pitch_motor_data.INS_speed_set = (float)rc_ctrl.rc.ch[1] / 660.0f * 5.0f;
		}
		else if (pitch_mode == 1 && pitch_mode_last == 0)
		{
			DM_pitch_motor_data.INS_angle_set = DM_pitch_motor_data.INS_angle;
		}

		if (pitch_mode == 1)
		{
			PID_calc(&DM_pitch_motor_data.auto_aim_pid, DM_pitch_motor_data.INS_angle, DM_pitch_motor_data.INS_angle_set);
			DM_pitch_motor_data.INS_speed_set = DM_pitch_motor_data.auto_aim_pid.out;
		}
	}
	else if (rc_ctrl.rc.s[1] == RC_SW_UP) // ������pitch����ҡ
	{
		fp32 pitch_up_down_aim = Pitch_Updown();
		PID_calc(&DM_pitch_motor_data.angle_pid, DM_pitch_motor_data.INS_angle, pitch_up_down_aim);
		DM_pitch_motor_data.INS_speed_set = DM_pitch_motor_data.angle_pid.out;
	}

	// DM������λ
	if ((DM_pitch_motor_data.p_int > PITCH_ECD_ANGLE_MAX || DM_pitch_motor_data.p_int < PITCH_ECD_ANGLE_MIN) && (DM_pitch_motor_data.p_int < 50000))
	{
		if (DM_pitch_motor_data.p_int < PITCH_ECD_ANGLE_MIN && DM_pitch_motor_data.INS_speed_set > 0)
		{
			DM_pitch_motor_data.INS_speed_set = 0;
			DM_pitch_motor_data.INS_angle_set = DM_pitch_motor_data.INS_angle;
		}
		if (DM_pitch_motor_data.p_int > PITCH_ECD_ANGLE_MAX && DM_pitch_motor_data.INS_speed_set < 0)
		{
			DM_pitch_motor_data.INS_speed_set = 0;
			DM_pitch_motor_data.INS_angle_set = DM_pitch_motor_data.INS_angle;
		}
	}
	PID_calc(&DM_pitch_motor_data.speed_pid, DM_pitch_motor_data.INS_speed, DM_pitch_motor_data.INS_speed_set);
	DM_pitch_motor_data.target_current = -DM_pitch_motor_data.speed_pid.out;
}

float Pitch_Updown(void)
{
	static float auto_pitch_watch = 0;
	static uint8_t flag = 0;
	AutoAim_Data_Receive.pitch_speed = 1;
	if (AutoAim_Data_Receive.pitch_speed == 0)
	{
		auto_pitch_watch = 0.0f;
	}

	if (flag == 0 && AutoAim_Data_Receive.pitch_speed)
	{
		auto_pitch_watch += 0.08f; //
		if (auto_pitch_watch >= 24.0f)
		{
			flag = 1;
			auto_pitch_watch = 25.0f;
		}
	}
	else if (flag == 1 && AutoAim_Data_Receive.pitch_speed)
	{
		auto_pitch_watch -= 0.08f;
		if (auto_pitch_watch <= 3.0)
		{
			flag = 0;
			auto_pitch_watch = 3.0;
		}
	}
	return auto_pitch_watch;
}

void DM_Auto_Enable()
{
	static uint8_t enable_send_count = 0;
	static uint8_t gimbal_output_last = 0;
	if (Game_Robot_State.power_management_gimbal_output && !gimbal_output_last)
	{
		enable_send_count = 5; // ʹ�����
		HAL_Delay(1000);
	}
	gimbal_output_last = Game_Robot_State.power_management_gimbal_output;

	while (enable_send_count > 0)
	{
		enable_DM(DM4310_ID, 0x01);
		enable_send_count--;
	}
}

void Gimbal_Task(void const *argument)
{
	Gimbal_Motor_Init();
	rc_ctrl.rc.s[1] = RC_SW_DOWN;
	HAL_Delay(1200);
	enable_DM(DM4310_ID, 0x01);

	vTaskDelay(200);

	while (1)
	{
		DM_Auto_Enable();
		Gimbal_Motor_Data_Update();

		//	angle_calculate();

		Yaw_Motor_Control();
		Pitch_Motor_Control();

		if (rc_ctrl.rc.s[1] == RC_SW_DOWN)
		{
			CAN_Gimbal_CMD(0, 0, 0, 0);
			ctrl_motor(DM4310_ID, 0, 0, 0, 0, 0);
			CAN_Shoot_CMD(0, 0, shoot_motor_3508[0].target_current, shoot_motor_3508[1].target_current);
		}
		else
		{
			CAN_Gimbal_CMD(gimbal_m6020[0].give_current, 0, 0, 0);
			ctrl_motor(DM4310_ID, 0, 0, 0, 0, DM_pitch_motor_data.target_current);
			CAN_Shoot_CMD(0, shoot_m2006[0].target_current, shoot_motor_3508[0].target_current, shoot_motor_3508[1].target_current);
		}

		//		Vofa_Send_Data4((float)dial_stop_cnt,motor_measure_shoot[2].given_current,shoot_m2006[0].target_current,0);
		//	Vofa_Send_Data4(AutoAim_Data_Receive.yaw_aim,gimbal_m6020[0].INS_angle,(float)AutoAim_Data_Receive.fire_or_not,0);
		//		Vofa_Send_Data4((float)DM_pitch_motor_data.INS_angle_set,(float)DM_pitch_motor_data.INS_angle,DM_pitch_motor_data.INS_speed,DM_pitch_motor_data.INS_speed_set);
		vTaskDelay(1);
	}
}

/*********************************************************DM*********************************************************/
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 50.0f
#define POS_MIN -12.5f
#define POS_MAX 12.5f
#define SPD_MIN -18.0f
#define SPD_MAX 18.0f
#define T_MIN -30.0f
#define T_MAX 30.0f
#define I_MIN -30.0f
#define I_MAX 30.0f

void ctrl_motor(uint16_t id, float _pos, float _vel, float _KP, float _KD, float _torq) // can1
{
	uint8_t TX_Data[8];
	uint32_t send_mail_box;
	CAN_TxHeaderTypeDef Tx_Msg;

	// uint8_t *pbuf,*vbuf;
	Tx_Msg.StdId = id;		   // set chassis motor current.
	Tx_Msg.IDE = CAN_ID_STD;   // ��ʹ����չ��ʶ��
	Tx_Msg.RTR = CAN_RTR_DATA; // ��Ϣ����Ϊ����֡��һ֡8λ
	Tx_Msg.DLC = 8;

	uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
	pos_tmp = float_to_uint(_pos, -12.5, 12.5, 16);
	vel_tmp = float_to_uint(_vel, -45, 45, 12);
	kp_tmp = float_to_uint(_KP, KP_MIN, KP_MAX, 12);
	kd_tmp = float_to_uint(_KD, KD_MIN, KD_MAX, 12);
	tor_tmp = float_to_uint(_torq, T_MIN, T_MAX, 12);

	TX_Data[0] = (pos_tmp >> 8);
	TX_Data[1] = pos_tmp;
	TX_Data[2] = (vel_tmp >> 4);
	TX_Data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
	TX_Data[4] = kp_tmp;
	TX_Data[5] = (kd_tmp >> 4);
	TX_Data[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
	TX_Data[7] = tor_tmp;

	HAL_CAN_AddTxMessage(&hcan2, &Tx_Msg, TX_Data, &send_mail_box);
}

void enable_DM(uint8_t id, uint8_t ctrl_mode)
{
	uint8_t TX_Data[8];
	uint32_t send_mail_box;
	CAN_TxHeaderTypeDef Tx_Msg;

	if (ctrl_mode == 1)
		Tx_Msg.StdId = 0x000 + DM4310_ID; // set chassis motor current.
	else if (ctrl_mode == 2)
		Tx_Msg.StdId = 0x100 + DM4310_ID; // set chassis motor current.
	else if (ctrl_mode == 3)
		Tx_Msg.StdId = 0x200 + DM4310_ID; // set chassis motor current.
	Tx_Msg.IDE = CAN_ID_STD;
	Tx_Msg.RTR = CAN_RTR_DATA;
	Tx_Msg.DLC = 8;

	TX_Data[0] = 0xff;
	TX_Data[1] = 0xff;
	TX_Data[2] = 0xff;
	TX_Data[3] = 0xff;
	TX_Data[4] = 0xff;
	TX_Data[5] = 0xff;
	TX_Data[6] = 0xff;
	TX_Data[7] = 0xfc;

	HAL_CAN_AddTxMessage(&hcan2, &Tx_Msg, TX_Data, &send_mail_box);
}

// ����ʧ��֡
void disable_DM(uint8_t id, uint8_t ctrl_mode)
{
	uint8_t TX_Data[8];
	uint32_t send_mail_box;
	CAN_TxHeaderTypeDef Tx_Msg;

	if (ctrl_mode == 1)
		Tx_Msg.StdId = 0x000 + DM4310_ID; // set chassis motor current.
	else if (ctrl_mode == 2)
		Tx_Msg.StdId = 0x100 + DM4310_ID; // set chassis motor current.
	else if (ctrl_mode == 3)
		Tx_Msg.StdId = 0x200 + DM4310_ID; // set chassis motor current.

	Tx_Msg.IDE = CAN_ID_STD;
	Tx_Msg.RTR = CAN_RTR_DATA;
	Tx_Msg.DLC = 8;

	TX_Data[0] = 0xff;
	TX_Data[1] = 0xff;
	TX_Data[2] = 0xff;
	TX_Data[3] = 0xff;
	TX_Data[4] = 0xff;
	TX_Data[5] = 0xff;
	TX_Data[6] = 0xff;
	TX_Data[7] = 0xfd;

	HAL_CAN_AddTxMessage(&hcan2, &Tx_Msg, TX_Data, &send_mail_box);
}
