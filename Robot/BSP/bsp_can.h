#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "struct_typedef.h"
#include "pid.h"
#include "can.h"

#define TX_QUEUE_SIZE 64

#pragma pack(push, 1)
typedef struct
{
	uint16_t ecd;
	int16_t speed_rpm;
	int16_t given_current;
	uint8_t temperate;
	int16_t last_ecd;
	fp32 code;

} motor_measure_t;

typedef struct
{
	int id;
	int state;
	int p_int;
	int v_int;
	int t_int;
	int kp_int;
	int kd_int;
	float pos;
	float vel;
	float toq;
	float Kp;
	float Kd;
	float Tmos;
	float Tcoil;

	uint16_t target_pos;
	int16_t target_vel;
	int16_t target_current;
	fp32 INS_speed;
	fp32 INS_speed_set;
	fp32 INS_speed_last;
	fp32 INS_angle;
	fp32 INS_angle_set;

	pid_type_def speed_pid;
	pid_type_def angle_pid;
	pid_type_def auto_aim_pid;

} DM_motor_data_t;

#pragma pack(pop) //这行位置别改，pid.h里面我写注释的两行也别改

/**
 * can��Ϣ�ṹ��
 */
typedef struct
{
	uint8_t data[8];
	CAN_TxHeaderTypeDef tx_header;
} CanTxMsgTypeDef;

/* can��Ϣ���нṹ�� */
typedef struct
{
	CanTxMsgTypeDef can_msg_buffer[TX_QUEUE_SIZE];
	uint8_t head;
	uint8_t tail;
	uint8_t element_number;
} CanTxQueueTypeDef;


extern void can_filter_init(void);

extern motor_measure_t motor_measure_chassis[4];
extern motor_measure_t motor_measure_gimbal[2];
extern motor_measure_t motor_measure_shoot[3];
extern DM_motor_data_t DM_pitch_motor_data;

float uint_to_float(int x_int, float x_min, float x_max, int bits);
int float_to_uint(float x, float x_min, float x_max, int bits);
void CAN_TxQueue_Push(CAN_TxHeaderTypeDef *pHeader, uint8_t *pData);
void CAN_TxQueue_Init();

#endif
