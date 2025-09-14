/*****************************************************************************************************************************
 * @file: bsp_can.h
 * @author: Shiki
 * @date: 2025.7.12
 * @brief:	哨兵2025赛季CAN通讯支持包
 * 此文件定义了大疆电机can接收的数据结构体和达妙电机结构体，并对外提供机器人各个机构的电机结构体实例。
 *****************************************************************************************************************************/

#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "struct_typedef.h"
#include "pid.h"
#include "can.h"

#define GIMBAL_PITCH_DM_SendID 0x01

typedef enum
{
	CAN_CHASSIS_CMD,
	CAN_GIMBAL_YAW_CMD,
	CAN_GIMBAL_PITCH_CMD,
	CAN_SHOOT_CMD,
	CAN_CAP_CMD,
} CAN_CMD_ID; // CAN发送命令类型,用于把不同的can消息送入对应消息队列统一发送

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
	volatile float target_current;
	fp32 INS_speed;
	fp32 INS_speed_set;
	fp32 INS_speed_last;
	fp32 INS_angle;
	fp32 INS_angle_set;

	pid_type_def speed_pid;
	pid_type_def angle_pid;
	pid_type_def auto_aim_pid;

} DM_motor_data_t;

#pragma pack(pop) // 这行位置别改，pid.h里面我写注释的两行也别改，不信你改了试试

typedef struct
{
	CAN_TxHeaderTypeDef tx_header;
	uint8_t data[8];
} CanTxMsgTypeDef;   // CAN报文结构体，用于can发送队列中


extern motor_measure_t motor_measure_chassis[4];
extern motor_measure_t motor_measure_gimbal[2];
extern motor_measure_t motor_measure_shoot[3];
extern DM_motor_data_t DM_pitch_motor_data;
extern int32_t dial_angle;

void Can_Filter_Init(void);
void Can_Buffer_Init(void);
void Create_Can_Send_Queues(void);
void Allocate_Can_Buffer(int16_t data1, int16_t data2, int16_t data3, int16_t data4, CAN_CMD_ID can_cmd_id); 
void CAN_TX_TimerIRQHandler(void);
void CAN_Resend_Timer_IRQHandler(void);
void Ctrl_DM_Motor(uint16_t id, float _pos, float _vel, float _KP, float _KD, float _torq);
void enable_DM(uint8_t id, uint8_t ctrl_mode);
void disable_DM(uint8_t id, uint8_t ctrl_mode);

void CAN_Cap_CMD(float data1, float data2, float data3, float data4);
void CAN_Chassis_CMD(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
void CAN_Gimbal_CMD(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
void CAN_Shoot_CMD(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

#endif
