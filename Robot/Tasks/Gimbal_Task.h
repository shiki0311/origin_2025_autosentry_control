#ifndef _GIMBAL_TASK
#define _GIMBAL_TASK

#include "struct_typedef.h"
#include "pid.h"

typedef struct
{
	fp32 INS_speed;
	fp32 INS_speed_last;
	fp32 INS_speed_set;
	fp32 INS_speed_set_last;
	fp32 INS_angle;
	fp32 INS_angle_set;
	fp32 ENC_angle;
	fp32 ENC_speed;
	fp32 ENC_angle_actual;
	fp32 ENC_angle_set;
	int16_t give_current;

	pid_type_def speed_pid;
	pid_type_def angle_pid;
	pid_type_def auto_aim_pid;
} gimbal_motor_t; // 云台yaw轴6020结构体

/*************************云台模式枚举体****************************/
typedef enum
{
	AUTOAIM,			   // 自瞄模式
	GIMBAL_REMOTE_CONTROL, // 遥控器控制模式
	NAV,				   // 导航自主巡逻模式
	GIMBAL_SAFE			   // 失能模式
} gimbal_mode_t;

/**************************在gimbal_task.c中的全局变量及常量区定义底盘模式表，在不同的底盘模式下设置对应底盘控制逻辑***************************/
typedef struct
{
	gimbal_mode_t mode;	   // 底盘模式
	void (*handler)(void); // 不同云台模式对应的处理函数
} gimbal_command_t;

/*************************云台电机被控变量枚举体****************************/
typedef enum
{
	POSITION,
	SPEED
} gimbal_motor_control_mode_t; // 用于判断在当前云台模式下电机的控制模式，控速度还是位置

/*******************************云台电机类型结构体************************************/
typedef enum
{
	PITCH_MOTOR,
	YAW_MOTOR,
} gimbal_motor_id_t;

/*******************************pitch导航模式下自主巡航模式************************************/
typedef enum
{
	HIT_ROBOT,	// 击打机器人
	HIT_OUTPOST // 击打前哨站
} pitch_updown_mode_t;

/*******************************pitch导航模式下自主巡航参数结构体************************************/
typedef struct
{
	float min_angle; // 当前pitch巡航模式下的最小角度
	float max_angle; // 当前pitch巡航模式下的最大角度
	float step;		 // 当前pitch巡航模式下每次的步进角度
} PitchSwingParams;

extern gimbal_motor_t gimbal_m6020[2];

void Gimbal_Task(void const *argument);
#endif
