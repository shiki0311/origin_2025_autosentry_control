#ifndef _CHASSIS_TASK
#define _CHASSIS_TASK

#include "main.h"
#include "struct_typedef.h"
#include "pid.h"


#define CAN_CAP_TX_ID 0x140
#define CAN_CAP_RX_ID 0x130

#define CAN_CHASSIS_ALL_ID 0x200
#define CAN_3508_M1_ID 0x201
#define CAN_3508_M2_ID 0x202
#define CAN_3508_M3_ID 0x203
#define CAN_3508_M4_ID 0x204
#define CHASSIS_CAN hcan1

#define MOTOR_DISTANCE_TO_CENTER 0.231f//245mm
#define CHASSIS_WZ_SET_SCALE 0.0f


typedef struct
{
  fp32 speed;
  fp32 speed_set;
  int16_t give_current;
	
	pid_type_def pid;
}chassis_motor_t;

typedef struct
{
  fp32 vx;
	fp32 vy;
	fp32 wz;
	
  fp32 chassis_follow_gimbal_angle;	 // 底盘跟随云台的相差值
	pid_type_def chassis_follow_gimbal_pid;
}chassis_control_t;
extern double init_chassis_power;
extern chassis_motor_t chassis_m3508[4];
extern chassis_control_t chassis_control;
void Chassis_Task(void const * argument);
void chassis_feedback_update(void);
void CAN_Cap_CMD(float data1,float data2,float data3,float data4);
float Limit_To_180(float in);
fp32 Angle_Z_Suit_ZERO_Get(fp32 Target_Angle,fp32 Target_Speed);
fp32 limit(float data, float min, float max);
float ramp_control(float ref ,float set,float accel);
#endif


