#ifndef _SHOOT_TASK
#define _SHOOT_TASK

#include "main.h"
#include "Gimbal_Task.h"

#define CAN_SHOOT_ALL_ID 0x1FF
#define CAN_2006_M1_ID 0x206 
//#define CAN_2006_M2_ID 0x202
#define SHOOT_CAN hcan1
#define fric_15m 1500 //13.7

#define fric_18m 1670//22
#define fric_30m 1730//1690

typedef struct
{
  fp32 speed;
  fp32 speed_set;
	fp32 angle;
  fp32 angle_set;
	fp32 ENC_angle;
  int16_t give_current;
	
	pid_type_def speed_pid;
	pid_type_def angle_pid;
} shoot_motor_t;

typedef struct
{
  fp32 speed;
  fp32 speed_set;
	int16_t give_pwm;
	
	pid_type_def pid;
} fric_motor_t;

extern shoot_motor_t shoot_m2006[2];
extern uint8_t fric_state;
extern uint8_t dial_mode;//mode:0 continue,1 single;
extern uint16_t shoot_flag,shoot_cnt,safe_flag;
extern uint16_t fric_speed;
void Fric_PWR(uint8_t power);
void Shoot_Task(void const * argument);

#endif
