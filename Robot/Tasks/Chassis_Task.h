#ifndef _CHASSIS_TASK
#define _CHASSIS_TASK

#include "main.h"
#include "struct_typedef.h"
#include "pid.h"

typedef struct // 底盘全向轮电机结构体
{
  fp32 speed;
  fp32 speed_set;
  int16_t give_current;

  pid_type_def pid;
} chassis_motor_t;

typedef struct // 底盘控制参数结构体
{
  fp32 vx;
  fp32 vy;
  fp32 wz;

  fp32 chassis_follow_gimbal_angle;
  fp32 chassis_power_limit;
  fp32 init_chassis_power;
  bool_t chassis_follow_gimbal_zerochange;

  pid_type_def chassis_follow_gimbal_pid;
} chassis_control_t;

extern chassis_motor_t chassis_m3508[4];
extern chassis_control_t chassis_control;
extern float Real_Power;

void Chassis_Task(void const *argument);

#endif
