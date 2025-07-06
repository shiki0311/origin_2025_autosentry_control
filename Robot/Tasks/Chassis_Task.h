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

#define MOTOR_DISTANCE_TO_CENTER 0.231f // 245mm

/*************************中弹掉血切换小陀螺模式（2025赛季节省功率特供版）****************************/
typedef enum
{
  HEALTH_NORMAL, // 正常模式
  HEALTH_HURT    // 受伤旋转模式
} health_state_t;
/*************************底盘功率上限枚举体，不同的值对应不同的底盘功率上限****************************/

typedef enum
{
  REMOTE_CONTROL,
  NAV_NORMAL_MODE,
  HURT,
  UPHILL_START,
  ON_HILL
} chassis_max_power_control_t;

/*************************底盘模式枚举体****************************/
typedef enum
{
  FOLLOW_GIMBAL, // 底盘跟随云台移动模式
  ROTATE,        // 小陀螺移动模式
  SAFE           // 失能模式
} chassis_mode_t;

/**************************在task源码文件中定义底盘模式表，便于在不同的底盘模式下设置对应底盘控制逻辑***************************/
typedef void (*ChassisModeHandler)(void);
typedef struct
{
  chassis_mode_t mode;        // 底盘模式
  ChassisModeHandler handler; // 不同底盘模式对应的处理函数
} chassis_command_t;

typedef struct
{
  fp32 speed;
  fp32 speed_set;
  int16_t give_current;

  pid_type_def pid;
} chassis_motor_t;

typedef struct
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
typedef struct
{
  float vx; // (m/s)
  float vy; // (m/s)
  float wz; // (rad/s)

} chassis_real_speed_t;

extern chassis_motor_t chassis_m3508[4];
extern chassis_control_t chassis_control;
extern float Real_Power;
void Chassis_Task(void const *argument);
void chassis_feedback_update(void);
void chassis_follow_gimbal_handler(void);
void chassis_rotate_handler(void);
void chassis_safe_handler(void);
fp32 Angle_Z_Suit_ZERO_Get(fp32 Target_Angle);

#endif
