#ifndef _GIMBAL_TASK
#define _GIMBAL_TASK

#include "struct_typedef.h"
#include "pid.h"

#define CAN_GIMBAL_ALL_ID 0x1FF
#define CAN_6020_M1_ID 0x205
//#define CAN_6020_M2_ID 0x207
#define DM4310_ID 0x01

#define GIMBAL_CAN hcan1
#define Get_BYTE0(dwTemp)       (*(char *)(&dwTemp))	
#define Get_BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))	 
#define Get_BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define Get_BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))

typedef struct
{
  fp32 INS_speed;
  fp32 INS_speed_set;
	fp32 INS_speed_last;
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
} gimbal_motor_t;


extern gimbal_motor_t gimbal_m6020[2];
extern float auto_aim_yaw_exp,auto_aim_pitch_exp;
extern float pitch_compensation;

void Gimbal_Task(void const * argument);
void ctrl_motor(uint16_t id, float _pos, float _vel,float _KP, float _KD, float _torq);
void enable_DM(uint8_t id, uint8_t ctrl_mode);
void disable_DM(uint8_t id, uint8_t ctrl_mode);
void Pitch_Updown(void);

#endif
