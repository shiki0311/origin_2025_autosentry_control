#include "Shoot_Task.h"
#include "Gimbal_Task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "remote_control.h"
#include "bsp_can.h"
#include "bsp_tim.h"
#include "usart.h"
#include <stdio.h>
#include <stdlib.h>
#include "referee.h"
#include "math.h"

#define SHOOT_MOTOR_SPEED_PID_KP 5.0f
#define SHOOT_MOTOR_SPEED_PID_KI 0.05f
#define SHOOT_MOTOR_SPEED_PID_KD 0.0f
#define SHOOT_MOTOR_SPEED_PID_MAX_OUT 28000.0f
#define SHOOT_MOTOR_SPEED_PID_MAX_IOUT 10000.0f

#define SHOOT_MOTOR_ANGLE_PID_KP 0.3f
#define SHOOT_MOTOR_ANGLE_PID_KI 0.0f
#define SHOOT_MOTOR_ANGLE_PID_KD 0.2f
#define SHOOT_MOTOR_ANGLE_PID_MAX_OUT 10000.0f
#define SHOOT_MOTOR_ANGLE_PID_MAX_IOUT 10000.0f

extern CAN_HandleTypeDef hcan1;

shoot_motor_t shoot_m2006[2];
uint8_t fric_state=0;//0:off,1:on
uint8_t dial_mode=0,if_single_hit=0;//mode:0 continue,1 single
int16_t dial_speed=0;
extern int32_t dial_angle;
float dial_err=0;
uint16_t fric_speed=1005;
extern fifo_s_t Referee_FIFO;
uint16_t shoot_flag=0,shoot_cnt=0,safe_flag=0;

void Shoot_Motor_Init(void)
{
	const static fp32 shoot_motor_speed_pid[3] = {SHOOT_MOTOR_SPEED_PID_KP, SHOOT_MOTOR_SPEED_PID_KI, SHOOT_MOTOR_SPEED_PID_KD};
	const static fp32 shoot_motor_angle_pid[3] = {SHOOT_MOTOR_ANGLE_PID_KP, SHOOT_MOTOR_ANGLE_PID_KI, SHOOT_MOTOR_ANGLE_PID_KD};
	
	shoot_m2006[0].speed=0;
	shoot_m2006[0].speed_set=0;
	shoot_m2006[0].angle=0;
	shoot_m2006[0].angle_set=0;
	shoot_m2006[0].ENC_angle=0;
	shoot_m2006[0].give_current=0;
	
	PID_init(&shoot_m2006[0].speed_pid,PID_POSITION,shoot_motor_speed_pid,SHOOT_MOTOR_SPEED_PID_MAX_OUT,SHOOT_MOTOR_SPEED_PID_MAX_IOUT);
	PID_init(&shoot_m2006[0].angle_pid,PID_POSITION,shoot_motor_angle_pid,SHOOT_MOTOR_ANGLE_PID_MAX_OUT,SHOOT_MOTOR_ANGLE_PID_MAX_IOUT);
	
	shoot_m2006[1].speed=0;
	shoot_m2006[1].speed_set=0;
	shoot_m2006[1].angle=0;
	shoot_m2006[1].angle_set=0;
	shoot_m2006[1].ENC_angle=0;
	shoot_m2006[1].give_current=0;
	
	PID_init(&shoot_m2006[1].speed_pid,PID_POSITION,shoot_motor_speed_pid,SHOOT_MOTOR_SPEED_PID_MAX_OUT,SHOOT_MOTOR_SPEED_PID_MAX_IOUT);
	PID_init(&shoot_m2006[1].angle_pid,PID_POSITION,shoot_motor_angle_pid,SHOOT_MOTOR_ANGLE_PID_MAX_OUT,SHOOT_MOTOR_ANGLE_PID_MAX_IOUT);
	
}

CAN_TxHeaderTypeDef  shoot_tx_message;
uint8_t              shoot_can_send_data[8];

static void CAN_Shoot_CMD(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)//-30000,+30000
{
	uint32_t send_mail_box;
	shoot_tx_message.StdId = CAN_SHOOT_ALL_ID;
	shoot_tx_message.IDE = CAN_ID_STD;
	shoot_tx_message.RTR = CAN_RTR_DATA;
	shoot_tx_message.DLC = 0x08;
	shoot_can_send_data[0] = motor1 >> 8;
	shoot_can_send_data[1] = motor1;
	shoot_can_send_data[2] = motor2 >> 8;
	shoot_can_send_data[3] = motor2;
	shoot_can_send_data[4] = motor3 >> 8;
	shoot_can_send_data[5] = motor3;
	shoot_can_send_data[6] = motor4 >> 8;
	shoot_can_send_data[7] = motor4;

	HAL_CAN_AddTxMessage(&SHOOT_CAN, &shoot_tx_message, shoot_can_send_data, &send_mail_box);
}


void Shoot_Motor_Data_Update(void)//只用到了shoot_m2006[0]
{
	shoot_m2006[0].speed=motor_measure_shoot[0].speed_rpm;
	shoot_m2006[0].angle=dial_angle;
	
//	shoot_m2006[1].speed=motor_measure_shoot[1].speed_rpm;
//	shoot_m2006[1].angle=dial_angle;
}

//extern ext_shoot_data_t shoot_data_t;
//float bullet_speed_max=0,bullet_speed_min=30;

void Fric_Motor_Control(void)
{
		fric_speed=fric_18m;
		
    Fric_PWR(fric_state);
}

uint16_t dial_single_cnt=0;
uint32_t dial_stop_cnt=0;

void Dial_Motor_Control(void)
{
	if(Game_Robot_State.power_management_shooter_output==0x01)
	{
		if(abs(shoot_m2006[0].give_current)>18000 && fabs(shoot_m2006[0].speed)<100) //堵转自检 反转并重启
		{ 
			dial_stop_cnt++;
			if(dial_stop_cnt>250)
			{
				if(shoot_m2006[0].give_current<0)
					shoot_m2006[0].give_current=5000;
				else
					shoot_m2006[0].give_current=-5000;
				vTaskDelay(500);
				Shoot_Motor_Init();
				PID_clear(&shoot_m2006[0].speed_pid);
				PID_clear(&shoot_m2006[0].angle_pid);
				dial_stop_cnt=0;
				shoot_m2006[0].angle_set=dial_angle;
			}
		}

	}
	
	if(shoot_flag==1)  // 摩擦轮已开 拨弹盘可以转
	{
		if(dial_mode==0)//continue 连发模式
		{
			shoot_m2006[0].speed_set=dial_speed; // 在switch_task中赋值
		}
		else//single 单发模式
		{	
			if(if_single_hit==1) // 一小段时间后将if_single_hit 从 1 变 0
			{
				dial_single_cnt++;
				if(dial_single_cnt>50)
				{
					if_single_hit=0;
					dial_single_cnt=0;
				}
			}
			
			PID_calc(&shoot_m2006[0].angle_pid,shoot_m2006[0].angle,shoot_m2006[0].angle_set);
			shoot_m2006[0].speed_set=shoot_m2006[0].angle_pid.out;
		}
	}
	else // 摩擦轮没开
	{
		shoot_m2006[0].speed_set=0;
	}
	PID_calc(&shoot_m2006[0].speed_pid,shoot_m2006[0].speed,shoot_m2006[0].speed_set);
//	PID_calc(&shoot_m2006[1].speed_pid,shoot_m2006[1].speed,shoot_m2006[1].speed_set);
	
	shoot_m2006[0].give_current=shoot_m2006[0].speed_pid.out;
//	shoot_m2006[1].give_current=shoot_m2006[1].speed_pid.out;
}
bool_t flag_cooling_limit[2]={0};
uint16_t cooling_limit_cnt[2]={0};

void Shoot_Power_Control() // 根据裁判系统传回的数据判定是否发射
{
	if(Power_Heat_Data.shooter_17mm_1_barrel_heat>=(Game_Robot_State.shooter_barrel_heat_limit-30)) // 枪管1冷却值上限 
	{
		flag_cooling_limit[0]=1;
	}
	if(Power_Heat_Data.shooter_17mm_2_barrel_heat>=(Game_Robot_State.shooter_barrel_heat_limit-30))
	{
		flag_cooling_limit[1]=1;
	}
	if(flag_cooling_limit[0]!=0 || flag_cooling_limit[1]!=0) //拨弹盘1停转 等半秒     /////有裁判系统记得注释回来
	{
		
			shoot_m2006[0].angle_set=shoot_m2006[0].angle;
			shoot_m2006[0].speed_set=0;
			shoot_m2006[0].give_current=0;
		
			PID_clear(&shoot_m2006[0].speed_pid);
			PID_clear(&shoot_m2006[0].angle_pid);
		
			cooling_limit_cnt[0]++;
		
		if(cooling_limit_cnt[0]>=250)//500ms 等半秒再打
		{
			cooling_limit_cnt[0]=0;
			flag_cooling_limit[0]=0;
			flag_cooling_limit[1]=0;
		}
	}
//	if(flag_cooling_limit[1]!=0) // 拨弹盘2停转 等半秒
//	{
//		
//			shoot_m2006[1].angle_set=shoot_m2006[1].angle;
//			shoot_m2006[1].speed_set=0;
//			shoot_m2006[1].give_current=0;
//		
//			PID_clear(&shoot_m2006[1].speed_pid);
//			PID_clear(&shoot_m2006[1].angle_pid);
//		
//			cooling_limit_cnt[1]++;
//		
//		if(cooling_limit_cnt[1]>=250)//500ms
//		{
//			cooling_limit_cnt[1]=0;
//			flag_cooling_limit[1]=0;
//		}
//	}
}

void Shoot_Task(void const * argument)
{
	Shoot_Motor_Init();
	vTaskDelay(200);
	
	while(1)
	{		
		Shoot_Motor_Data_Update();

		Fric_Motor_Control();  // 摩擦轮默认开机自启
		Dial_Motor_Control();
		Shoot_Power_Control();
	
   //CAN_Shoot_CMD(0,shoot_m2006[0].give_current,0,0);
		vTaskDelay(2);
	}
}

//摩擦轮缓启动
void Fric_PWR(uint8_t power){
	if(power){
		if(TIM1->CCR1 < fric_speed){       //摩擦轮PWM值未达到
			TIM1->CCR1 += 3, TIM1->CCR2 += 3;
      TIM1->CCR3 += 3, TIM1->CCR4 += 3;		//PWM值持续增大，摩擦轮功率逐渐增大
		}
			shoot_flag=1;
	}
	else{
		if(TIM1->CCR1 >= 1000){					//保证摩擦轮转速在还没准备好启动摩擦轮时为0 
			TIM1->CCR1 -= 4,TIM1->CCR2 -= 4;
      TIM1->CCR3 -= 4,TIM1->CCR4 -= 4;			//PWM值持续减小，摩擦轮功率逐渐减小
		}
	}
}     