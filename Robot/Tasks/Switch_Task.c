#include "Switch_Task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "Shoot_Task.h"
#include "remote_control.h"
#include "referee.h"
#include "Nmanifold_usbd_task.h"
#include "SolveTrajectory.h"

#define DIAL_SPEED_LOW  4500//3000
#define DIAL_SPEED_HIGH 5000


RC_ctrl_t rc_ctrl_last;

//uint8_t if_magazine_open=0;//0:off,1:on
extern shoot_motor_t shoot_m2006[2];
extern uint8_t dial_mode,if_single_hit;//mode:0 continue,1 single;
extern int16_t dial_speed;
extern int32_t dial_angle;
extern uint8_t fric_state;
extern ext_game_robot_state_t Game_Robot_State;
extern fifo_s_t Referee_FIFO;
uint8_t autoaim_last_shoot_freq=0;
uint8_t autoaim_shoot_freq=0;
uint8_t dial_mode_last=0;
uint8_t dial_mode_test;

uint32_t shoot_power_good_cnt=0; //摩擦轮缓慢启动
uint8_t if_predict=0;//0:no,1:yes

float auto_aim_pitch_offset=0;

extern float chassis_follow_gimbal_target;
uint32_t switch_180_cnt=501;
uint8_t chassis_follow_gimbal_changing=0;

extern uint8_t autoaim_mode;
extern uint8_t autoaim_armor;

void Switch_Task(void const * argument)
{
	while(1)
	{	
		//180 rotate 180°掉头 好像没法进入这两个if ，switch_180_cnt没有其他程序改变
//		if(switch_180_cnt<500)
//		{
//			switch_180_cnt++;
//		}
//		if(switch_180_cnt==500)
//		{
//			if(chassis_follow_gimbal_target==0)
//				chassis_follow_gimbal_target=180;
//			else if(chassis_follow_gimbal_target==180)
//				chassis_follow_gimbal_target=0;
//			chassis_follow_gimbal_changing=0;
//			switch_180_cnt++;
//		}
		
		//predict
		
//			Game_Robot_State.power_management_shooter_output=0x01; // 先设置上，方便平时调试？
		
//		if(Game_Robot_State.shooter_id1_17mm_cooling_limit>=200)
//		{
//			
//		}
//		if(Game_Robot_State.shooter_id2_17mm_cooling_limit>=200)
//		{

//		}
		
		if(((rc_ctrl.rc.s[0]==RC_SW_DOWN)&&!(rc_ctrl.rc.s[1]==RC_SW_UP))||rc_ctrl.rc.s[1]==RC_SW_DOWN||Game_Robot_State.power_management_shooter_output==0x00)
		{
			shoot_power_good_cnt=0;
			shoot_flag=0;
			
		}		
		//fric 如果没有给发射系统供电，就什么都不转
		if((Game_Robot_State.power_management_shooter_output==0x01) && shoot_power_good_cnt<=20 ) 
			shoot_power_good_cnt++;//delay 2000*2 ms 这里的延时有什么用？
				
			
		if(((rc_ctrl.rc.s[1]==RC_SW_UP)||(rc_ctrl.rc.s[1]==RC_SW_MID))&&(shoot_power_good_cnt>=20)) //摩擦轮缓慢启动完毕
		{
			fric_state=1;
		}
		else 
		{
			fric_state=0;	
		}

		if(shoot_flag==1)//摩擦轮转速达到，可以转波蛋盘
		{
			//dial
			if(dial_mode==0) // 连发
			{
//				if(rc_ctrl.rc.s[0]==RC_SW_UP && rc_ctrl.rc.s[1]==RC_SW_MID) //右开关在上且没有识别到目标：平时测试打弹
//					dial_speed=DIAL_SPEED_HIGH;
//				 if(rc_ctrl.rc.s[0]==RC_SW_UP && rc_ctrl.rc.s[1]==RC_SW_MID && AutoAim_Data_Receive.fire_or_not==1) // 右开关在上且识别到目标：平时测试自瞄
				if(rc_ctrl.rc.s[0]==RC_SW_UP && rc_ctrl.rc.s[1]==RC_SW_MID)
					dial_speed=DIAL_SPEED_HIGH;
				else if(rc_ctrl.rc.s[1]==RC_SW_UP && AutoAim_Data_Receive.track != 0) // 左开关在上且识别到目标：比赛
				{
//					if( tar_position[idx].yaw < 0.7  && tar_position[idx].yaw > -0.6 && fabs(AutoAim_Data_Receive.yaw_aim)<7 ) // 2.5m打前哨
//					{
//						if(AutoAim_Data_Receive.armor_num==3) dial_speed=DIAL_SPEED_LOW;
//						else dial_speed=DIAL_SPEED_HIGH;
//					}
				  if(AutoAim_Data_Receive.fire_or_not==1)
					{
						dial_speed=DIAL_SPEED_HIGH;
					}
					else dial_speed=0;
				}
				else
					dial_speed=0;
			}
			else // 单发模式
			{       
				// 如果有进行发射的状态切换（开关拨弹盘）先让拨弹盘短暂停一下
				if((!(rc_ctrl_last.rc.s[0]==RC_SW_MID)&&(rc_ctrl.rc.s[0]==RC_SW_MID))||(!(rc_ctrl_last.rc.s[0]==RC_SW_UP)&&(rc_ctrl.rc.s[0]==RC_SW_UP)))
				{
					if(if_single_hit==0) // 通过shoot_task的一段代码。控制单发的时间，保证不会变成连发
					{
						if_single_hit=1;
						shoot_m2006[0].angle_set=(shoot_m2006[0].angle+8192*45/10);
						shoot_m2006[1].angle_set=(shoot_m2006[1].angle+8192*45/10);
					}
				}
				// 如果在手动控制发射时，有自瞄信息传入，并且是单发模式，会执行以下代码
//				if(autoaim_last_shoot_freq==0&&AutoAim_Data_Receive.Shoot_Freq!=0&&rc_ctrl.rc.s[0]==RC_SW_UP)//antitop automantic firing
//				if(autoaim_last_shoot_freq==0&&rc_ctrl.rc.s[1]==RC_SW_UP&&(rc_ctrl.rc.s[0]==RC_SW_UP||rc_ctrl.rc.s[0]==RC_SW_MID))
				{
//					autoaim_shoot_freq=AutoAim_Data_Receive.Shoot_Freq;//autoaim shoot frequence update
					if(if_single_hit==0)
					{
						if_single_hit=1;
						shoot_m2006[0].angle_set=(shoot_m2006[0].angle+8192*45/10*(float)(autoaim_shoot_freq/10));//autoaim_shoot_freq/10=firing times per armor plate
						shoot_m2006[1].angle_set=(shoot_m2006[1].angle+8192*45/10*(float)(autoaim_shoot_freq/10));//autoaim_shoot_freq/10=firing times per armor plate
					}
				}
			}
		}		
		
//		if(Autoaim_Mode==0||(Autoaim_Mode==3&&AutoAim_Data_Receive.Aimed_ID<9)) dial_mode=0; // 普通自瞄模式 或 反小陀螺且ID小于9？ 连发模式
//		else if(Autoaim_Mode==1||Autoaim_Mode==2||(Autoaim_Mode==3&&AutoAim_Data_Receive.Aimed_ID>=9)) dial_mode=1; // 打大小能量机关 或 反小陀螺且ID大于9？单发模式
		
		if(Autoaim_Mode==0||(Autoaim_Mode==3)) dial_mode=0; // 普通自瞄模式 或 反小陀螺且ID小于9？ 连发模式
		else if(Autoaim_Mode==1||Autoaim_Mode==2||Autoaim_Mode==3) dial_mode=1; // 打大小能量机关 或 反小陀螺且ID大于9？单发模式
		
		if(dial_mode==1&&dial_mode_last==0)  //切换单连发模式的时候缓一缓
		{
			shoot_m2006[0].angle_set=shoot_m2006[0].angle;
			shoot_m2006[1].angle_set=shoot_m2006[1].angle;
		}

//		autoaim_last_shoot_freq =AutoAim_Data_Receive.Shoot_Freq;
		rc_ctrl_last=rc_ctrl;	
		dial_mode_last=dial_mode;
		vTaskDelay(2);
	}
}
