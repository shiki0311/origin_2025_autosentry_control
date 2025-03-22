#include "Gimbal_Task.h"
#include "INS_Task.h"
#include "Shoot_Task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "can.h"
#include "remote_control.h"
#include "bsp_can.h"
#include "Chassis_Task.h"
#include "main.h"
#include "arm_math.h"
#include "usart.h"
#include "string.h"
#include "Nmanifold_usbd_task.h"
#include "referee.h"
#include "Vofa_send.h"

#define YAW_MOUSE_SEN   0.015f
#define PITCH_MOUSE_SEN 0.012f
#define ANGLE_TO_RAD    0.01745f
#define RAD_TO_ANGLE    57.295779f

#define PITCH_ECD_ANGLE_MAX 27178  // 27800
#define PITCH_ECD_ANGLE_MIN 24850// 25000

#define YAW_MOTOR_SPEED_PID_KP 600.0f
#define YAW_MOTOR_SPEED_PID_KI 1.1f//80.0f
#define YAW_MOTOR_SPEED_PID_KD 0.00f
#define YAW_MOTOR_SPEED_PID_MAX_OUT 30000.0f
#define YAW_MOTOR_SPEED_PID_MAX_IOUT 10000.0f

#define YAW_MOTOR_ANGLE_PID_KP 24.80f
#define YAW_MOTOR_ANGLE_PID_KI 0.013113f
#define YAW_MOTOR_ANGLE_PID_KD 100.3f
#define YAW_MOTOR_ANGLE_PID_MAX_OUT 1200.0f
#define YAW_MOTOR_ANGLE_PID_MAX_IOUT 100.0f

float yaw_pid_rate=0.3f;//0.3f

#define YAW_MOTOR_AUTO_AIM_PID_KP 20.0f
#define YAW_MOTOR_AUTO_AIM_PID_KI 0.0003f
#define YAW_MOTOR_AUTO_AIM_PID_KD 100.0f
#define YAW_MOTOR_AUTO_AIM_PID_MAX_OUT 80.0f
#define YAW_MOTOR_AUTO_AIM_PID_MAX_IOUT 30.0f

#define PITCH_MOTOR_SPEED_PID_KP 2.5f
#define PITCH_MOTOR_SPEED_PID_KI 0.003f
#define PITCH_MOTOR_SPEED_PID_KD 0.0f
#define PITCH_MOTOR_SPEED_PID_MAX_OUT 20.0f
#define PITCH_MOTOR_SPEED_PID_MAX_IOUT 5.0f

#define PITCH_MOTOR_ANGLE_PID_KP 0.2f//0.2f
#define PITCH_MOTOR_ANGLE_PID_KI 0.0f
#define PITCH_MOTOR_ANGLE_PID_KD 30.0f//3.0f
#define PITCH_MOTOR_ANGLE_PID_MAX_OUT 4.5f
#define PITCH_MOTOR_ANGLE_PID_MAX_IOUT 0.05f

float pitch_pid_rate=0.25f;
#define PITCH_MOTOR_AUTO_AIM_PID_KP 1.8f
#define PITCH_MOTOR_AUTO_AIM_PID_KI 0.0008f//0.0005f
#define PITCH_MOTOR_AUTO_AIM_PID_KD 80.0f
#define PITCH_MOTOR_AUTO_AIM_PID_MAX_OUT 2.5f
#define PITCH_MOTOR_AUTO_AIM_PID_MAX_IOUT 100.0f

CAN_TxHeaderTypeDef  gimbal_tx_message;

gimbal_motor_t gimbal_m6020[2];

uint8_t yaw_mode=0,yaw_mode_last=0;//0:speed,1:angle
uint8_t pitch_mode=0,pitch_mode_last=0;//0:speed,1:angle

uint8_t identify_flag=1;
uint16_t flag_cnt[2]={0};
float yaw_angle_err=0,pitch_angle_err=0;

float auto_aim_yaw_exp=0,auto_aim_pitch_exp=0;
int auto_aim_vx=0,auto_aim_vz=0;
uint8_t auto_aim_flag=0,auto_aim_change_cnt=0,auto_aim_id_last=0,auto_aim_id=0;
uint8_t auto_aim_tracking=0;
float auto_pitch_watch=0;
float pitch_compensation=0;
uint8_t gimbal_output_last = 0;
uint8_t enable_send_count = 0;

extern DM_motor_data_t DM_pitch_motor_data;

extern ext_game_robot_state_t Game_Robot_State;
extern fifo_s_t Referee_FIFO;
extern uint8_t chassis_follow_gimbal_changing;

extern Chassis_Gimbal_Angle_TX Chassis_Gimbal_Angle_Tramsit;

extern Chassis_Data_Rx Chassis_Data_Receive;

extern void Vofa_Send_Data4(float data1, float data2,float data3, float data4);
static void CAN_Gimbal_CMD(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)//-30000,+30000
{
	uint32_t send_mail_box;
	uint8_t  gimbal_can_send_data[8];
	gimbal_tx_message.StdId = CAN_GIMBAL_ALL_ID;
	gimbal_tx_message.IDE = CAN_ID_STD;
	gimbal_tx_message.RTR = CAN_RTR_DATA;
	gimbal_tx_message.DLC = 0x08;
	gimbal_can_send_data[0] = motor1 >> 8;
	gimbal_can_send_data[1] = motor1;
	gimbal_can_send_data[2] = motor2 >> 8;
	gimbal_can_send_data[3] = motor2;
	gimbal_can_send_data[4] = motor3 >> 8;
	gimbal_can_send_data[5] = motor3;
	gimbal_can_send_data[6] = motor4 >> 8;
	gimbal_can_send_data[7] = motor4;

	HAL_CAN_AddTxMessage(&GIMBAL_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}

//void angle_calculate(void)
//{
//	if(fabs(AutoAim_Data_Receive.yaw_aim)>=7) autoSolveTrajectory_highspeed();
//	else autoSolveTrajectory();

//	if(AutoAim_Data_Receive.armor_num == 3) pitch_compensation=-1.951f*distance_xyz+9.845f;
//	else pitch_compensation=-1.951f*distance_xyz+10.845f;
//	//-1.951f*distance_xyz+8.345f
//	if( AutoAim_Data_Receive.track == 0 )
//	{
//		auto_aim_yaw_exp = 0;
//		auto_aim_pitch_exp = 0;
//	}
//	else
//	{
//		auto_aim_yaw_exp = yaw_tra * 180.0f / 3.141592653589f;
//		auto_aim_pitch_exp = pitch_tra * 180.0f / 3.141592653589f + pitch_compensation;
//	}
//	
//}

void Gimbal_Motor_Init(void)
{
	const static fp32 yaw_motor_speed_pid[3] = {YAW_MOTOR_SPEED_PID_KP, YAW_MOTOR_SPEED_PID_KI, YAW_MOTOR_SPEED_PID_KD};
	const static fp32 yaw_motor_angle_pid[3] = {YAW_MOTOR_ANGLE_PID_KP, YAW_MOTOR_ANGLE_PID_KI, YAW_MOTOR_ANGLE_PID_KD};
	const static fp32 yaw_motor_auto_aim_pid[3] = {YAW_MOTOR_AUTO_AIM_PID_KP, YAW_MOTOR_AUTO_AIM_PID_KI, YAW_MOTOR_AUTO_AIM_PID_KD};
	
	const static fp32 pitch_motor_speed_pid[3] = {PITCH_MOTOR_SPEED_PID_KP, PITCH_MOTOR_SPEED_PID_KI, PITCH_MOTOR_SPEED_PID_KD};
	const static fp32 pitch_motor_angle_pid[3] = {PITCH_MOTOR_ANGLE_PID_KP, PITCH_MOTOR_ANGLE_PID_KI, PITCH_MOTOR_ANGLE_PID_KD};
	const static fp32 pitch_motor_auto_aim_pid[3] = {PITCH_MOTOR_AUTO_AIM_PID_KP, PITCH_MOTOR_AUTO_AIM_PID_KI, PITCH_MOTOR_AUTO_AIM_PID_KD};
	
	for(uint8_t i=0;i<2;i++)
	{
		gimbal_m6020[i].INS_speed=0;
		gimbal_m6020[i].INS_speed_set=0;
		gimbal_m6020[i].INS_angle=0;
		gimbal_m6020[i].INS_angle_set=0;
		gimbal_m6020[i].ENC_angle=0;
		gimbal_m6020[i].ENC_angle_actual=0;
		gimbal_m6020[i].ENC_angle_set=0;
		gimbal_m6020[i].ENC_speed =0;
		gimbal_m6020[i].give_current=0;
	}
	
	DM_pitch_motor_data.INS_angle =0;
	DM_pitch_motor_data.INS_angle_set=0;
	DM_pitch_motor_data.INS_speed=0;
	DM_pitch_motor_data.INS_speed_set=0;
	DM_pitch_motor_data.target_pos=0;
	DM_pitch_motor_data.target_vel=0;
	DM_pitch_motor_data.target_current=0;
	DM_pitch_motor_data.pos=0;
	DM_pitch_motor_data.vel=0;
	DM_pitch_motor_data.toq=0;
	
	PID_init(&gimbal_m6020[0].speed_pid,PID_POSITION,yaw_motor_speed_pid,YAW_MOTOR_SPEED_PID_MAX_OUT,YAW_MOTOR_SPEED_PID_MAX_IOUT);
	PID_init(&gimbal_m6020[0].angle_pid,PID_POSITION,yaw_motor_angle_pid,YAW_MOTOR_ANGLE_PID_MAX_OUT,YAW_MOTOR_ANGLE_PID_MAX_IOUT);
	PID_init(&gimbal_m6020[0].auto_aim_pid,PID_POSITION,yaw_motor_auto_aim_pid,YAW_MOTOR_AUTO_AIM_PID_MAX_OUT,YAW_MOTOR_AUTO_AIM_PID_MAX_IOUT);
	
	PID_init(&DM_pitch_motor_data.speed_pid,PID_POSITION,pitch_motor_speed_pid,PITCH_MOTOR_SPEED_PID_MAX_OUT,PITCH_MOTOR_SPEED_PID_MAX_IOUT);
	PID_init(&DM_pitch_motor_data.angle_pid,PID_POSITION,pitch_motor_angle_pid,PITCH_MOTOR_ANGLE_PID_MAX_OUT,PITCH_MOTOR_ANGLE_PID_MAX_IOUT);	
	PID_init(&DM_pitch_motor_data.auto_aim_pid,PID_POSITION,pitch_motor_auto_aim_pid,PITCH_MOTOR_AUTO_AIM_PID_MAX_OUT,PITCH_MOTOR_AUTO_AIM_PID_MAX_IOUT);		
	
}

void Gimbal_Motor_Data_Update(void)
{
	//yaw
	//z2+y2
	fp32 temp;
	
	arm_sqrt_f32(bmi088_real_data.gyro[2]*bmi088_real_data.gyro[2]+bmi088_real_data.gyro[0]*bmi088_real_data.gyro[0],&temp);	
	if(bmi088_real_data.gyro[2]<0)
	{
		temp=-temp;
	}
	//gimbal_m6020[0].INS_speed=temp*0.1f+gimbal_m6020[0].INS_speed*0.9f;
	gimbal_m6020[0].INS_speed=bmi088_real_data.gyro[2]*RAD_TO_ANGLE;
//	gimbal_m6020[0].INS_speed=bmi088_real_data.gyro[2];
	gimbal_m6020[0].INS_angle=INS_angle_deg[0];
	gimbal_m6020[0].ENC_angle=motor_measure_gimbal[0].ecd;	
	gimbal_m6020[0].ENC_speed=motor_measure_gimbal[0].speed_rpm;
	
	//pitch
	//gimbal_m6020[1].INS_speed=bmi088_real_data.gyro[2]*0.1f+gimbal_m6020[1].INS_speed*0.9f;
//	gimbal_m6020[1].INS_speed=bmi088_real_data.gyro[0];
//	gimbal_m6020[1].INS_angle=-INS_angle_deg[1];
//	gimbal_m6020[1].ENC_angle=motor_measure_gimbal[1].ecd;
	DM_pitch_motor_data.INS_speed=bmi088_real_data.gyro[1];
	DM_pitch_motor_data.INS_angle=INS_angle_deg[2];
	
	
}
uint16_t test_err=0,test_ins_speed_set=0;
void Yaw_Motor_Control(void)
{
	
//	auto_aim_id=AutoAim_Data_Receive.Aimed_ID;
	auto_aim_tracking=AutoAim_Data_Receive.track;
	
////	开启自瞄的两种方法：1、不开导航的情况下开自瞄  2、开启导航和自瞄
	if((auto_aim_tracking)||(rc_ctrl.rc.s[1]==RC_SW_UP&&auto_aim_tracking))
	{		  
		yaw_angle_err = AutoAim_Data_Receive.yaw_aim-gimbal_m6020[0].INS_angle;
		if(yaw_angle_err>180)
			yaw_angle_err-=360;
		else
			if(yaw_angle_err<-180)
				yaw_angle_err+=360;

		PID_calc(&gimbal_m6020[0].auto_aim_pid,yaw_angle_err,0);		
		gimbal_m6020[0].INS_speed_set=(-gimbal_m6020[0].auto_aim_pid.out)+(gimbal_m6020[0].INS_speed-gimbal_m6020[0].INS_speed_last)*3.5; //乘0.8的目标yaw角速度、前馈
		gimbal_m6020[0].INS_angle_set= AutoAim_Data_Receive.yaw_aim;
		yaw_mode=yaw_mode_last=1;
//		gimbal_m6020[0].INS_angle_set=gimbal_m6020[0].INS_angle;
	}
	
	// 遥控器控制 并且排除没有遥控器连接的情况
	else if(rc_ctrl.rc.s[1]==RC_SW_MID)
	{
		yaw_mode_last=yaw_mode;
		if(rc_ctrl.rc.ch[0]>10||rc_ctrl.rc.ch[0]<-10)
		{			
			yaw_mode=0;//0遥控器有输入,1遥控器无输入
		}
		else
		{
			yaw_mode=1;
		}
		
//		if(chassis_follow_gimbal_changing==1) //好像永远无法进入这个条件，没有置1的代码
//		{
//			yaw_mode=1;
//			yaw_mode_last=yaw_mode;
//		}

		if(yaw_mode==0)
		{
			gimbal_m6020[0].INS_speed_set=-(float)rc_ctrl.rc.ch[0]/660.0f*5.0f*RAD_TO_ANGLE;
//			gimbal_m6020[0].INS_speed_set=100;
		}
		else if(yaw_mode==1&&yaw_mode_last==0)//记录遥控器从给yaw轴电机速度到不给速度的一瞬间的imu数据
		{
			gimbal_m6020[0].INS_angle_set=gimbal_m6020[0].INS_angle;
		}
			
		if(yaw_mode==1)//遥控器无输入，yaw轴定死
		{
			yaw_angle_err=(gimbal_m6020[0].INS_angle_set-gimbal_m6020[0].INS_angle);
			if(yaw_angle_err>180) yaw_angle_err-=360;
			else if(yaw_angle_err<-180) yaw_angle_err+=360;
			
			PID_calc(&gimbal_m6020[0].angle_pid,yaw_angle_err,0);
			gimbal_m6020[0].INS_speed_set=-gimbal_m6020[0].angle_pid.out;
		}

	}
	else if(rc_ctrl.rc.s[1]==RC_SW_UP) // 导航
	{
//			gimbal_m6020[0].INS_speed_set= Chassis_Data_Receive.yaw_speed/1000.0f*RAD_TO_ANGLE;
			gimbal_m6020[0].INS_speed_set= Chassis_Data_Receive.yaw_speed*RAD_TO_ANGLE;
	}


	PID_calc(&gimbal_m6020[0].speed_pid,gimbal_m6020[0].INS_speed,gimbal_m6020[0].INS_speed_set);
	gimbal_m6020[0].give_current=gimbal_m6020[0].speed_pid.out;
}

fp32 pitch_err=0;

void Pitch_Motor_Control(void)
{
//	gimbal_m6020[1].ENC_angle_actual=(float)(((uint16_t)gimbal_m6020[1].ENC_angle+(8192-5077))%8192)/8192.0f*360.0f;
//	if(gimbal_m6020[1].ENC_angle_actual>180)
//	{
//		gimbal_m6020[1].ENC_angle_actual-=360;
//	}
	
	//开自瞄 
	if((auto_aim_tracking) || (rc_ctrl.rc.s[1]==RC_SW_UP&&auto_aim_tracking))  
	{  
		
//		pitch_angle_err=auto_aim_pitch_exp-gimbal_m6020[2].INS_angle;
		pitch_angle_err=(-AutoAim_Data_Receive.pitch_aim)-DM_pitch_motor_data.INS_angle;
		PID_calc(&DM_pitch_motor_data.auto_aim_pid,pitch_angle_err,0);

		DM_pitch_motor_data.INS_speed_set=(-DM_pitch_motor_data.auto_aim_pid.out)+(DM_pitch_motor_data.INS_speed-DM_pitch_motor_data.INS_speed_last)*0.5;
		DM_pitch_motor_data.INS_angle_set=AutoAim_Data_Receive.pitch_aim;
		
		pitch_mode=pitch_mode_last=1;
		
//		gimbal_m6020[1].INS_angle_set=gimbal_m6020[1].INS_angle;
	}
	
	// 手控
	else if(rc_ctrl.rc.s[1]==RC_SW_MID)  
	{
		pitch_mode_last=pitch_mode;
		if((rc_ctrl.rc.ch[1]>5||rc_ctrl.rc.ch[1]<-5))
		{			
			pitch_mode=0;
		}
		else
		{
			pitch_mode=1;
		}
		
//		pitch_mode=0;
		
		if(pitch_mode==0)
		{		
				DM_pitch_motor_data.INS_speed_set=(float)rc_ctrl.rc.ch[1]/660.0f*5.0f;
		}
		else if(pitch_mode==1&&pitch_mode_last==0)
		{
			DM_pitch_motor_data.INS_angle_set=DM_pitch_motor_data.INS_angle;

	//		gimbal_m6020[1].ENC_angle_set=gimbal_m6020[1].ENC_angle_actual;
		}
		
		if(pitch_mode==1)
		{
//			gimbal_m6020[1].INS_angle_set=60*arm_sin_f32(5*tt);
//			if(gimbal_m6020[1].INS_angle_set>16)gimbal_m6020[1].INS_angle_set=16;
//			if(gimbal_m6020[1].INS_angle_set<-20)gimbal_m6020[1].INS_angle_set=-20;
			
//			if(gimbal_m6020[1].INS_angle_set>0)gimbal_m6020[1].INS_angle_set=20;
//			if(gimbal_m6020[1].INS_angle_set<-0)gimbal_m6020[1].INS_angle_set=-20;

			PID_calc(&DM_pitch_motor_data.angle_pid,DM_pitch_motor_data.INS_angle,DM_pitch_motor_data.INS_angle_set);
			pitch_err=DM_pitch_motor_data.INS_angle_set-DM_pitch_motor_data.INS_angle;
			DM_pitch_motor_data.INS_speed_set=DM_pitch_motor_data.angle_pid.out;
			
//			if(gimbal_m6020[1].ENC_angle>5600&&gimbal_m6020[1].INS_speed_set>0)gimbal_m6020[1].INS_speed_set=0;
	//		PID_calc(&gimbal_m6020[1].angle_pid,gimbal_m6020[1].ENC_angle_actual,gimbal_m6020[1].ENC_angle_set);
	//		gimbal_m6020[1].INS_speed_set=gimbal_m6020[1].angle_pid.out;
		}
	}
	else if(rc_ctrl.rc.s[1]==RC_SW_UP) // 如果处于自瞄状态：就让pitch上下摇
	{
			PID_calc(&DM_pitch_motor_data.angle_pid,DM_pitch_motor_data.INS_angle,auto_pitch_watch);
			DM_pitch_motor_data.INS_speed_set=DM_pitch_motor_data.angle_pid.out+0.05f;
	}
	
	// 电子限位
	if((DM_pitch_motor_data.p_int>PITCH_ECD_ANGLE_MAX||DM_pitch_motor_data.p_int < PITCH_ECD_ANGLE_MIN) && (DM_pitch_motor_data.p_int <50000))
	{
	  if(DM_pitch_motor_data.p_int<PITCH_ECD_ANGLE_MIN && DM_pitch_motor_data.INS_speed_set>0)
		{ 
			DM_pitch_motor_data.INS_speed_set=0;
		  DM_pitch_motor_data.INS_angle_set=DM_pitch_motor_data.INS_angle;
		}
		if(DM_pitch_motor_data.p_int > PITCH_ECD_ANGLE_MAX && DM_pitch_motor_data.INS_speed_set<0)
		{ 
			DM_pitch_motor_data.INS_speed_set=0;
		  DM_pitch_motor_data.INS_angle_set=DM_pitch_motor_data.INS_angle;
		}
	}
	PID_calc(&DM_pitch_motor_data.speed_pid,DM_pitch_motor_data.INS_speed,DM_pitch_motor_data.INS_speed_set);
	DM_pitch_motor_data.target_current=-DM_pitch_motor_data.speed_pid.out;
//	if(gimbal_m6020[1].ENC_angle<4200&&gimbal_m6020[1].speed_pid.out>0)gimbal_m6020[1].give_current=0.2*gimbal_m6020[1].speed_pid.out;
//	if(gimbal_m6020[1].ENC_angle>3250&gimbal_m6020[1].speed_pid.out<0)gimbal_m6020[1].give_current=0.2*gimbal_m6020[1].speed_pid.out;
}
/****************************************************system identify***********************************************************/
//uint8_t BUFF_niming[30];
//void niming_sent_data(float A,float B,float time)//匿名地?端打包函数
//{
//int i;
//uint8_t sumcheck = 0;
//uint8_t addcheck = 0;
//uint8_t _cnt=0;
//BUFF_niming[_cnt++]=0xAA;
//BUFF_niming[_cnt++]=0xFF;
//BUFF_niming[_cnt++]=0XF1;
//BUFF_niming[_cnt++]=12;
//BUFF_niming[_cnt++]=Get_BYTE0(A);
//BUFF_niming[_cnt++]=Get_BYTE1(A);
//BUFF_niming[_cnt++]=Get_BYTE2(A);
//BUFF_niming[_cnt++]=Get_BYTE3(A);
//BUFF_niming[_cnt++]=Get_BYTE0(B);
//BUFF_niming[_cnt++]=Get_BYTE1(B);
//BUFF_niming[_cnt++]=Get_BYTE2(B);
//BUFF_niming[_cnt++]=Get_BYTE3(B);
//BUFF_niming[_cnt++]=Get_BYTE0(time);
//BUFF_niming[_cnt++]=Get_BYTE1(time);
//BUFF_niming[_cnt++]=Get_BYTE2(time);
//BUFF_niming[_cnt++]=Get_BYTE3(time);
//for(i=0;i<BUFF_niming[3]+4;i++)
//{
//sumcheck+=BUFF_niming[i];
//addcheck+=sumcheck;
//}
//BUFF_niming[_cnt++]=sumcheck;
//BUFF_niming[_cnt++]=addcheck;
//HAL_UART_Transmit_DMA(&huart1,BUFF_niming,_cnt);
//}

//void Yaw_Motor_Identification()//系统辨识函数
//{
//static float sin_time = 0;
//static float last_sin_time = 0;
//static uint8_t i_sin = 0;
//static float phtic = 0;
//static float f[64] = {1.000000, 1.499250, 2.000000, 2.500000, 3.003003, 3.496503, 4.000000, 4.504505,
//											5.000000, 5.494505, 5.988024, 6.493506, 6.993007, 7.518797, 8.000000, 8.474576, 9.009009, 9.523810,
//											10.000000, 10.526316, 10.989011, 11.494253, 12.048193, 12.500000, 12.987013, 13.513514, 14.084507,
//											14.492754, 14.925373, 15.384615, 15.873016, 16.393443, 16.949153, 17.543860, 17.857143, 18.518519,
//											18.867925, 19.607843, 20.000000, 20.408163, 20.833333, 21.276596, 22.222222, 23.809524, 26.315789,
//											27.777778, 30.303030, 32.258065, 34.482759, 35.714286, 38.461538, 40.000000, 50.0000, 58.8235, 71.4286,
//											76.9231, 90.9091, 100.0000, 111.1111, 125.0000, 200.0000, 250.0000, 333.3333, 500.0000};
//	if(identify_flag)
//	{
//		phtic += f[i_sin] * 0.002;
//		float speed_set = 25000 * sin(phtic);
//		sin_time += 0.002;
//		PID_calc(&gimbal_m6020[0].speed_pid, gimbal_m6020[0].INS_speed, speed_set);
//		gimbal_m6020[0].give_current = gimbal_m6020[0].speed_pid.out;
//		CAN_Gimbal_CMD((int16_t)speed_set, gimbal_m6020[1].give_current, 0, 0);//需更换为被调参机器?的云台电机控制代码
//		niming_sent_data(speed_set,gimbal_m6020[0].INS_speed,sin_time);
//		if(last_sin_time + 3.1415926*4/(float)f[i_sin] <= sin_time)
//		{
//			i_sin++;
//			last_sin_time = sin_time;
//		}
//		if(i_sin > 63)
//		{
//			identify_flag = 0;
//		}
//	}
//}

void Pitch_Updown(void)
{
	static uint8_t flag=0;
	if( flag == 0 )
	{
		auto_pitch_watch += 0.1f;
		if(AutoAim_Data_Receive.pitch_speed == 0)
		{
			if(auto_pitch_watch>=25.0f)
			{
				flag = 1;
				auto_pitch_watch = 25.0f;
			}
		}		
		else
		{
			if(auto_pitch_watch>=25.0f)
			{
				flag = 1;
				auto_pitch_watch = 25.0f;
			}
		}
		
	}
	else
	{
		auto_pitch_watch -= 0.1f;
		if( auto_pitch_watch <=-25 )
		{
			flag = 0;
			auto_pitch_watch=-25;
		}
	}
}

void DM_reset(){
	if(Game_Robot_State.power_management_gimbal_output && !gimbal_output_last)
        {		
            enable_send_count = 5; // 设置发送计数器,使能达秒十次
						HAL_Delay(1000);
        }
        gimbal_output_last = Game_Robot_State.power_management_gimbal_output;
				
				while(enable_send_count>0)
        {
//						HAL_Delay(1000);
            enable_DM(DM4310_ID, 0x01);
						enable_send_count--;				
        }
}

void Gimbal_Task(void const * argument)
{
	Gimbal_Motor_Init();
	rc_ctrl.rc.s[1]=RC_SW_DOWN;
	HAL_Delay(1200);
	enable_DM(DM4310_ID,0x01);
	
	vTaskDelay(200);
	
	while(1)
	{
		DM_reset();
		Gimbal_Motor_Data_Update();
		
	//	angle_calculate();
		
		Yaw_Motor_Control();
		Pitch_Updown();
		Pitch_Motor_Control();
		//Chassis_Gimbal_Angle_Tramsit.chassis_follow_gimbal_angle=DM_pitch_motor_data.pos; // 这个参数好像没用到
		gimbal_m6020[0].INS_speed_last=gimbal_m6020[0].INS_speed;
		DM_pitch_motor_data.INS_speed_last=DM_pitch_motor_data.INS_speed;
		if(rc_ctrl.rc.s[1]==RC_SW_DOWN)
		{
			CAN_Gimbal_CMD(0,0,0,0);
			ctrl_motor(DM4310_ID, 0, 0,0, 0, 0);
		}
		else
		{
			CAN_Gimbal_CMD(gimbal_m6020[0].give_current,0,0,0);	
//			CAN_Gimbal_CMD(0,shoot_m2006[0].give_current,0,0);
			ctrl_motor(DM4310_ID, 0, 0,0, 0, DM_pitch_motor_data.target_current);
//			ctrl_motor(DM4310_ID, 0, 0,0, 0, 0);
		}

	Vofa_Send_Data4(AutoAim_Data_Receive.yaw_aim,gimbal_m6020[0].INS_angle,(float)AutoAim_Data_Receive.fire_or_not,0);
//		Vofa_Send_Data4((float)DM_pitch_motor_data.INS_angle_set,(float)DM_pitch_motor_data.INS_angle,DM_pitch_motor_data.INS_speed,DM_pitch_motor_data.INS_speed_set);
		vTaskDelay(1);
	}
}

/*********************************************************达妙电机控制程序*********************************************************/
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 50.0f
#define POS_MIN -12.5f
#define POS_MAX 12.5f
#define SPD_MIN -18.0f
#define SPD_MAX 18.0f
#define T_MIN -30.0f
#define T_MAX 30.0f
#define I_MIN -30.0f
#define I_MAX 30.0f

void ctrl_motor(uint16_t id, float _pos, float _vel,float _KP, float _KD, float _torq)//can1
{
	uint8_t TX_Data[8];
	uint32_t send_mail_box;
	CAN_TxHeaderTypeDef Tx_Msg;
	
	//uint8_t *pbuf,*vbuf;
	Tx_Msg.StdId=id;	 //set chassis motor current.
	Tx_Msg.IDE=CAN_ID_STD;		  // 不使用扩展标识符
	Tx_Msg.RTR=CAN_RTR_DATA;		  // 消息类型为数据帧，一帧8位
	Tx_Msg.DLC=8; 

	uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
	pos_tmp = float_to_uint(_pos, -12.5, 12.5, 16);
	vel_tmp = float_to_uint(_vel, -45, 45, 12);
	kp_tmp = float_to_uint(_KP, KP_MIN, KP_MAX, 12);
	kd_tmp = float_to_uint(_KD, KD_MIN, KD_MAX, 12);
	tor_tmp = float_to_uint(_torq, T_MIN, T_MAX, 12);
	
	TX_Data[0] = (pos_tmp >> 8);
	TX_Data[1] = pos_tmp;
	TX_Data[2] = (vel_tmp >> 4);
	TX_Data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
	TX_Data[4] = kp_tmp;
	TX_Data[5] = (kd_tmp >> 4);
	TX_Data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
	TX_Data[7] = tor_tmp;

	HAL_CAN_AddTxMessage(&hcan2, &Tx_Msg, TX_Data, &send_mail_box);  
}

void enable_DM(uint8_t id, uint8_t ctrl_mode)
{
	uint8_t TX_Data[8];
	uint32_t send_mail_box;
	CAN_TxHeaderTypeDef Tx_Msg;
	
	if(ctrl_mode==1)			Tx_Msg.StdId=0x000+DM4310_ID;	 //set chassis motor current.
	else if(ctrl_mode==2)	Tx_Msg.StdId=0x100+DM4310_ID;	 //set chassis motor current.
	else if(ctrl_mode==3)	Tx_Msg.StdId=0x200+DM4310_ID;	 //set chassis motor current.
	Tx_Msg.IDE=CAN_ID_STD;		  // 不使用扩展标识符
	Tx_Msg.RTR=CAN_RTR_DATA;		  // 消息类型为数据帧，一帧8位
	Tx_Msg.DLC=8; 
	
	TX_Data[0] = 0xff;
	TX_Data[1] = 0xff;
	TX_Data[2] = 0xff;
	TX_Data[3] = 0xff;
	TX_Data[4] = 0xff;
	TX_Data[5] = 0xff;
	TX_Data[6] = 0xff;
	TX_Data[7] = 0xfc;

	HAL_CAN_AddTxMessage(&hcan2, &Tx_Msg, TX_Data, &send_mail_box);    //将数据储存进邮箱FIFOx
}

//失能
void disable_DM(uint8_t id, uint8_t ctrl_mode)
{
	uint8_t TX_Data[8];
	uint32_t send_mail_box;
	CAN_TxHeaderTypeDef Tx_Msg;
	
	if(ctrl_mode==1)			Tx_Msg.StdId=0x000+DM4310_ID;	 //set chassis motor current.
	else if(ctrl_mode==2)	Tx_Msg.StdId=0x100+DM4310_ID;	 //set chassis motor current.
	else if(ctrl_mode==3)	Tx_Msg.StdId=0x200+DM4310_ID;	 //set chassis motor current.

	Tx_Msg.IDE=CAN_ID_STD;		  // 不使用扩展标识符
	Tx_Msg.RTR=CAN_RTR_DATA;		  // 消息类型为数据帧，一帧8位
	Tx_Msg.DLC=8; 
	
	TX_Data[0] = 0xff;
	TX_Data[1] = 0xff;
	TX_Data[2] = 0xff;
	TX_Data[3] = 0xff;
	TX_Data[4] = 0xff;
	TX_Data[5] = 0xff;
	TX_Data[6] = 0xff;
	TX_Data[7] = 0xfd;

	HAL_CAN_AddTxMessage(&hcan2, &Tx_Msg, TX_Data, &send_mail_box);    //将数据储存进邮箱FIFOx
}
