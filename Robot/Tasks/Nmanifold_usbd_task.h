#ifndef NMANIFOLD_USBD_TASK_H
#define NMANIFOLD_USBD_TASK_H

#include <stdint.h>

#include "main.h"
#include "cmsis_os.h"
#include "struct_typedef.h"

#include "INS_task.h"
#include "arm_math.h"

#include "usbd_cdc_if.h"
/****************************************常量定义段********************************************/
#define USBD_RX_BUF_LENGHT		APP_RX_DATA_SIZE
#define USBD_TX_BUF_LENGHT		APP_TX_DATA_SIZE

//命令字ID
#define CMD_ID_AUTOAIM_DATA_RX	 		0x81			//接收自瞄数据（上→下）
//#define CMD_ID_IMU					0x11			//IMU相关数据（下→上）
#define CMD_ID_AUTOAIM_DATA_TX	 		0x14			//发送自瞄数据（下→上）
#define CMD_ID_WORKING_MODE			 	0x15			//操作模式（下→上）
#define CMD_ID_DIAL_SWITCH			 	0x17			//拨码开关（下→上）
#define	CMD_ID_REFEREE_DATA_TX			0x18			//裁判系统穿透（下 上）

#define CMD_ID_MOVE_CMD_DATA_RX      	0x82      //接收底盘控制数据（上->下）（哨兵专用）
#define CMD_ID_ROTATE_DATA_RX       	0x85      //接收小陀螺速度（上->下）（哨兵专用）
#define CMD_ID_CHASSIA_GIMBAL_ANGLE 	0X16      //发送底盘相对云台角度（下->上）（哨兵专用）
#define CMD_ID_CHASSIS_DATA_TX	 		0x12      //上传底盘轮速计数据（下->上）（哨兵专用）

//数据段长度，以字节为单位
//#define LENGTH_IMU							40			//IMU相关数据（哨兵，下→上）
#define LENGTH_AUTOAIM_DATA_RX		34			//接收自瞄数据（上→下）
#define LENGTH_AUTOAIM_DATA_TX		12			//发送自瞄数据（下→上） 4+4+4+1
#define LENGTH_DIAL_SWITCH				4				//拨码开关（下→上）
#define LENGTH_WORKING_MODE				1				//操作模式（下→上）


#define LENGTH_CHASSIS_GIMBAL_ANGLE 4     //发送底盘相对云台角度（下->上）（哨兵专用）
#define LENGTH_CHASSIS_DATA_TX    24      //上传底盘轮速计数据（下->上）（哨兵专用）
#define LENGTH_REFEREE_DATA_TX		48			//发送决策需要的数据 （哨兵专用）
//#define LENGTH_move_cmd_DATA_RX    20      //接收整车控制数据（上->下）（哨兵专用） 没用到
#define LENGTH_NUC_DATA_RX		69		//	接收整车控制数据（上->下）（哨兵专用） （nuc数据打包发下，导航和自瞄）

//自瞄模式
#define AUTOAIM_MODE_NORMAL						0x00		//自瞄模式：普通
#define AUTOAIM_MODE_SMALL_ENERGY			0x01		//自瞄模式：小能量机关
#define AUTOAIM_MODE_BIG_ENERGY				0x02		//自瞄模式：大能量机关
#define AUTOAIM_MODE_ANTI_TOP					0x03		//自瞄模式：反小陀螺

//自瞄装甲
#define AUTOAIM_ARMOR_AUTO					0x00		//自瞄装甲：自动
#define AUTOAIM_ARMOR_SMALL					0x04		//自瞄装甲：小装甲模块
#define AUTOAIM_ARMOR_BIG						0x08		//自瞄装甲：大装甲模块

//拨码开关
#define SWITCH_OFF									1				//不启用
#define SWITCH_ON										2				//敌方队伍颜色：蓝
#define ENEMY_INFANTRY_ARMOR_SMALL	0				//敌方步兵装甲：小装甲
#define ENEMY_INFANTRY_ARMOR_BIG		1				//敌方步兵装甲：大装甲


/*******************************************END**********************************************/



#pragma pack(push, 1)

/*************************************发送结构体定义段****************************************/
typedef struct{
	uint8_t Header;			//帧头
	uint8_t Length;			//帧长（包含帧头与校验位）
	uint8_t Cmd_ID;			//命令字
}__attribute__((__packed__)) Protocol_Head_Data;	//通信协议数据流前段数据

/*typedef struct{
	float Gyro[3];			//角速度（X，Y，Z）
	float Accel[3];			//加速度（X，Y，Z）
	float Quat[4];			//四元数（X，Y，Z，W）
}__attribute__((__packed__)) IMU_Data;//IMU数据*/

typedef struct{
	float Yaw;							//当前yaw（°）
	float Pitch;						//当前pitch（°）
	float Roll;							//当前Roll（°）
//	uint8_t Bullet_Speed;				//子弹弹速（m/s）
	//uint8_t Robot_Team_Color;			//机器人所属队伍的颜色
}__attribute__((__packed__)) AutoAim_Data_Tx;

typedef struct{
	float x;					  //前方向数据（m）
	float y;						//左方向数据（m）
	float z;						//逆时针方向数据（m）
	float vx;           //前方向速度(m/s)
	float vy;           //左方向速度(m/s)
	float wz;           //逆时针转动速度(rad/s)
	//uint8_t Robot_Team_Color;		//机器人所属队伍的颜色
}__attribute__((__packed__)) Chassis_Data_Tx;

typedef struct{
	float chassis_follow_gimbal_angle;
}__attribute__((__packed__)) Chassis_Gimbal_Angle_TX;//通信协议数据流前段数据

typedef uint8_t Working_Mode;//操作模式

typedef struct{
	uint8_t Switch_off;				//是否启用拨码开关 1不启用，2启用
	uint8_t Infantry_Armor[3];	//依次为敌方3、4、5号步兵的装甲类型（0:小装甲；1:大装甲）
}__attribute__((__packed__)) Dial_Switch_Data;//拨码开关

typedef struct{
	uint16_t remain_HP;					//剩余血量
	uint16_t max_HP;						//总血量
														
	uint8_t	game_progress; 				//比赛阶段
	uint16_t stage_remain_time;		//比赛剩余时间
	uint16_t coin_remaining_num; 	//剩余经济
	uint16_t bullet_remaining_num_17mm;	//剩余发弹量

	uint16_t red_1_HP;					//双方血量
	uint16_t red_2_HP;
	uint16_t red_3_HP;
	uint16_t red_4_HP;
	uint16_t red_7_HP;
	uint16_t red_outpost_HP;
	uint16_t red_base_HP;
	
	uint16_t blue_1_HP;
	uint16_t blue_2_HP;
	uint16_t blue_3_HP;
	uint16_t blue_4_HP;
	uint16_t blue_7_HP;
	uint16_t blue_outpost_HP;
	uint16_t blue_base_HP;
		
	uint32_t rfid_status;					//rfid扫描内容
	uint32_t event_data;
	uint8_t hurt_reason;
	
}__attribute__((__packed__)) Referee_Data_Tx;
/*******************************************END**********************************************/

#pragma pack(pop)


/*************************************接收结构体定义段****************************************/
typedef struct{
	float yaw_aim;
	float pitch_aim;
	bool_t fire_or_not;
	bool_t track;
	float vx;
	float vy;                                                   

	
	float rotate;
	float yaw_speed;
	float pitch_speed;	
}__attribute__((__packed__)) AutoAim_Data_Rx;//自瞄数据

typedef struct{
	fp32 vx;//	int16_t vx;						//前方向速度（m/s)
	fp32 vy;//	int16_t vy;					//左方向速度（m/s)
	fp32 yaw_speed;//	int16_t yaw_speed;			//yaw轴速度(rad/s)
	fp32 pitch_speed;
}__attribute__((__packed__)) Chassis_Data_Rx;//底盘数据

typedef struct{
	float vx;//	int16_t vx;						//前方向速度（m/s)
	float vy;//	int16_t vy;					//左方向速度（m/s)
	float rotate;
	float yaw_speed;//	int16_t yaw_speed;			//yaw轴速度(rad/s)
	float pitch_speed;

}__attribute__((__packed__)) Move_cmd_Data_Rx;//底盘数据 自瞄数据

typedef struct{
	int16_t rotate;						//逆时针速度(rad/s)
}__attribute__((__packed__)) Rotate_Data_Rx;//小陀螺数据
/*******************************************END**********************************************/




//对外接口
extern AutoAim_Data_Rx AutoAim_Data_Receive;
extern Chassis_Data_Rx Chassis_Data_Receive;
extern Chassis_Data_Tx Chassis_Data_Tramsit;
extern uint8_t Autoaim_Mode, Autoaim_Armor;
extern Rotate_Data_Rx Rotate_Data_Receive;
extern Chassis_Gimbal_Angle_TX Chassis_Gimbal_Angle_Tramsit;
extern uint8_t Referee_Buffer[2][512];
extern uint16_t LENTH_REFEREE_BUF;
extern Dial_Switch_Data Dial_Switch;


//函数声明
void manifold_USBD_task(void);
uint8_t USBD_IRQHandler(uint8_t* Buf, uint16_t Len);
uint8_t NUC_Data_Unpack(void);
void NUC_USBD_Tx(uint8_t cmdid);
uint8_t CRC_Calculation(uint8_t *ptr, uint16_t len);



//8位版本CRC表
static const uint8_t CRC08_Table[256] = {
    0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
    0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
    0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
    0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
    0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
    0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
    0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
    0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
    0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
    0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
    0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
    0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
    0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
    0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
    0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
    0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
};



#endif
