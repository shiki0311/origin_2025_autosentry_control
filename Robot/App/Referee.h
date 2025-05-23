/**
  ******************************************************************************
  * @file    referee.h
  * @author  Shiki
  * @version V1.3.0
  * @date    2025/03/1
  * @brief   Header file of referee.c
  ******************************************************************************
  * @attention
	*
	*   依据裁判系统 串口协议附录 V1.7
	*
  ******************************************************************************
  */
	
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __REFEREE_H__
#define __REFEREE_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "protocol.h"
#include "fifo.h"

/* Defines -------------------------------------------------------------------*/
#define Referee_UART huart6
#define Referee_IRQHandler USART6_IRQHandler

/* Referee Defines -----------------------------------------------------------*/
/* 比赛类型 */
#define Game_Type_RMUC     1 //超级对抗赛
#define Game_Type_RMUT     2 //单项赛
#define Game_Type_RMUA     3 //人工智能挑战赛
#define Game_Type_RMUL_3V3 4 //高校联盟赛3V3
#define Game_Type_RMUL_1V1 5 //高校联盟赛1V1

/* 比赛阶段 */
#define Game_Progress_Unstart   0 //未开始比赛
#define Game_Progress_Prepare   1 //准备阶段
#define Game_Progress_SelfCheck 2 //自检阶段
#define Game_Progress_5sCount   3 //5s倒计时
#define Game_Progress_Battle    4 //对战中
#define Game_Progress_Calculate 5 //比赛结算中

/* 比赛结果 */
#define Game_Result_Draw    0 //平局
#define Game_Result_RedWin  1 //红方胜利
#define Game_Result_BlueWin 2 //蓝方胜利

/* 警告信息 */
#define Warning_Yellow  1 //黄牌警告
#define Warning_Red     2 //红牌警告
#define Warning_Failure 3 //判负

/* 机器人ID */
#define Robot_ID_Red_Hero         1 //红方英雄
#define Robot_ID_Red_Engineer     2 //红方工程
#define Robot_ID_Red_Infantry3    3 //红方步兵3
#define Robot_ID_Red_Infantry4    4 //红方步兵4
#define Robot_ID_Red_Infantry5    5 //红方步兵5
#define Robot_ID_Red_Aerial       6 //红方无人机
#define Robot_ID_Red_Sentry       7 //红方哨兵
#define Robot_ID_Red_Darts        8 //红方飞镖
#define Robot_ID_Red_Radar        9 //红方雷达
#define Robot_ID_Blue_Hero      101 //蓝方英雄
#define Robot_ID_Blue_Engineer  102 //蓝方工程
#define Robot_ID_Blue_Infantry3 103 //蓝方步兵3
#define Robot_ID_Blue_Infantry4 104 //蓝方步兵4
#define Robot_ID_Blue_Infantry5 105 //蓝方步兵5
#define Robot_ID_Blue_Aerial    106 //蓝方无人机
#define Robot_ID_Blue_Sentry    107 //蓝方哨兵
#define Robot_ID_Blue_Darts     108 //蓝方飞镖
#define Robot_ID_Blue_Radar     109 //蓝方雷达
#define Referee_Server       0x8080 //裁判系统服务器，用于哨兵和雷达自主决策指令

/* 机器人等级 */
#define Robot_Level_1 1 //1级
#define Robot_Level_2 2 //2级
#define Robot_Level_3 3 //3级

/* 扣血类型 */
#define Hurt_Type_ArmoredPlate     0 //装甲板伤害
#define Hurt_Type_ModuleOffline    1 //模块离线
#define Hurt_Type_OverShootSpeed   2 //枪口超射速
#define Hurt_Type_OverShootHeat    3 //枪管超热量
#define Hurt_Type_OverChassisPower 4 //底盘超功率
#define Hurt_Type_Collision        5 //装甲撞击

/* 发射机构编号 */
#define Shooter_ID1_17mm 1 //1号17mm发射机构
#define Shooter_ID2_17mm 2 //2号17mm发射机构
#define Shooter_ID1_42mm 3 //1号42mm发射机构

/* 飞镖信息 */
#define Dart_State_Open     0 //飞镖闸门开启
#define Dart_State_Close    1 //飞镖闸门关闭
#define Dart_State_Changing 2 //正在开启或者关闭中
#define Dart_Target_Outpost 0 //飞镖目标为前哨站
#define Dart_Target_Base    1 //飞镖目标为基地

/* 操作手ID */
#define Cilent_ID_Red_Hero       0x0101 //红方英雄操作手
#define Cilent_ID_Red_Engineer   0x0102 //红方工程操作手
#define Cilent_ID_Red_Infantry3  0x0103 //红方步兵3操作手
#define Cilent_ID_Red_Infantry4  0x0104 //红方步兵4操作手
#define Cilent_ID_Red_Infantry5  0x0105 //红方步兵5操作手
#define Cilent_ID_Red_Aerial     0x0106 //红方飞手
#define Cilent_ID_Blue_Hero      0x0165 //蓝方英雄操作手
#define Cilent_ID_Blue_Engineer  0x0166 //蓝方工程操作手
#define Cilent_ID_Blue_Infantry3 0x0167 //蓝方步兵3操作手
#define Cilent_ID_Blue_Infantry4 0x0168 //蓝方步兵4操作手
#define Cilent_ID_Blue_Infantry5 0x0169 //蓝方步兵5操作手
#define Cilent_ID_Blue_Aerial    0x016A //蓝方飞手

/* UI绘制内容cmdID */
#define UI_DataID_Delete   0x100 //客户端删除图形
#define UI_DataID_Draw1    0x101 //客户端绘制1个图形
#define UI_DataID_Draw2    0x102 //客户端绘制2个图形
#define UI_DataID_Draw5    0x103 //客户端绘制5个图形
#define UI_DataID_Draw7    0x104 //客户端绘制7个图形
#define UI_DataID_DrawChar 0x110 //客户端绘制字符图形

/* 雷达，哨兵自主决策cmdID */
#define SENTRY_AUTO_SEND 		0x0120
#define LIDAR_AUTO_SEND 		0x0121

/* UI删除操作 */
#define UI_Delete_Invalid 0 //空操作
#define UI_Delete_Layer   1 //删除图层
#define UI_Delete_All     2 //删除所有

/* UI图形操作 */
#define UI_Graph_invalid 0 //空操作
#define UI_Graph_Add     1 //增加图形
#define UI_Graph_Change  2 //修改图形
#define UI_Graph_Delete  3 //删除图形

/* UI图形类型 */
#define UI_Graph_Line      0 //直线
#define UI_Graph_Rectangle 1 //矩形
#define UI_Graph_Circle    2 //整圆
#define UI_Graph_Ellipse   3 //椭圆
#define UI_Graph_Arc       4 //圆弧
#define UI_Graph_Float     5 //浮点型
#define UI_Graph_Int       6 //整形
#define UI_Graph_String    7 //字符型

/* UI图形颜色 */
#define UI_Color_Main   0 //红蓝主色
#define UI_Color_Yellow 1 //黄色
#define UI_Color_Green  2 //绿色
#define UI_Color_Orange 3 //橙色
#define UI_Color_Purple 4 //紫色
#define UI_Color_Pink   5 //粉色
#define UI_Color_Cyan   6 //青色
#define UI_Color_Black  7 //黑色
#define UI_Color_White  8 //白色

#pragma pack(push, 1)

/* 0x000X --------------------------------------------------------------------*/
typedef __packed struct  //0x0001 比赛状态数据
{
	uint8_t  game_type : 4;
	uint8_t  game_progress : 4;
	uint16_t stage_remain_time;
	uint64_t SyncTimeStamp;
} ext_game_status_t;

typedef __packed struct  //0x0002 比赛结果数据
{
	uint8_t winner;
} ext_game_result_t;

typedef __packed struct  //0x0003 机器人血量数据
{
	uint16_t red_1_robot_HP;
	uint16_t red_2_robot_HP;
	uint16_t red_3_robot_HP;
	uint16_t red_4_robot_HP;
	uint16_t reserved;
	uint16_t red_7_robot_HP;
	uint16_t red_outpost_HP;
	uint16_t red_base_HP;
	uint16_t blue_1_robot_HP;
	uint16_t blue_2_robot_HP;
	uint16_t blue_3_robot_HP;
	uint16_t blue_4_robot_HP;
	uint16_t reserved_1;
	uint16_t blue_7_robot_HP;
	uint16_t blue_outpost_HP;
	uint16_t blue_base_HP;
} ext_game_robot_HP_t;

/* 0x010X --------------------------------------------------------------------*/
typedef __packed struct  //0x0101 场地事件数据
{
  uint32_t event_type;
} ext_event_data_t;


typedef __packed struct  //0x0104 裁判警告信息
{
	uint8_t level;
	uint8_t foul_robot_id;
	uint8_t last_foul_robot_count;
} ext_referee_warning_t;

typedef __packed struct  //0x0105 飞镖发射口倒计时
{
	uint8_t dart_remaining_time;
	uint16_t dart_info;
} ext_dart_remaining_time_t;

/* 0x020X --------------------------------------------------------------------*/
typedef __packed struct  //0x0201 比赛机器人状态，重点关注电管输出状态
{
	uint8_t robot_id; 
  uint8_t robot_level; 
  uint16_t current_HP;  
  uint16_t maximum_HP; 
  uint16_t shooter_barrel_cooling_value; 
  uint16_t shooter_barrel_heat_limit; 
  uint16_t chassis_power_limit;  
  uint8_t power_management_gimbal_output : 1; 
  uint8_t power_management_chassis_output : 1;  
  uint8_t power_management_shooter_output : 1;
} ext_game_robot_state_t;

typedef __packed struct  //0x0202 实时功率热量数据
{
	uint16_t reserved;
	uint16_t reserved_1; 
  float reserved_2; 
  uint16_t buffer_energy; 
  uint16_t shooter_17mm_1_barrel_heat; 
  uint16_t shooter_17mm_2_barrel_heat; 
  uint16_t shooter_42mm_barrel_heat; 
} ext_power_heat_data_t;

typedef __packed struct  //0x0203 机器人位置
{
	float x;
	float y;
	float angle; // 测速模块朝向，正北为0
} ext_game_robot_pos_t;

typedef __packed struct  //0x0204 机器人增益
{
	uint8_t recovery_buff;
	uint8_t cooling_buff;
	uint8_t defence_buff;
	uint8_t vulnerability_buff;
	uint16_t attack_buff;
	uint8_t remaining_energy;  //重点关注
} ext_buff_musk_t;


typedef __packed struct  //0x0206 伤害状态//重点关注装甲模块被弹丸攻击导致扣血
{
	uint8_t armor_type : 4;
	uint8_t hurt_type  : 4;
} ext_robot_hurt_t;

typedef __packed struct  //0x0207 实时射击信息
{
	uint8_t bullet_type;
	uint8_t shooter_id;
	uint8_t bullet_freq;
	float   bullet_speed;
} ext_shoot_data_t;

typedef __packed struct  //0x0208 子弹剩余发射数
{
	uint16_t bullet_remaining_num_17mm;
	uint16_t bullet_remaining_num_42mm;
	uint16_t coin_remaining_num;
} ext_bullet_remaining_t;

typedef __packed struct  //0x0209 机器人RFID状态
{
	uint32_t rfid_status;
} ext_rfid_status_t;

typedef __packed struct  //0x020B 地面机器人位置
{
	float hero_x;  
  float hero_y;  
  float engineer_x;  
  float engineer_y;  
  float standard_3_x;  
  float standard_3_y;  
  float standard_4_x;  
  float standard_4_y;  
  float reserved_1;  
  float reserved_2;
} ext_ground_robot_position_t;

typedef __packed struct //0x020D 哨兵相关信息
{
	uint32_t sentry_info;
	uint16_t sentry_info_2;
} ext_sentry_info_t;


/* 0x030X --------------------------------------------------------------------*/
typedef __packed struct  //0x0301 机器人间通信 头结构体
{
	uint16_t data_cmd_id;
	uint16_t sender_ID;
	uint16_t receiver_ID;
} ext_student_interactive_header_data_t;

typedef __packed struct  //0x0301 机器人间通信 数据结构体
{
	uint8_t *data;
} robot_interactive_data_t;

typedef __packed struct  //0x0301 机器人间通信 哨兵自主决策指令
{
	uint32_t sentry_cmd_data;
} sentry_cmd_t;

typedef __packed struct  //0x0303 小地图下发信息标识
{
	float    target_position_x;
	float    target_position_y;
	float    target_position_z;
	uint8_t  commd_keyboard;
	uint16_t target_robot_ID;
} ext_robot_command_t;

typedef __packed struct  //0x0305 小地图接收信息标识
{
	uint16_t target_robot_ID;
	float    target_position_x;
	float    target_position_y;
} ext_client_map_command_t;

/* 自定义绘制UI结构体 -------------------------------------------------------*/
typedef __packed struct  //绘制UI UI图形数据
{
	uint8_t  graphic_name[3];
	uint32_t operate_tpye:3;
	uint32_t graphic_tpye:3;
	uint32_t layer:4;
	uint32_t color:4;
	uint32_t start_angle:9;
	uint32_t end_angle:9;
	uint32_t width:10;
	uint32_t start_x:11;
	uint32_t start_y:11;
	uint32_t radius:10;
	uint32_t end_x:11;
	uint32_t end_y:11;
} graphic_data_struct_t;

typedef __packed struct  //绘制UI UI字符串数据
{
	uint8_t  string_name[3];
	uint32_t operate_tpye:3;
	uint32_t graphic_tpye:3;
	uint32_t layer:4;
	uint32_t color:4;
	uint32_t start_angle:9;
	uint32_t end_angle:9;
	uint32_t width:10;
	uint32_t start_x:11;
	uint32_t start_y:11;
	uint32_t null;
	uint8_t stringdata[30];
} string_data_struct_t;

typedef __packed struct  //绘制UI UI删除图形数据
{
	uint8_t operate_tpye;
	uint8_t layer;
} delete_data_struct_t;

typedef __packed struct //绘制UI 绘制1个图形完整结构体
{
	frame_header_struct_t Referee_Transmit_Header;
	uint16_t CMD_ID;
	ext_student_interactive_header_data_t Interactive_Header;
	graphic_data_struct_t Graphic[1];
	uint16_t CRC16;
} UI_Graph1_t;

typedef __packed struct //绘制UI 绘制2个图形完整结构体
{
	frame_header_struct_t Referee_Transmit_Header;
	uint16_t CMD_ID;
	ext_student_interactive_header_data_t Interactive_Header;
	graphic_data_struct_t Graphic[2];
	uint16_t CRC16;
} UI_Graph2_t;

typedef __packed struct //绘制UI 绘制5个图形完整结构体
{
	frame_header_struct_t Referee_Transmit_Header;
	uint16_t CMD_ID;
	ext_student_interactive_header_data_t Interactive_Header;
	graphic_data_struct_t Graphic[5];
	uint16_t CRC16;
} UI_Graph5_t;

typedef __packed struct //绘制UI 绘制7个图形完整结构体
{
	frame_header_struct_t Referee_Transmit_Header;
	uint16_t CMD_ID;
	ext_student_interactive_header_data_t Interactive_Header;
	graphic_data_struct_t Graphic[7];
	uint16_t CRC16;
} UI_Graph7_t;

typedef __packed struct //绘制UI 绘制1字符串完整结构体
{ 
	frame_header_struct_t Referee_Transmit_Header;
	uint16_t CMD_ID;
	ext_student_interactive_header_data_t Interactive_Header;
	string_data_struct_t String;
	uint16_t CRC16;
} UI_String_t;

typedef __packed struct  //绘制UI UI删除图形完整结构体
{
	frame_header_struct_t Referee_Transmit_Header;
	uint16_t CMD_ID;
	ext_student_interactive_header_data_t Interactive_Header;
	delete_data_struct_t Delete;
	uint16_t CRC16;
} UI_Delete_t;

typedef __packed struct  //发送哨兵自主决策指令完整结构体
{
	frame_header_struct_t Referee_Transmit_Header;
	uint16_t CMD_ID;
	ext_student_interactive_header_data_t Interactive_Header;
	sentry_cmd_t sentry_cmd;
	uint16_t CRC16;
} Sentry_Auto_Cmd_Send_t;

#pragma pack(pop)
/* Structs -------------------------------------------------------------------*/
/* protocol包头结构体 */
extern frame_header_struct_t Referee_Receive_Header;

/* 0x000X */
extern ext_game_status_t   Game_Status;
extern ext_game_result_t   Game_Result;
extern ext_game_robot_HP_t Game_Robot_HP;
/* 0x010X */
extern ext_event_data_t                Event_Data;
extern ext_referee_warning_t           Referee_Warning;
extern ext_dart_remaining_time_t       Dart_Remaining_Time;

/* 0x020X */
extern ext_game_robot_state_t Game_Robot_State;
extern ext_power_heat_data_t  Power_Heat_Data;
extern ext_game_robot_pos_t   Game_Robot_Pos;
extern ext_buff_musk_t        Buff_Musk;
extern ext_robot_hurt_t       Robot_Hurt;
extern ext_shoot_data_t       Shoot_Data;
extern ext_bullet_remaining_t Bullet_Remaining;
extern ext_rfid_status_t      RFID_Status;
extern ext_ground_robot_position_t Ground_Robot_Position;
extern ext_sentry_info_t           Sentry_Info;

/* 0x030X */
extern ext_student_interactive_header_data_t Student_Interactive_Header_Data;
extern robot_interactive_data_t              Robot_Interactive_Data;
extern ext_robot_command_t                   Robot_Command;
extern ext_client_map_command_t              Client_Map_Command;

/* 绘制UI专用结构体 */
extern UI_Graph1_t UI_Graph1;
extern UI_Graph2_t UI_Graph2;
extern UI_Graph5_t UI_Graph5;
extern UI_Graph7_t UI_Graph7;
extern UI_String_t UI_String;
extern UI_Delete_t UI_Delete;

/* 哨兵专用结构体 */
extern Sentry_Auto_Cmd_Send_t Sentry_Auto_Cmd_Send;

/* Functions -----------------------------------------------------------------*/
void Referee_StructInit(void);
void Referee_UARTInit(uint8_t *Buffer0, uint8_t *Buffer1, uint16_t BufferLength);

void Referee_UnpackFifoData(unpack_data_t *p_obj, fifo_s_t *referee_fifo);
void Referee_SolveFifoData(uint8_t *frame);

void Sentry_PushUp_Cmd(Sentry_Auto_Cmd_Send_t *Sentry_Auto_Cmd , uint8_t RobotID);
void UI_Draw_Line(graphic_data_struct_t *Graph,        //UI图形数据结构体指针
	                char                   GraphName[3], //图形名 作为客户端的索引
									uint8_t                GraphOperate, //UI图形操作 对应UI_Graph_XXX的4种操作
									uint8_t                Layer,        //UI图形图层 [0,9]
									uint8_t                Color,        //UI图形颜色 对应UI_Color_XXX的9种颜色
									uint16_t               Width,        //线宽
									uint16_t               StartX,       //起始坐标X
									uint16_t               StartY,       //起始坐标Y
									uint16_t               EndX,         //截止坐标X
									uint16_t               EndY);        //截止坐标Y
void UI_Draw_Rectangle(graphic_data_struct_t *Graph,        //UI图形数据结构体指针
	                     char                   GraphName[3], //图形名 作为客户端的索引
									     uint8_t                GraphOperate, //UI图形操作 对应UI_Graph_XXX的4种操作
									     uint8_t                Layer,        //UI图形图层 [0,9]
							     	 	 uint8_t                Color,        //UI图形颜色 对应UI_Color_XXX的9种颜色
							     	   uint16_t               Width,        //线宽
							     		 uint16_t               StartX,       //起始坐标X
							     		 uint16_t               StartY,       //起始坐标Y
							     		 uint16_t               EndX,         //截止坐标X
							     		 uint16_t               EndY);        //截止坐标Y
void UI_Draw_Circle(graphic_data_struct_t *Graph,        //UI图形数据结构体指针
	                  char                   GraphName[3], //图形名 作为客户端的索引
									  uint8_t                GraphOperate, //UI图形操作 对应UI_Graph_XXX的4种操作
									  uint8_t                Layer,        //UI图形图层 [0,9]
							     	uint8_t                Color,        //UI图形颜色 对应UI_Color_XXX的9种颜色
										uint16_t               Width,        //线宽
										uint16_t               CenterX,      //圆心坐标X
							      uint16_t               CenterY,      //圆心坐标Y
										uint16_t               Radius);      //半径
void UI_Draw_Ellipse(graphic_data_struct_t *Graph,        //UI图形数据结构体指针
	                   char                   GraphName[3], //图形名 作为客户端的索引
									   uint8_t                GraphOperate, //UI图形操作 对应UI_Graph_XXX的4种操作
									   uint8_t                Layer,        //UI图形图层 [0,9]
							     	 uint8_t                Color,        //UI图形颜色 对应UI_Color_XXX的9种颜色
										 uint16_t               Width,        //线宽
										 uint16_t               CenterX,      //圆心坐标X
							       uint16_t               CenterY,      //圆心坐标Y
										 uint16_t               XHalfAxis,    //X半轴长
										 uint16_t               YHalfAxis);   //Y半轴长
void UI_Draw_Arc(graphic_data_struct_t *Graph,        //UI图形数据结构体指针
	               char                   GraphName[3], //图形名 作为客户端的索引
							   uint8_t                GraphOperate, //UI图形操作 对应UI_Graph_XXX的4种操作
								 uint8_t                Layer,        //UI图形图层 [0,9]
							   uint8_t                Color,        //UI图形颜色 对应UI_Color_XXX的9种颜色
								 uint16_t               StartAngle,   //起始角度 [0,360]
								 uint16_t               EndAngle,     //截止角度 [0,360]
								 uint16_t               Width,        //线宽
								 uint16_t               CenterX,      //圆心坐标X
							   uint16_t               CenterY,      //圆心坐标Y
								 uint16_t               XHalfAxis,    //X半轴长
								 uint16_t               YHalfAxis);   //Y半轴长
void UI_Draw_Float(graphic_data_struct_t *Graph,        //UI图形数据结构体指针
	                 char                   GraphName[3], //图形名 作为客户端的索引
							     uint8_t                GraphOperate, //UI图形操作 对应UI_Graph_XXX的4种操作
								   uint8_t                Layer,        //UI图形图层 [0,9]
							     uint8_t                Color,        //UI图形颜色 对应UI_Color_XXX的9种颜色
									 uint16_t               NumberSize,   //字体大小
									 uint16_t               Significant,  //有效位数
									 uint16_t               Width,        //线宽
							     uint16_t               StartX,       //起始坐标X
							     uint16_t               StartY,       //起始坐标Y
									 float                  FloatData);   //数字内容
void UI_Draw_Int(graphic_data_struct_t *Graph,        //UI图形数据结构体指针
	               char                   GraphName[3], //图形名 作为客户端的索引
							   uint8_t                GraphOperate, //UI图形操作 对应UI_Graph_XXX的4种操作
								 uint8_t                Layer,        //UI图形图层 [0,9]
							   uint8_t                Color,        //UI图形颜色 对应UI_Color_XXX的9种颜色
								 uint16_t               NumberSize,   //字体大小
								 uint16_t               Width,        //线宽
							   uint16_t               StartX,       //起始坐标X
							   uint16_t               StartY,       //起始坐标Y
								 int32_t                IntData);     //数字内容
void UI_Draw_String(string_data_struct_t *String,        //UI图形数据结构体指针
	                  char                  StringName[3], //图形名 作为客户端的索引
							      uint8_t               StringOperate, //UI图形操作 对应UI_Graph_XXX的4种操作
								    uint8_t               Layer,         //UI图形图层 [0,9]
							      uint8_t               Color,         //UI图形颜色 对应UI_Color_XXX的9种颜色
										uint16_t              CharSize,      //字体大小
									  uint16_t              StringLength,  //字符串长度
									  uint16_t              Width,         //线宽
							      uint16_t              StartX,        //起始坐标X
							      uint16_t              StartY,        //起始坐标Y
										char                 *StringData);   //字符串内容

void UI_PushUp_Graphs(uint8_t Counter, void *Graphs, uint8_t RobotID);
void UI_PushUp_String(UI_String_t *String, uint8_t RobotID);
void UI_PushUp_Delete(UI_Delete_t *Delete, uint8_t RobotID);


#ifdef __cplusplus
}
#endif

#endif /* __REFEREE_H__ */
