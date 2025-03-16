#ifndef __CILENT_UI__
#define __CILENT_UI__

#include "main.h"
#include "protocol.h"
/****************************内容 ID********************/
#define UI_Data_ID_Del 0x0100 
#define UI_Data_ID_Draw1 0x0101
#define UI_Data_ID_Draw2 0x0102
#define UI_Data_ID_Draw5 0x0103
#define UI_Data_ID_Draw7 0x0104
#define UI_Data_ID_DrawChar 0x110
/****************************红方机器人ID********************/
#define UI_Data_RobotID_RHero 1         
#define UI_Data_RobotID_REngineer 2
#define UI_Data_RobotID_RInfantry3 3
#define UI_Data_RobotID_RInfantry4 4
#define UI_Data_RobotID_RInfantry5 5
#define UI_Data_RobotID_RAerial 6
#define UI_Data_RobotID_RSentry 7
#define UI_Data_RobotID_RRadar 9
/****************************蓝方机器人ID********************/
#define UI_Data_RobotID_BHero 101
#define UI_Data_RobotID_BEngineer 102
#define UI_Data_RobotID_BInfantry3 103
#define UI_Data_RobotID_BInfantry4 104
#define UI_Data_RobotID_BInfantry5 105
#define UI_Data_RobotID_BAerial 106
#define UI_Data_RobotID_BSentry 107
#define UI_Data_RobotID_BRadar 109
/**************************红方客户端ID************************/
#define UI_Data_CilentID_RHero 0x0101
#define UI_Data_CilentID_REngineer 0x0102
#define UI_Data_CilentID_RInfantry3 0x0103
#define UI_Data_CilentID_RInfantry4 0x0104
#define UI_Data_CilentID_RInfantry5 0x0105
#define UI_Data_CilentID_RAerial 0x0106
/***************************蓝方客户端ID***********************/
#define UI_Data_CilentID_BHero 0x0165
#define UI_Data_CilentID_BEngineer 0x0166
#define UI_Data_CilentID_BInfantry3 0x0167
#define UI_Data_CilentID_BInfantry4 0x0168
#define UI_Data_CilentID_BInfantry5 0x0169
#define UI_Data_CilentID_BAerial 0x016A
/***************************删除操作***************************/
#define UI_Data_Del_NoOperate 0
#define UI_Data_Del_Layer 1
#define UI_Data_Del_ALL 2
/***************************图形操作********************/
#define UI_Graph_ADD 1
#define UI_Graph_Change 2
#define UI_Graph_Del 3
/***************************图形类型********************/
#define UI_Graph_Line 0         //直线
#define UI_Graph_Rectangle 1    //矩形
#define UI_Graph_Circle 2       //整圆
#define UI_Graph_Ellipse 3      //椭圆
#define UI_Graph_Arc 4          //圆弧
#define UI_Graph_Float 5        //浮点数
#define UI_Graph_Int 6          //整型数
#define UI_Graph_Char 7         //字符
/***************************颜色********************/
#define UI_Color_Main 0         //红蓝主色
#define UI_Color_Yellow 1
#define UI_Color_Green 2
#define UI_Color_Orange 3
#define UI_Color_Purplish_red 4 //紫红色
#define UI_Color_Pink 5
#define UI_Color_Cyan 6         //青色
#define UI_Color_Black 7
#define UI_Color_White 8

//自定义UI相关结构体
typedef __packed struct 
{
    uint16_t data_cmd_id;
		uint16_t send_id;
    uint16_t receiver_id;
} ext_student_interactive_header_data_t;

typedef __packed struct
{ 
uint8_t graphic_name[3]; 
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

typedef __packed struct
{ 
uint8_t graphic_name[3]; 
uint32_t operate_tpye:3; 
uint32_t graphic_tpye:3; 
uint32_t layer:4; 
uint32_t color:4; 
uint32_t start_angle:9;
uint32_t end_angle:9;
uint32_t width:10; 
uint32_t start_x:11; 
uint32_t start_y:11; 
float    num;
} float_data_struct_t;

typedef __packed struct
{ 
	graphic_data_struct_t Graph_Form;
	uint8_t data[30];
} ext_string_data_struct_t;

typedef __packed struct
{ 
		frame_header_struct_t send_frame_header;
		uint16_t cmd_id;
		ext_student_interactive_header_data_t interactive_header_data;
		graphic_data_struct_t grapic_data_struct[5];
		uint16_t CRC_16;
} ext_drawing_data_t;

typedef __packed struct
{ 
		frame_header_struct_t send_frame_header;
		uint16_t cmd_id;
		ext_student_interactive_header_data_t interactive_header_data;
		ext_string_data_struct_t string_data_struct_t;
		uint16_t CRC_16;
} ext_client_custom_character_t;


extern void Line_Draw(graphic_data_struct_t *image,uint8_t imagename[3],uint32_t Graph_Operate,uint32_t Graph_Layer,uint32_t Graph_Color,uint32_t Graph_Width,uint32_t Start_x,uint32_t Start_y,uint32_t End_x,uint32_t End_y);
extern void Char_Draw(ext_string_data_struct_t *image,uint8_t imagename[3],uint32_t Graph_Operate,uint32_t Graph_Layer,uint32_t Graph_Color,uint32_t Graph_Size,uint32_t Graph_Digit,uint32_t Graph_Width,uint32_t Start_x,uint32_t Start_y,char *Char_Data);
#endif

