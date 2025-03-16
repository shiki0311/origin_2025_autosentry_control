/**
  ******************************************************************************
  * @file    referee.c
  * @author  Karolance Future
  * @version V1.0.0
  * @date    2022/03/21
  * @brief   
  ******************************************************************************
  * @attention
	*   
  ******************************************************************************
  */

/* Private includes ----------------------------------------------------------*/
#include "referee.h"
#include "protocol.h"
#include "string.h"
#include "usart.h"
#include "fifo.h"
#include "crcs.h"

/* Private define ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
/* protocol��ͷ�ṹ�� */
frame_header_struct_t Referee_Receive_Header;

/* 0x000X */
ext_game_status_t   Game_Status;
ext_game_result_t   Game_Result;
ext_game_robot_HP_t Game_Robot_HP;

/* 0x010X */
ext_event_data_t                Event_Data;
ext_referee_warning_t           Referee_Warning;
ext_dart_remaining_time_t       Dart_Remaining_Time;

/* 0x020X */
ext_game_robot_state_t Game_Robot_State;
ext_power_heat_data_t  Power_Heat_Data;
ext_game_robot_pos_t   Game_Robot_Pos;
ext_buff_musk_t        Buff_Musk;
ext_robot_hurt_t       Robot_Hurt;
ext_shoot_data_t       Shoot_Data;
ext_bullet_remaining_t Bullet_Remaining;
ext_rfid_status_t      RFID_Status;
ext_ground_robot_position_t Ground_Robot_Position;

/* 0x030X */
ext_student_interactive_header_data_t Student_Interactive_Header_Data;
robot_interactive_data_t              Robot_Interactive_Data;
ext_robot_command_t                   Robot_Command;
ext_client_map_command_t              Client_Map_Command;

/* ����UIר�ýṹ�� */
UI_Graph1_t UI_Graph1;
UI_Graph2_t UI_Graph2;
UI_Graph5_t UI_Graph5;
UI_Graph7_t UI_Graph7;
UI_String_t UI_String;
UI_Delete_t UI_Delete;

/* Functions -----------------------------------------------------------------*/
/*==============================================================================
              ##### ����ϵͳ��ʼ������ #####
  ==============================================================================
    [..]  �ò����ṩ���º���:
		  (+) ����ϵͳ�ṹ���ʼ������ Referee_StructInit
			(+) ����ϵͳ���ڳ�ʼ������ Referee_UARTInit
*/
void Referee_StructInit(void)
{
	//��ͷ�ṹ���ʼ��
	memset(&Referee_Receive_Header,          0, sizeof(Referee_Receive_Header));
	/* 0x000X */
	memset(&Game_Status,                     0, sizeof(Game_Status));
	memset(&Game_Result,                     0, sizeof(Game_Result));
	memset(&Game_Robot_HP,                   0, sizeof(Game_Robot_HP));
	/* 0x010X */
	memset(&Event_Data,                      0, sizeof(Event_Data));
	memset(&Referee_Warning,                 0, sizeof(Referee_Warning));
	memset(&Dart_Remaining_Time,             0, sizeof(Dart_Remaining_Time));
	/* 0x020X */
	memset(&Game_Robot_State,                0, sizeof(Game_Robot_State));
	memset(&Power_Heat_Data,                 0, sizeof(Power_Heat_Data));
	memset(&Game_Robot_Pos,                  0, sizeof(Game_Robot_Pos));
	memset(&Buff_Musk,                       0, sizeof(Buff_Musk));
	memset(&Robot_Hurt,                      0, sizeof(Robot_Hurt));
	memset(&Shoot_Data,                      0, sizeof(Shoot_Data));
	memset(&Bullet_Remaining,                0, sizeof(Bullet_Remaining));
	memset(&RFID_Status,                     0, sizeof(RFID_Status));
	memset(&Ground_Robot_Position,           0, sizeof(Ground_Robot_Position));
	/* 0x030X */
	memset(&Student_Interactive_Header_Data, 0, sizeof(Student_Interactive_Header_Data));
	memset(&Robot_Interactive_Data,          0, sizeof(Robot_Interactive_Data));
	memset(&Robot_Command,                   0, sizeof(Robot_Command));
	memset(&Client_Map_Command,              0, sizeof(Client_Map_Command));
}

void Referee_UARTInit(uint8_t *Buffer0, uint8_t *Buffer1, uint16_t BufferLength)
{
	/* ʹ�ܴ���DMA */
	SET_BIT(Referee_UART.Instance->CR3, USART_CR3_DMAR);
	SET_BIT(Referee_UART.Instance->CR3, USART_CR3_DMAT);
	
	/* ʹ�ܴ��ڿ����ж� */
	__HAL_UART_ENABLE_IT(&Referee_UART, UART_IT_IDLE);
	
	/* ȷ��DMA RXʧ�� */
	while(Referee_UART.hdmarx->Instance->CR & DMA_SxCR_EN)
	{
		__HAL_DMA_DISABLE(Referee_UART.hdmarx);
	}
	
	/* ��ձ�־λ */
	__HAL_DMA_CLEAR_FLAG(Referee_UART.hdmarx, DMA_LISR_TCIF1);

	/* ���ý���˫������ */
	Referee_UART.hdmarx->Instance->PAR  = (uint32_t) & (Referee_UART.Instance->DR);
	Referee_UART.hdmarx->Instance->M0AR = (uint32_t)(Buffer0);
	Referee_UART.hdmarx->Instance->M1AR = (uint32_t)(Buffer1);
	
	/* �������ݳ��� */
	__HAL_DMA_SET_COUNTER(Referee_UART.hdmarx, BufferLength);
	
	/* ʹ��˫������ */
	SET_BIT(Referee_UART.hdmarx->Instance->CR, DMA_SxCR_DBM);
	
	/* ʹ��DMA RX */
	__HAL_DMA_ENABLE(Referee_UART.hdmarx);
	
	/* ȷ��DMA TXʧ�� */
	while(Referee_UART.hdmatx->Instance->CR & DMA_SxCR_EN)
	{
		__HAL_DMA_DISABLE(Referee_UART.hdmatx);
	}
	
	Referee_UART.hdmatx->Instance->PAR  = (uint32_t) & (Referee_UART.Instance->DR);
}

/*==============================================================================
              ##### ����ϵͳ���ݽ������� #####
  ==============================================================================
    [..]  �ò����ṩ���º���:
		  (+) ����ϵͳ�������ݽ�ѹ���� Referee_UnpackFifoData
      (+) ����ϵͳ�������ݴ����� Referee_SolveFifoData
*/
void Referee_UnpackFifoData(unpack_data_t *referee_unpack_obj, fifo_s_t *referee_fifo)
{
  uint8_t byte = 0;
  uint8_t sof  = HEADER_SOF;
	
  while(fifo_s_used(referee_fifo))
  {
    byte = fifo_s_get(referee_fifo);
    switch(referee_unpack_obj->unpack_step)
    {
      case STEP_HEADER_SOF:
      {
        if(byte == sof)
        {
          referee_unpack_obj->unpack_step = STEP_LENGTH_LOW;
          referee_unpack_obj->protocol_packet[referee_unpack_obj->index++] = byte;
        }
        else
        {
          referee_unpack_obj->index = 0;
        }
      }break;
      
      case STEP_LENGTH_LOW:
      {
        referee_unpack_obj->data_len = byte;
        referee_unpack_obj->protocol_packet[referee_unpack_obj->index++] = byte;
        referee_unpack_obj->unpack_step = STEP_LENGTH_HIGH;
      }break;
      
      case STEP_LENGTH_HIGH:
      {
        referee_unpack_obj->data_len |= (byte << 8);
        referee_unpack_obj->protocol_packet[referee_unpack_obj->index++] = byte;
        if(referee_unpack_obj->data_len < (REF_PROTOCOL_FRAME_MAX_SIZE - REF_HEADER_CRC_CMDID_LEN))
        {
          referee_unpack_obj->unpack_step = STEP_FRAME_SEQ;
        }
        else
        {
          referee_unpack_obj->unpack_step = STEP_HEADER_SOF;
          referee_unpack_obj->index = 0;
        }
      }break;
			
      case STEP_FRAME_SEQ:
      {
        referee_unpack_obj->protocol_packet[referee_unpack_obj->index++] = byte;
        referee_unpack_obj->unpack_step = STEP_HEADER_CRC8;
      }break;
			
      case STEP_HEADER_CRC8:
      {
        referee_unpack_obj->protocol_packet[referee_unpack_obj->index++] = byte;
        if(referee_unpack_obj->index == REF_PROTOCOL_HEADER_SIZE)
        {
          if(CRC08_Verify(referee_unpack_obj->protocol_packet, REF_PROTOCOL_HEADER_SIZE))
          {
            referee_unpack_obj->unpack_step = STEP_DATA_CRC16;
          }
          else
          {
            referee_unpack_obj->unpack_step = STEP_HEADER_SOF;
            referee_unpack_obj->index = 0;
          }
        }
      }break;  
      
      case STEP_DATA_CRC16:
      {
        if(referee_unpack_obj->index <  (REF_HEADER_CRC_CMDID_LEN + referee_unpack_obj->data_len))
        {
           referee_unpack_obj->protocol_packet[referee_unpack_obj->index++] = byte;  
        }
        if(referee_unpack_obj->index >= (REF_HEADER_CRC_CMDID_LEN + referee_unpack_obj->data_len))
        {
          referee_unpack_obj->unpack_step = STEP_HEADER_SOF;
          referee_unpack_obj->index = 0;
          if(CRC16_Verify(referee_unpack_obj->protocol_packet, REF_HEADER_CRC_CMDID_LEN + referee_unpack_obj->data_len))
          {
            Referee_SolveFifoData(referee_unpack_obj->protocol_packet);
          }
        }
      }break;

      default:
      {
        referee_unpack_obj->unpack_step = STEP_HEADER_SOF;
        referee_unpack_obj->index = 0;
      }break;
    }
  }
}

void Referee_SolveFifoData(uint8_t *frame)
{
	uint16_t cmd_id = 0;
	uint8_t  index  = 0;
	
	memcpy(&Referee_Receive_Header, frame, sizeof(frame_header_struct_t));
	index += sizeof(frame_header_struct_t);
	memcpy(&cmd_id, frame + index, sizeof(uint16_t));
	index += sizeof(uint16_t);
	
	switch(cmd_id)
	{
		case GAME_STATE_CMD_ID:      	         memcpy(&Game_Status,               frame + index, sizeof(ext_game_status_t));               break;
		case GAME_RESULT_CMD_ID:               memcpy(&Game_Result,               frame + index, sizeof(ext_game_result_t));               break;
		case GAME_ROBOT_HP_CMD_ID:             memcpy(&Game_Robot_HP,             frame + index, sizeof(ext_game_robot_HP_t));             break;		
    case FIELD_EVENTS_CMD_ID:              memcpy(&Event_Data,                frame + index, sizeof(ext_event_data_t));                break;
		case REFEREE_WARNING_CMD_ID:           memcpy(&Referee_Warning,           frame + index, sizeof(ext_referee_warning_t));           break;
		case DART_REMAINING_TIME_CMD_ID:       memcpy(&Dart_Remaining_Time,       frame + index, sizeof(ext_dart_remaining_time_t));       break;
		
		case ROBOT_STATE_CMD_ID:               memcpy(&Game_Robot_State,          frame + index, sizeof(ext_game_robot_state_t));          break;
		case POWER_HEAT_DATA_CMD_ID:           memcpy(&Power_Heat_Data,           frame + index, sizeof(ext_power_heat_data_t));           break;
		case ROBOT_POS_CMD_ID:                 memcpy(&Game_Robot_Pos,            frame + index, sizeof(ext_game_robot_pos_t));            break;
		case BUFF_MUSK_CMD_ID:                 memcpy(&Buff_Musk,                 frame + index, sizeof(ext_buff_musk_t));                 break;
		case ROBOT_HURT_CMD_ID:                memcpy(&Robot_Hurt,                frame + index, sizeof(ext_robot_hurt_t));                break;
		case SHOOT_DATA_CMD_ID:                memcpy(&Shoot_Data,                frame + index, sizeof(ext_shoot_data_t));                break;
		case BULLET_REMAINING_CMD_ID:          memcpy(&Bullet_Remaining,          frame + index, sizeof(ext_bullet_remaining_t));          break;
		case ROBOT_RFID_STATE_CMD_ID:          memcpy(&RFID_Status,               frame + index, sizeof(ext_rfid_status_t));               break;
		
		case STUDENT_INTERACTIVE_DATA_CMD_ID:  memcpy(&Robot_Interactive_Data,    frame + index, sizeof(robot_interactive_data_t));        break;
		case ROBOT_COMMAND_CMD_ID:             memcpy(&Robot_Command,             frame + index, sizeof(ext_robot_command_t));             break;
		case CLIENT_MAP_COMMAND_CMD_ID:        memcpy(&Client_Map_Command,        frame + index, sizeof(ext_client_map_command_t));        break;
		
		default:                                                                                                                           break;
	}
}

/*==============================================================================
              ##### UI����ͼ�λ��ƺ��� #####
  ==============================================================================
    [..]  �ò����ṩ���º���:
		  (+) ����ֱ�� UI_Draw_Line
      (+) ���ƾ��� UI_Draw_Rectangle
      (+) ������Բ UI_Draw_Circle
      (+) ������Բ UI_Draw_Ellipse
      (+) ����Բ�� UI_Draw_Arc
      (+) ����С�� UI_Draw_Float
      (+) �������� UI_Draw_Int
      (+) �����ַ� UI_Draw_String
*/
void UI_Draw_Line(graphic_data_struct_t *Graph,        //UIͼ�����ݽṹ��ָ��
	                char                   GraphName[3], //ͼ���� ��Ϊ�ͻ��˵�����
									uint8_t                GraphOperate, //UIͼ�β��� ��ӦUI_Graph_XXX��4�ֲ���
									uint8_t                Layer,        //UIͼ��ͼ�� [0,9]
									uint8_t                Color,        //UIͼ����ɫ ��ӦUI_Color_XXX��9����ɫ
									uint16_t               Width,        //�߿�
									uint16_t               StartX,       //��ʼ����X
									uint16_t               StartY,       //��ʼ����Y
									uint16_t               EndX,         //��ֹ����X
									uint16_t               EndY)         //��ֹ����Y
{
	Graph->graphic_name[0] = GraphName[0];
	Graph->graphic_name[1] = GraphName[1];
	Graph->graphic_name[2] = GraphName[2];
	Graph->operate_tpye    = GraphOperate;
	Graph->graphic_tpye    = UI_Graph_Line;
	Graph->layer           = Layer;
	Graph->color           = Color;
	Graph->width           = Width;
	Graph->start_x         = StartX;
	Graph->start_y         = StartY;
	Graph->end_x           = EndX;
	Graph->end_y           = EndY;
}

void UI_Draw_Rectangle(graphic_data_struct_t *Graph,        //UIͼ�����ݽṹ��ָ��
	                     char                   GraphName[3], //ͼ���� ��Ϊ�ͻ��˵�����
									     uint8_t                GraphOperate, //UIͼ�β��� ��ӦUI_Graph_XXX��4�ֲ���
									     uint8_t                Layer,        //UIͼ��ͼ�� [0,9]
							     	 	 uint8_t                Color,        //UIͼ����ɫ ��ӦUI_Color_XXX��9����ɫ
							     	   uint16_t               Width,        //�߿�
							     		 uint16_t               StartX,       //��ʼ����X
							     		 uint16_t               StartY,       //��ʼ����Y
							     		 uint16_t               EndX,         //��ֹ����X
							     		 uint16_t               EndY)         //��ֹ����Y
{
	Graph->graphic_name[0] = GraphName[0];
	Graph->graphic_name[1] = GraphName[1];
	Graph->graphic_name[2] = GraphName[2];
	Graph->operate_tpye    = GraphOperate;
	Graph->graphic_tpye    = UI_Graph_Rectangle;
	Graph->layer           = Layer;
	Graph->color           = Color;
	Graph->width           = Width;
	Graph->start_x         = StartX;
	Graph->start_y         = StartY;
	Graph->end_x           = EndX;
	Graph->end_y           = EndY;
}

void UI_Draw_Circle(graphic_data_struct_t *Graph,        //UIͼ�����ݽṹ��ָ��
	                  char                   GraphName[3], //ͼ���� ��Ϊ�ͻ��˵�����
									  uint8_t                GraphOperate, //UIͼ�β��� ��ӦUI_Graph_XXX��4�ֲ���
									  uint8_t                Layer,        //UIͼ��ͼ�� [0,9]
							     	uint8_t                Color,        //UIͼ����ɫ ��ӦUI_Color_XXX��9����ɫ
										uint16_t               Width,        //�߿�
										uint16_t               CenterX,      //Բ������X
							      uint16_t               CenterY,      //Բ������Y
										uint16_t               Radius)       //�뾶
{
	Graph->graphic_name[0] = GraphName[0];
	Graph->graphic_name[1] = GraphName[1];
	Graph->graphic_name[2] = GraphName[2];
	Graph->operate_tpye    = GraphOperate;
	Graph->graphic_tpye    = UI_Graph_Circle;
	Graph->layer           = Layer;
	Graph->color           = Color;
	Graph->width           = Width;
	Graph->start_x         = CenterX;
	Graph->start_y         = CenterY;
	Graph->radius          = Radius;
}

void UI_Draw_Ellipse(graphic_data_struct_t *Graph,        //UIͼ�����ݽṹ��ָ��
	                   char                   GraphName[3], //ͼ���� ��Ϊ�ͻ��˵�����
									   uint8_t                GraphOperate, //UIͼ�β��� ��ӦUI_Graph_XXX��4�ֲ���
									   uint8_t                Layer,        //UIͼ��ͼ�� [0,9]
							     	 uint8_t                Color,        //UIͼ����ɫ ��ӦUI_Color_XXX��9����ɫ
										 uint16_t               Width,        //�߿�
										 uint16_t               CenterX,      //Բ������X
							       uint16_t               CenterY,      //Բ������Y
										 uint16_t               XHalfAxis,    //X���᳤
										 uint16_t               YHalfAxis)    //Y���᳤
{
	Graph->graphic_name[0] = GraphName[0];
	Graph->graphic_name[1] = GraphName[1];
	Graph->graphic_name[2] = GraphName[2];
	Graph->operate_tpye    = GraphOperate;
	Graph->graphic_tpye    = UI_Graph_Ellipse;
	Graph->layer           = Layer;
	Graph->color           = Color;
	Graph->width           = Width;
	Graph->start_x         = CenterX;
	Graph->start_y         = CenterY;
	Graph->end_x           = XHalfAxis;
	Graph->end_y           = YHalfAxis;
}

void UI_Draw_Arc(graphic_data_struct_t *Graph,        //UIͼ�����ݽṹ��ָ��
	               char                   GraphName[3], //ͼ���� ��Ϊ�ͻ��˵�����
							   uint8_t                GraphOperate, //UIͼ�β��� ��ӦUI_Graph_XXX��4�ֲ���
								 uint8_t                Layer,        //UIͼ��ͼ�� [0,9]
							   uint8_t                Color,        //UIͼ����ɫ ��ӦUI_Color_XXX��9����ɫ
								 uint16_t               StartAngle,   //��ʼ�Ƕ� [0,360]
								 uint16_t               EndAngle,     //��ֹ�Ƕ� [0,360]
								 uint16_t               Width,        //�߿�
								 uint16_t               CenterX,      //Բ������X
							   uint16_t               CenterY,      //Բ������Y
								 uint16_t               XHalfAxis,    //X���᳤
								 uint16_t               YHalfAxis)    //Y���᳤
{
	Graph->graphic_name[0] = GraphName[0];
	Graph->graphic_name[1] = GraphName[1];
	Graph->graphic_name[2] = GraphName[2];
	Graph->operate_tpye    = GraphOperate;
	Graph->graphic_tpye    = UI_Graph_Arc;
	Graph->layer           = Layer;
	Graph->color           = Color;
	Graph->start_angle     = StartAngle;
	Graph->end_angle       = EndAngle;
	Graph->width           = Width;
	Graph->start_x         = CenterX;
	Graph->start_y         = CenterY;
	Graph->end_x           = XHalfAxis;
	Graph->end_y           = YHalfAxis;
}

void UI_Draw_Float(graphic_data_struct_t *Graph,        //UIͼ�����ݽṹ��ָ��
	                 char                   GraphName[3], //ͼ���� ��Ϊ�ͻ��˵�����
							     uint8_t                GraphOperate, //UIͼ�β��� ��ӦUI_Graph_XXX��4�ֲ���
								   uint8_t                Layer,        //UIͼ��ͼ�� [0,9]
							     uint8_t                Color,        //UIͼ����ɫ ��ӦUI_Color_XXX��9����ɫ
									 uint16_t               NumberSize,   //�����С
									 uint16_t               Significant,  //��Чλ��
									 uint16_t               Width,        //�߿�
							     uint16_t               StartX,       //��ʼ����X
							     uint16_t               StartY,       //��ʼ����Y
									 float                  FloatData)    //��������
{
	Graph->graphic_name[0] = GraphName[0];
	Graph->graphic_name[1] = GraphName[1];
	Graph->graphic_name[2] = GraphName[2];
	Graph->operate_tpye    = GraphOperate;
	Graph->graphic_tpye    = UI_Graph_Float;
	Graph->layer           = Layer;
	Graph->color           = Color;
	Graph->start_angle     = NumberSize;
	Graph->end_angle       = Significant;
	Graph->width           = Width;
	Graph->start_x         = StartX;
	Graph->start_y         = StartY;
	int32_t IntData = FloatData * 1000;
	Graph->radius          = (IntData & 0x000003ff) >>  0;
	Graph->end_x           = (IntData & 0x001ffc00) >> 10;
	Graph->end_y           = (IntData & 0xffe00000) >> 21;
}

void UI_Draw_Int(graphic_data_struct_t *Graph,        //UIͼ�����ݽṹ��ָ��
	               char                   GraphName[3], //ͼ���� ��Ϊ�ͻ��˵�����
							   uint8_t                GraphOperate, //UIͼ�β��� ��ӦUI_Graph_XXX��4�ֲ���
								 uint8_t                Layer,        //UIͼ��ͼ�� [0,9]
							   uint8_t                Color,        //UIͼ����ɫ ��ӦUI_Color_XXX��9����ɫ
								 uint16_t               NumberSize,   //�����С
								 uint16_t               Width,        //�߿�
							   uint16_t               StartX,       //��ʼ����X
							   uint16_t               StartY,       //��ʼ����Y
								 int32_t                IntData)      //��������
{
	Graph->graphic_name[0] = GraphName[0];
	Graph->graphic_name[1] = GraphName[1];
	Graph->graphic_name[2] = GraphName[2];
	Graph->operate_tpye    = GraphOperate;
	Graph->graphic_tpye    = UI_Graph_Int;
	Graph->layer           = Layer;
	Graph->color           = Color;
	Graph->start_angle     = NumberSize;
	Graph->width           = Width;
	Graph->start_x         = StartX;
	Graph->start_y         = StartY;
	Graph->radius          = (IntData & 0x000003ff) >>  0;
	Graph->end_x           = (IntData & 0x001ffc00) >> 10;
	Graph->end_y           = (IntData & 0xffe00000) >> 21;
}

void UI_Draw_String(string_data_struct_t *String,        //UIͼ�����ݽṹ��ָ��
	                  char                  StringName[3], //ͼ���� ��Ϊ�ͻ��˵�����
							      uint8_t               StringOperate, //UIͼ�β��� ��ӦUI_Graph_XXX��4�ֲ���
								    uint8_t               Layer,         //UIͼ��ͼ�� [0,9]
							      uint8_t               Color,         //UIͼ����ɫ ��ӦUI_Color_XXX��9����ɫ
										uint16_t              CharSize,      //�����С
									  uint16_t              StringLength,  //�ַ�������
									  uint16_t              Width,         //�߿�
							      uint16_t              StartX,        //��ʼ����X
							      uint16_t              StartY,        //��ʼ����Y
										char                 *StringData)    //�ַ�������
{
	String->string_name[0] = StringName[0];
	String->string_name[1] = StringName[1];
	String->string_name[2] = StringName[2];
	String->operate_tpye   = StringOperate;
	String->graphic_tpye   = UI_Graph_String;
	String->layer          = Layer;
	String->color          = Color;
	String->start_angle    = CharSize;
	String->end_angle      = StringLength;
	String->width          = Width;
	String->start_x        = StartX;
	String->start_y        = StartY;
	for(int i = 0; i < StringLength; i ++) String->stringdata[i] = *StringData ++;
}

/*==============================================================================
              ##### UI����ͼ�����ͺ��� #####
  ==============================================================================
    [..]  �ò����ṩ���º���:
		  (+) ����ͼ�� UI_PushUp_Graphs
			(+) �����ַ� UI_PushUp_String
			(+) ɾ��ͼ�� UI_PushUp_Delete
*/
void UI_PushUp_Graphs(uint8_t Counter /* 1,2,5,7 */, void *Graphs /* ��Counter��һ�µ�UI_Graphx�ṹ��ͷָ�� */, uint8_t RobotID)
{
	UI_Graph1_t *Graph = (UI_Graph1_t *)Graphs; //����ֻ��һ������ͼ��
	
	/* ��� frame_header */
	Graph->Referee_Transmit_Header.SOF  = HEADER_SOF;
	     if(Counter == 1) Graph->Referee_Transmit_Header.data_length = 6 + 1 * 15;
	else if(Counter == 2) Graph->Referee_Transmit_Header.data_length = 6 + 2 * 15;
	else if(Counter == 5) Graph->Referee_Transmit_Header.data_length = 6 + 5 * 15;
	else if(Counter == 7) Graph->Referee_Transmit_Header.data_length = 6 + 7 * 15;
	Graph->Referee_Transmit_Header.seq  = Graph->Referee_Transmit_Header.seq + 1;
	Graph->Referee_Transmit_Header.CRC8 = CRC08_Calculate((uint8_t *)(&Graph->Referee_Transmit_Header), 4);
	
	/* ��� cmd_id */
	Graph->CMD_ID = STUDENT_INTERACTIVE_DATA_CMD_ID;
	
	/* ��� student_interactive_header */
	     if(Counter == 1) Graph->Interactive_Header.data_cmd_id = UI_DataID_Draw1;
	else if(Counter == 2) Graph->Interactive_Header.data_cmd_id = UI_DataID_Draw2;
	else if(Counter == 5) Graph->Interactive_Header.data_cmd_id = UI_DataID_Draw5;
	else if(Counter == 7) Graph->Interactive_Header.data_cmd_id = UI_DataID_Draw7;
	Graph->Interactive_Header.sender_ID   = RobotID ;      //��ǰ������ID
	Graph->Interactive_Header.receiver_ID = RobotID + 256; //��Ӧ������ID
	
	/* ��� frame_tail ��CRC16 */
	     if(Counter == 1)
	{
		UI_Graph1_t *Graph1 = (UI_Graph1_t *)Graphs;
		Graph1->CRC16 = CRC16_Calculate((uint8_t *)Graph1, sizeof(UI_Graph1_t) - 2);
	}
	else if(Counter == 2)
	{
		UI_Graph2_t *Graph2 = (UI_Graph2_t *)Graphs;
		Graph2->CRC16 = CRC16_Calculate((uint8_t *)Graph2, sizeof(UI_Graph2_t) - 2);
	}
	else if(Counter == 5)
	{
		UI_Graph5_t *Graph5 = (UI_Graph5_t *)Graphs;
		Graph5->CRC16 = CRC16_Calculate((uint8_t *)Graph5, sizeof(UI_Graph5_t) - 2);
	}
	else if(Counter == 7)
	{
		UI_Graph7_t *Graph7 = (UI_Graph7_t *)Graphs;
		Graph7->CRC16 = CRC16_Calculate((uint8_t *)Graph7, sizeof(UI_Graph7_t) - 2);
	}
	
	/* ʹ�ô���PushUp������ϵͳ */
	     if(Counter == 1) HAL_UART_Transmit_DMA(&Referee_UART, (uint8_t *)Graph, sizeof(UI_Graph1_t));
	else if(Counter == 2) HAL_UART_Transmit_DMA(&Referee_UART, (uint8_t *)Graph, sizeof(UI_Graph2_t));
	else if(Counter == 5) HAL_UART_Transmit_DMA(&Referee_UART, (uint8_t *)Graph, sizeof(UI_Graph5_t));
	else if(Counter == 7) HAL_UART_Transmit_DMA(&Referee_UART, (uint8_t *)Graph, sizeof(UI_Graph7_t));
}

void UI_PushUp_String(UI_String_t *String, uint8_t RobotID)
{
	/* ��� frame_header */
	String->Referee_Transmit_Header.SOF  = HEADER_SOF;
	String->Referee_Transmit_Header.data_length = 6 + 45;
	String->Referee_Transmit_Header.seq  = String->Referee_Transmit_Header.seq + 1;
	String->Referee_Transmit_Header.CRC8 = CRC08_Calculate((uint8_t *)(&String->Referee_Transmit_Header), 4);
	
	/* ��� cmd_id */
	String->CMD_ID = STUDENT_INTERACTIVE_DATA_CMD_ID;
	
	/* ��� student_interactive_header */
	String->Interactive_Header.data_cmd_id = UI_DataID_DrawChar;
	String->Interactive_Header.sender_ID   = RobotID ;      //��ǰ������ID
	String->Interactive_Header.receiver_ID = RobotID + 256; //��Ӧ������ID
	
	/* ��� frame_tail ��CRC16 */
	String->CRC16 = CRC16_Calculate((uint8_t *)String, sizeof(UI_String_t) - 2);
	
	/* ʹ�ô���PushUp������ϵͳ */
	HAL_UART_Transmit_DMA(&Referee_UART, (uint8_t *)String, sizeof(UI_String_t));
}

void UI_PushUp_Delete(UI_Delete_t *Delete, uint8_t RobotID)
{
	/* ��� frame_header */
	Delete->Referee_Transmit_Header.SOF  = HEADER_SOF;
	Delete->Referee_Transmit_Header.data_length = 6 + 2;
	Delete->Referee_Transmit_Header.seq  = Delete->Referee_Transmit_Header.seq + 1;
	Delete->Referee_Transmit_Header.CRC8 = CRC08_Calculate((uint8_t *)(&Delete->Referee_Transmit_Header), 4);
	
	/* ��� cmd_id */
	Delete->CMD_ID = STUDENT_INTERACTIVE_DATA_CMD_ID;
	
	/* ��� student_interactive_header */
	Delete->Interactive_Header.data_cmd_id = UI_DataID_Delete;
	Delete->Interactive_Header.sender_ID   = RobotID ;      //��ǰ������ID
	Delete->Interactive_Header.receiver_ID = RobotID + 256; //��Ӧ������ID
	
	/* ��� frame_tail ��CRC16 */
	Delete->CRC16 = CRC16_Calculate((uint8_t *)Delete, sizeof(UI_Delete_t) - 2);
	
	/* ʹ�ô���PushUp������ϵͳ */
	HAL_UART_Transmit_DMA(&Referee_UART, (uint8_t *)Delete, sizeof(UI_Delete_t));
}
