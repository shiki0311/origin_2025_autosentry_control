/*******************************************************************************
 * @file       Cboard_To_Nuc_usbd_communication.c
 * @brief      向上位机NUC发送陀螺仪，裁判系统数据。解包上位机发下来的自瞄和导航数据
 * @note
 * @history
 *  Version    Date                 Author          Modification
 *  V1.0usbd   October-28-2024		captainwu		1.transform to usbd
 *  V2.0tim    2025-7               Shiki           2.从在freertos task发送数据改为使用定时器中断发送数据
 *****************************************************************************************/
#include "Cboard_To_Nuc_usbd_communication.h"
#include "arm_math.h"
#include "Shoot_Task.h"
#include "referee.h"
#include "remote_control.h"
#include "tim.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

uint8_t USBD_Buf[2][USBD_RX_BUF_LENGHT], NUC_USBD_RxBuf[USBD_RX_BUF_LENGHT], NUC_USBD_TxBuf[USBD_TX_BUF_LENGHT];
uint8_t yaw_rotate_flag_last = 0;
AutoAim_Data_Tx AutoAim_Data_Transmit;
AutoAim_Data_Rx AutoAim_Data_Receive;
Referee_Data_Tx Referee_Data_Tramsit;

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
uint8_t NUC_Data_Unpack(void)
{
	switch (NUC_USBD_RxBuf[2])
	{
	case CMD_ID_AUTOAIM_DATA_RX:
	{
		yaw_rotate_flag_last = AutoAim_Data_Receive.yaw_rotate_flag;
		memcpy(&AutoAim_Data_Receive, NUC_USBD_RxBuf + 3, sizeof(AutoAim_Data_Rx));
		break;
	}

	default:
		return 1;
	}

	return 0;
}
void NUC_USBD_Tx(uint8_t cmdid)
{
	Protocol_Head_Data Protocol_Head;
	Protocol_Head.Header = 0xAA;
	Protocol_Head.Cmd_ID = cmdid;
	switch (cmdid)
	{
	case CMD_ID_AUTOAIM_DATA_TX:
		Protocol_Head.Length = LENGTH_AUTOAIM_DATA_TX + 4;
		memcpy(NUC_USBD_TxBuf, (uint8_t *)(&Protocol_Head), 3);
		AutoAim_Data_Transmit.Pitch = INS_angle_deg[1];
		AutoAim_Data_Transmit.Roll = INS_angle_deg[2];
		AutoAim_Data_Transmit.Yaw = INS_angle_deg[0];

		memcpy(NUC_USBD_TxBuf + 3, (uint8_t *)(&AutoAim_Data_Transmit), LENGTH_AUTOAIM_DATA_TX);

		NUC_USBD_TxBuf[LENGTH_AUTOAIM_DATA_TX + 3] = CRC_Calculation(NUC_USBD_TxBuf, LENGTH_AUTOAIM_DATA_TX + 3);
		//			HAL_UART_Transmit_DMA(&huart1, Usart1_Dma_Txbuf, LENGTH_AUTOAIM_DATA_TX + 6-2+1);
		CDC_Transmit_FS(NUC_USBD_TxBuf, LENGTH_AUTOAIM_DATA_TX + 6 - 2);
		break;

	case CMD_ID_REFEREE_DATA_TX:
		Protocol_Head.Length = LENGTH_REFEREE_DATA_TX + 4;
		memcpy(NUC_USBD_TxBuf, (uint8_t *)(&Protocol_Head), 3);

		Referee_Data_Tramsit.remain_HP = Game_Robot_State.current_HP;
		Referee_Data_Tramsit.max_HP = Game_Robot_State.maximum_HP;
		Referee_Data_Tramsit.game_progress = Game_Status.game_progress;
		Referee_Data_Tramsit.stage_remain_time = Game_Status.stage_remain_time;
		Referee_Data_Tramsit.coin_remaining_num = Bullet_Remaining.coin_remaining_num;
		Referee_Data_Tramsit.bullet_remaining_num_17mm = Bullet_Remaining.bullet_remaining_num_17mm;

		Referee_Data_Tramsit.red_1_HP = 1000; // 这个变量不知道为什么上位机接收解算会错误，其他变量都可以正常传输通讯，所以直接传个定值了
		Referee_Data_Tramsit.red_2_HP = Game_Robot_HP.red_2_robot_HP;
		Referee_Data_Tramsit.red_3_HP = Game_Robot_HP.red_3_robot_HP;
		Referee_Data_Tramsit.red_4_HP = Game_Robot_HP.red_4_robot_HP;
		Referee_Data_Tramsit.red_7_HP = Game_Robot_HP.red_7_robot_HP;
		Referee_Data_Tramsit.red_outpost_HP = Game_Robot_HP.red_outpost_HP;
		Referee_Data_Tramsit.red_base_HP = Game_Robot_HP.red_base_HP;

		Referee_Data_Tramsit.blue_1_HP = Game_Robot_HP.blue_1_robot_HP;
		Referee_Data_Tramsit.blue_2_HP = Game_Robot_HP.blue_2_robot_HP;
		Referee_Data_Tramsit.blue_3_HP = Game_Robot_HP.blue_3_robot_HP;
		Referee_Data_Tramsit.blue_4_HP = Game_Robot_HP.blue_4_robot_HP;
		Referee_Data_Tramsit.blue_7_HP = Game_Robot_HP.blue_7_robot_HP;
		Referee_Data_Tramsit.blue_outpost_HP = Game_Robot_HP.blue_outpost_HP;
		Referee_Data_Tramsit.blue_base_HP = Game_Robot_HP.blue_base_HP;

		Referee_Data_Tramsit.rfid_status = RFID_Status.rfid_status;
		Referee_Data_Tramsit.event_data = Event_Data.event_type;
		Referee_Data_Tramsit.hurt_reason = Robot_Hurt.hurt_type;
		Referee_Data_Tramsit.enemy_hero_position = Student_Interactive_Data.enemy_hero_position_data;
		Referee_Data_Tramsit.defend_fortress = Student_Interactive_Data.check_defend_fortress;
		memcpy(NUC_USBD_TxBuf + 3, (uint8_t *)(&Referee_Data_Tramsit), LENGTH_REFEREE_DATA_TX);

		NUC_USBD_TxBuf[LENGTH_AUTOAIM_DATA_TX + 3] = CRC_Calculation(NUC_USBD_TxBuf, LENGTH_REFEREE_DATA_TX + 3);
		//				HAL_UART_Transmit_DMA(&huart1, Usart1_Dma_Txbuf, LENGTH_REFEREE_DATA_TX + 6-2);
		CDC_Transmit_FS(NUC_USBD_TxBuf, LENGTH_REFEREE_DATA_TX + 4);
		break;
	default:
		return;
	}
}

void NUC_TX_Autoaim(TIM_HandleTypeDef *htim)
{
	if (htim == &htim1)
	{
		// 5ms trigger
		NUC_USBD_Tx(CMD_ID_AUTOAIM_DATA_TX);
	}
}

void NUC_TX_Referee(TIM_HandleTypeDef *htim)
{
	if (htim == &htim8)
	{
		// 50ms trigger todo
		NUC_USBD_Tx(CMD_ID_REFEREE_DATA_TX);
	}
}

uint8_t USBD_IRQHandler(uint8_t *Buf, uint16_t Len)
{
	memcpy(NUC_USBD_RxBuf, Buf, Len);
	if (NUC_USBD_RxBuf[0] != 0xAA || NUC_USBD_RxBuf[1] > 128) // 帧头不匹配或者数据包长度错误，丢弃这一帧
		return 1;

	if (NUC_USBD_RxBuf[1] == Len) // 校验数据包长度
	{
		NUC_Data_Unpack(); // 解包NUC数据
		return 0;
	}
	else	
		return 1;
}

uint8_t CRC_Calculation(uint8_t *ptr, uint16_t len)
{
	uint8_t crc = 0xff;
	while (len--)
	{
		crc = CRC08_Table[crc ^ *ptr++];
	}
	return crc;
}
