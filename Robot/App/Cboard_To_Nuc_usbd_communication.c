/**
  *****************************************************************************
  * @file       Nmanifold_usbd_task.c/h
  * @brief      MANIFOLD data solve. MANIFOLDï¿½ï¿½ï¿½Ý´ï¿½ï¿½ï¿½
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V2.0.0			Mar-3-2023			Ê¥ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½					1.Done
  *  V2.1.0			May-16-2023			Ê¥ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½					1.Add data transmission for dial switch
  *  V1.0usbd  		October-28-2024		captainwu				1.transform to usbd
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  *****************************************************************************
*/
/*****************************************************************************************
			ï¿½ï¿½Cï¿½å¡ªï¿½ï¿½NUCï¿½ï¿½ï¿½ï¿½Í¨ï¿½ï¿½Ð­ï¿½é£¨ï¿½ï¿½ï¿½ï¿½æ±¾ï¿½ï¿½Cï¿½ï¿½×²ï¿½Ê¹ï¿½Ã²ï¿½ï¿½ï¿??
ï¿½ï¿½
ï¿½ï¿½
ï¿½ï¿½
ï¿½Ü½ï¿½ï¿½ï¿½Ô´ï¿½Ä¼ï¿½ï¿½ï¿½ï¿½ëµ½ï¿½ï¿½ï¿½ï¿½ï¿½Ð£ï¿½Í¬Ê±È·ï¿½ï¿½Í·ï¿½Ä¼ï¿½ï¿½Ú¹ï¿½ï¿½ÌµÄ°ï¿½ï¿½ï¿½Ä¿Â¼ï¿½Ð¡ï¿½
ï¿½ï¿½ï¿½ï¿½stm32f4xx_it.cï¿½ï¿½USART1_IRQHandler_1ï¿½ï¿½ï¿½ï¿½ï¿½Ðµï¿½ï¿½ï¿½USART1_IRQHandler_1ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
ï¿½ï¿½Ê¹ï¿½ï¿½xTaskCreateï¿½ï¿½osThreadDef+osThreadCreateï¿½ï¿½ï¿½ï¿½manifold_usart_taskï¿½ï¿½FreeRTOSï¿½ï¿½ï¿½ï¿½
ï¿½ß¼ï¿½ï¿??ï¿½ï¿½ï¿½Õ·ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ðµï¿½Ö¡Í·ï¿½ï¿½Ö¡ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Öºï¿½CRCÐ£ï¿½ï¿½Î»ï¿½Ç·ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
ï¿½ï¿½ï¿½ï¿½Ò»ï¿½Ðµ×²ï¿½ï¿½ï¿½ï¿½Ã¾ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Â¼ï¿½ï¿½ï¿½ï¿½ï¿½AutoAim_Data_Receiveï¿½á¹¹ï¿½ï¿½ï¿½Ú»ï¿½È¡ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ý¡ï¿??
*****************************************************************************************/
#include "Cboard_To_Nuc_usbd_communication.h"
#include "arm_math.h"
#include "Shoot_Task.h"
#include "referee.h"
#include "remote_control.h"
#include "tim.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

uint8_t RX_Lenth_Total = 0;
uint8_t USBD_Buf[2][USBD_RX_BUF_LENGHT], NUC_USBD_RxBuf[USBD_RX_BUF_LENGHT], NUC_USBD_TxBuf[USBD_TX_BUF_LENGHT];

AutoAim_Data_Tx AutoAim_Data_Transmit;
AutoAim_Data_Rx AutoAim_Data_Receive;
Referee_Data_Tx Referee_Data_Tramsit;

/**
 * @description: ¶¨Ê±Æ÷¶¨Ê±ÖÜÆÚÍê³ÉÊ±µ÷ÓÃµÄ»Øµ÷º¯Êý£¬TIM1ÓÃÓÚÃ¿5msÏòNUC·¢ËÍÒ»´ÎÍÓÂÝÒÇÊý¾Ý£¬TIM8ÓÃÓÚÃ¿100msÏòNUC·¢ËÍÒ»´Î²ÃÅÐÏµÍ³Êý¾Ý£¬ÔÚstm32f4xx_it.cÖÐµÄtimÖÐ¶Ï´¦Àíº¯ÊýÖÐµ÷ÓÃ
 * @return ÎÞ
 * @param {TIM_HandleTypeDef} *htim
 */
void NUC_TX_IRQCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim1)
	{
		// 5ms trigger
		NUC_USBD_Tx(CMD_ID_AUTOAIM_DATA_TX);
	}
	else if (htim == &htim8)
	{
		// 100ms trigger
		HAL_TIM_Base_Stop_IT(&htim1);
		NUC_USBD_Tx(CMD_ID_REFEREE_DATA_TX);
		HAL_TIM_Base_Start_IT(&htim1);
	}
}

uint8_t USBD_IRQHandler(uint8_t *Buf, uint16_t Len)
{
	memcpy(NUC_USBD_RxBuf + RX_Lenth_Total, Buf, Len);
	if (NUC_USBD_RxBuf[0] != 0xAA || NUC_USBD_RxBuf[1] > 128) // ï¿½Ò²ï¿½ï¿½Ç±ï¿½Ê¶ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ê¶ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½0xAAï¿½Í·ï¿½ï¿½ï¿½1
		return 1;
	RX_Lenth_Total = Len + RX_Lenth_Total;

	if (NUC_USBD_RxBuf[1] == RX_Lenth_Total) // ï¿½ï¿½ï¿½ï¿½Ð£ï¿½é£¬ï¿½ï¿½Ð£ï¿½ï¿½ß½ï¿½ï¿½ï¿??
	{
		NUC_Data_Unpack(); // ï¿½ï¿½ï¿½Ý½ï¿½ï¿??
		//				CDC_Transmit_FS(NUC_USBD_RxBuf, RX_Lenth_Total);
		RX_Lenth_Total = 0;
	}

	if (NUC_USBD_RxBuf[1] < RX_Lenth_Total)
	{
		RX_Lenth_Total = 0;
		return 1;
	}
}

uint8_t NUC_Data_Unpack(void)
{
	switch (NUC_USBD_RxBuf[2])
	{
	case CMD_ID_AUTOAIM_DATA_RX:
	{
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

		Referee_Data_Tramsit.red_1_HP = Game_Robot_HP.red_1_robot_HP;
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
		memcpy(NUC_USBD_TxBuf + 3, (uint8_t *)(&Referee_Data_Tramsit), LENGTH_REFEREE_DATA_TX);

		NUC_USBD_TxBuf[LENGTH_AUTOAIM_DATA_TX + 3] = CRC_Calculation(NUC_USBD_TxBuf, LENGTH_REFEREE_DATA_TX + 3);
		//				HAL_UART_Transmit_DMA(&huart1, Usart1_Dma_Txbuf, LENGTH_REFEREE_DATA_TX + 6-2);
		CDC_Transmit_FS(NUC_USBD_TxBuf, LENGTH_REFEREE_DATA_TX + 4);
		break;
	default:
		return;
	}
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
