/**
  *****************************************************************************
  * @file       Nmanifold_usbd_task.c/h
  * @brief      MANIFOLD data solve. MANIFOLD���ݴ���
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V2.0.0			Mar-3-2023			ʥ������					1.Done
  *  V2.1.0			May-16-2023			ʥ������					1.Add data transmission for dial switch
  *  V1.0usbd  		October-28-2024		captainwu				1.transform to usbd
  @verbatim
  ==============================================================================
	
  ==============================================================================
  @endverbatim
  *****************************************************************************
*/
/*****************************************************************************************
			��C�塪��NUC����ͨ��Э�飨����汾��C��ײ�ʹ�ò���?
��
��
��
�ܽ���Դ�ļ����뵽�����У�ͬʱȷ��ͷ�ļ��ڹ��̵İ���Ŀ¼�С�
����stm32f4xx_it.c��USART1_IRQHandler_1�����е���USART1_IRQHandler_1������
��ʹ��xTaskCreate��osThreadDef+osThreadCreate����manifold_usart_task��FreeRTOS����
�߼��?���շ��������е�֡ͷ��֡���������ֺ�CRCУ��λ�Ƿ�������
����һ�еײ����þ�����������¼�����AutoAim_Data_Receive�ṹ���ڻ�ȡ������������ݡ�?
*****************************************************************************************/
#include "Nmanifold_usbd_task.h"
#include "arm_math.h"
#include "Shoot_Task.h"
#include "referee.h"
#include "remote_control.h"

#include "usb_device.h"
#include "usbd_cdc_if.h"
extern USBD_HandleTypeDef hUsbDeviceFS;
extern ext_game_robot_state_t Game_Robot_State;
uint8_t RX_Lenth_Total=0;


uint8_t USBD_Buf[2][USBD_RX_BUF_LENGHT], NUC_USBD_RxBuf[USBD_RX_BUF_LENGHT],NUC_USBD_TxBuf[USBD_TX_BUF_LENGHT];

Protocol_Head_Data Protocol_Head;
AutoAim_Data_Tx AutoAim_Data_Transmit;
AutoAim_Data_Rx AutoAim_Data_Receive;
Dial_Switch_Data Dial_Switch;


Protocol_Head_Data Protocol_Head;
Working_Mode WMode;
AutoAim_Data_Tx AutoAim_Data_Tramsit;
AutoAim_Data_Rx AutoAim_Data_Receive;
Referee_Data_Tx Referee_Data_Tramsit;
Chassis_Data_Tx Chassis_Data_Tramsit;

Chassis_Gimbal_Angle_TX Chassis_Gimbal_Angle_Tramsit;

Dial_Switch_Data Dial_Switch;

uint8_t Autoaim_Mode = AUTOAIM_MODE_ANTI_TOP; // ��С����ģʽ
uint8_t Autoaim_Armor = AUTOAIM_ARMOR_AUTO;   // �Զ�ѡ���Сװ��?
uint8_t autoaim_mode=2,flag_gimbal_ecd_cnt=0;
uint8_t autoaim_armor=0x10;
uint8_t flag_AUTOAIM_DATA=0,cnt_AUTOAIM_DATA=0;
static fp32 fric_real_speed=30.0f;


void manifold_usart_task(void)
{
	vTaskDelay(200);
	memset(NUC_USBD_RxBuf, 0x00, USBD_RX_BUF_LENGHT);										
	vTaskDelay(200);
	static uint16_t t = 0;
	static uint8_t isDelay = 0;
	while(1)
	{
		if(++t > 799) 
			t = 0;
		
		//��NUC��λ������������̨��̬��Ϣ������������Ի�ȡ�����������?
		if(t % 7 == 0) 
		{
			NUC_USBD_Tx(CMD_ID_AUTOAIM_DATA_TX);	
			vTaskDelay(1);
		}
		
		// ���Ͳ���ϵͳ����
		if(t % 50 == 0)
		{
			NUC_USBD_Tx(CMD_ID_REFEREE_DATA_TX);	
			vTaskDelay(1);
		}
	};
}


uint8_t USBD_IRQHandler(uint8_t* Buf, uint16_t Len)
{		
		memcpy(NUC_USBD_RxBuf+RX_Lenth_Total,Buf, Len);
		if(NUC_USBD_RxBuf[0] != 0xAA || NUC_USBD_RxBuf[1] > 128 )//�Ҳ��Ǳ�ʶ������ʶ������0xAA�ͷ���1
			return 1;		
		RX_Lenth_Total=Len+RX_Lenth_Total;
	
		if(NUC_USBD_RxBuf[1] == RX_Lenth_Total) //����У�飬��У��߽���?
			{
				NUC_Data_Unpack();//���ݽ��?
//				CDC_Transmit_FS(NUC_USBD_RxBuf, RX_Lenth_Total);
				RX_Lenth_Total=0;
			}
			
		if(NUC_USBD_RxBuf[1] < RX_Lenth_Total)
		{
			RX_Lenth_Total=0;
			return 1;
		}
				
}


uint8_t NUC_Data_Unpack(void)
{	
	switch(NUC_USBD_RxBuf[2])
	{		
		case CMD_ID_AUTOAIM_DATA_RX:
		{
			memcpy(&AutoAim_Data_Receive, NUC_USBD_RxBuf + 3, sizeof(AutoAim_Data_Rx));
			break;
		}		/*��һ���е�ʺ����������������һ������һ����68�ֽ�	*/
							
		default:
			return 1;
	}
	
	return 0;
}
uint8_t  shijuefasong_or_not_watch=5;
void NUC_USBD_Tx(uint8_t cmdid){
	Working_Mode WMode;
	Protocol_Head.Header = 0xAA;
	Protocol_Head.Cmd_ID = cmdid;
	switch(cmdid){
		case CMD_ID_AUTOAIM_DATA_TX:   // ���ͱ�����̬���ݣ���������
			Protocol_Head.Length = LENGTH_AUTOAIM_DATA_TX + 4;
			memcpy(NUC_USBD_TxBuf, (uint8_t *)(&Protocol_Head), 3);
			AutoAim_Data_Tramsit.Pitch = INS_angle_deg[1];
			AutoAim_Data_Tramsit.Roll = INS_angle_deg[2];
			AutoAim_Data_Tramsit.Yaw = INS_angle_deg[0];
			
			memcpy(NUC_USBD_TxBuf + 3, (uint8_t *)(&AutoAim_Data_Tramsit), LENGTH_AUTOAIM_DATA_TX);
			
			NUC_USBD_TxBuf[LENGTH_AUTOAIM_DATA_TX + 3] = CRC_Calculation(NUC_USBD_TxBuf, LENGTH_AUTOAIM_DATA_TX + 3);
			flag_AUTOAIM_DATA=1;
			cnt_AUTOAIM_DATA++;
//			HAL_UART_Transmit_DMA(&huart1, Usart1_Dma_Txbuf, LENGTH_AUTOAIM_DATA_TX + 6-2+1); // �������ݼ��� 3λProtocol_Head �� 1λCRC��һλ������
			CDC_Transmit_FS(NUC_USBD_TxBuf, LENGTH_AUTOAIM_DATA_TX  + 6-2);
			break;

		
		case CMD_ID_WORKING_MODE:
			Protocol_Head.Length = LENGTH_WORKING_MODE + 4;
			memcpy(NUC_USBD_TxBuf,(uint8_t *)(&Protocol_Head), 3);
			WMode = Autoaim_Armor | Autoaim_Mode;
			memcpy(NUC_USBD_TxBuf + 3, (uint8_t *)(&WMode), LENGTH_WORKING_MODE);
			
			NUC_USBD_TxBuf[LENGTH_WORKING_MODE + 3] = CRC_Calculation(NUC_USBD_TxBuf, LENGTH_WORKING_MODE + 3);		
//			HAL_UART_Transmit_DMA(&huart1, Usart1_Dma_Txbuf, LENGTH_WORKING_MODE + 6-2);
			CDC_Transmit_FS(NUC_USBD_TxBuf, LENGTH_WORKING_MODE + 6-2);	
		break;
		
		case CMD_ID_DIAL_SWITCH:
			Protocol_Head.Length = LENGTH_DIAL_SWITCH + 4;
			memcpy(NUC_USBD_TxBuf,(uint8_t *)(&Protocol_Head), 3);
			
			Dial_Switch.Switch_off			= HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_9) ? SWITCH_ON	: SWITCH_OFF ;
			if(Dial_Switch.Switch_off == 2){
				Dial_Switch.Infantry_Armor[0] 	= HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7) ? ENEMY_INFANTRY_ARMOR_BIG	: ENEMY_INFANTRY_ARMOR_SMALL;
				Dial_Switch.Infantry_Armor[1] 	= HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_0) ? ENEMY_INFANTRY_ARMOR_BIG 	: ENEMY_INFANTRY_ARMOR_SMALL;
				Dial_Switch.Infantry_Armor[2] 	= HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_1) ? ENEMY_INFANTRY_ARMOR_BIG	: ENEMY_INFANTRY_ARMOR_SMALL;
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) ? GPIO_PIN_RESET : GPIO_PIN_SET);
		
				memcpy(NUC_USBD_TxBuf + 3, (uint8_t *)(&Dial_Switch), LENGTH_DIAL_SWITCH);
			
				NUC_USBD_TxBuf[LENGTH_DIAL_SWITCH + 3] = CRC_Calculation(NUC_USBD_TxBuf, LENGTH_DIAL_SWITCH + 3);
			
//				HAL_UART_Transmit_DMA(&huart1, Usart1_Dma_Txbuf, LENGTH_DIAL_SWITCH + 6-2);
				CDC_Transmit_FS(NUC_USBD_TxBuf, LENGTH_DIAL_SWITCH + 6-2);
			}
			break;
			
			case CMD_ID_CHASSIA_GIMBAL_ANGLE:
		
				Protocol_Head.Length = LENGTH_CHASSIS_GIMBAL_ANGLE + 4;
				memcpy(NUC_USBD_TxBuf, (uint8_t *)(&Protocol_Head), 3);
				memcpy(NUC_USBD_TxBuf + 3, (uint8_t *)(&Chassis_Gimbal_Angle_Tramsit), LENGTH_CHASSIS_GIMBAL_ANGLE);
		
				NUC_USBD_TxBuf[LENGTH_CHASSIS_GIMBAL_ANGLE + 3] = CRC_Calculation(NUC_USBD_TxBuf, LENGTH_CHASSIS_GIMBAL_ANGLE + 3);
					flag_gimbal_ecd_cnt++;
//				HAL_UART_Transmit_DMA(&huart1, Usart1_Dma_Txbuf_CHASSIA_GIMBAL_ANGLE, LENGTH_CHASSIS_GIMBAL_ANGLE + 6-2);
				CDC_Transmit_FS(NUC_USBD_TxBuf, LENGTH_CHASSIS_GIMBAL_ANGLE + 6-2);
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
				Referee_Data_Tramsit.event_data=Event_Data.event_type;
				Referee_Data_Tramsit.hurt_reason=Robot_Hurt.hurt_type;
				Referee_Data_Tramsit.sentry_bullet_already_get=(uint16_t)(Sentry_Info.sentry_info & 0x7FF);
				memcpy(NUC_USBD_TxBuf + 3, (uint8_t *)(&Referee_Data_Tramsit), LENGTH_REFEREE_DATA_TX);
				
				NUC_USBD_TxBuf[LENGTH_AUTOAIM_DATA_TX + 3] = CRC_Calculation(NUC_USBD_TxBuf, LENGTH_REFEREE_DATA_TX + 3); 
//				HAL_UART_Transmit_DMA(&huart1, Usart1_Dma_Txbuf, LENGTH_REFEREE_DATA_TX + 6-2); // �������ݼ��� 3λProtocol_Head �� 1λCRC
				CDC_Transmit_FS(NUC_USBD_TxBuf, LENGTH_REFEREE_DATA_TX + 4);
				break;
		default:
			return;
	}
}

uint8_t CRC_Calculation(uint8_t *ptr, uint16_t len) {
	uint8_t crc = 0xff;
	while (len--) {
			crc = CRC08_Table[crc ^ *ptr++];
	}
	return crc;
}
