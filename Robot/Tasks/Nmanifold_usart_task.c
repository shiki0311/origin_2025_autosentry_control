/**
  *****************************************************************************
  * @file       manifold_usart_task.c/h
  * @brief      MANIFOLD data solve. MANIFOLD���ݴ���
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V2.0.0			Mar-3-2023			ʥ������					1.Done
  @verbatim
  ==============================================================================
	Simplified and adapted for new serial port transmission protocol between RMboard_C and NUC.
  ==============================================================================
  @endverbatim
  *****************************************************************************
	*/
/*****************************************************************************************
			��C�塪��NUC����ͨ��Э�飨����汾��C��ײ�ʹ�ò���
��ȷ�������к���bsp_usart.c��bsp_usart.h��
��ȷ��NUC�ϵ�USBת����ģ����C���USART1����C������ϵ�UART2��������ȷ��
��ȷ�������е�USART1������ա�����DMA������Cube��������ã���USART1�Ĳ�������Ϊ115200��
�ܽ���Դ�ļ����뵽�����У�ͬʱȷ��ͷ�ļ��ڹ��̵İ���Ŀ¼�С�
����stm32f4xx_it.c��USART1_IRQHandler�����е���USART1_IRQHandler_1������
��ʹ��xTaskCreate��osThreadDef+osThreadCreate����manifold_usart_task��FreeRTOS����
�߼�鴮���շ��������е�֡ͷ��֡���������ֺ�CRCУ��λ�Ƿ�������
����һ�еײ����þ�����������¼�����AutoAim_Data_Receive�ṹ���ڻ�ȡ������������ݡ�
*****************************************************************************************/
#include "Nmanifold_usbd_task.h"
#include "bsp_usart.h"
#include "arm_math.h"
#include "Shoot_Task.h"
#include "referee.h"
#include "INS_Task.h"

extern UART_HandleTypeDef huart1;

uint8_t Usart1_Buf[2][USART_RX_BUF_LENGHT], NUC_USART_RxBuf[USART_RX_BUF_LENGHT];
uint8_t Usart1_Dma_Txbuf[64] = {0};
uint8_t Usart1_Dma_Txbuf_CHASSIS[64] = {0};
uint8_t Usart1_Dma_Txbuf_WORKING[64] = {0};
uint8_t Usart1_Dma_Txbuf_referee_info[64] = {0};
uint8_t Usart1_Dma_Txbuf_CHASSIA_GIMBAL_ANGLE[16] = {0};
Protocol_Head_Data Protocol_Head;
Working_Mode WMode;
AutoAim_Data_Tx AutoAim_Data_Tramsit;
AutoAim_Data_Rx AutoAim_Data_Receive;
Referee_Data_Tx Referee_Data_Tramsit;

Chassis_Data_Tx Chassis_Data_Tramsit;
Chassis_Data_Rx Chassis_Data_Receive;
Move_cmd_Data_Rx Move_cmd_Receive;

Rotate_Data_Rx Rotate_Data_Receive;

Chassis_Gimbal_Angle_TX Chassis_Gimbal_Angle_Tramsit;

Dial_Switch_Data Dial_Switch;

uint8_t Autoaim_Mode = AUTOAIM_MODE_ANTI_TOP; // ��С����ģʽ
uint8_t Autoaim_Armor = AUTOAIM_ARMOR_AUTO;   // �Զ�ѡ���Сװ��
uint8_t autoaim_mode=2,flag_gimbal_ecd_cnt=0;
uint8_t autoaim_armor=0x10;
uint8_t flag_AUTOAIM_DATA=0,cnt_AUTOAIM_DATA=0;
static fp32 fric_real_speed=30.0f;

//extern ext_game_robot_state_t Game_Robot_State;
//extern ext_game_robot_state_t Game_State;
//extern ext_game_robot_state_t Bullet_Remaining;
//extern ext_game_robot_state_t Game_Robot_HP;
//extern ext_game_robot_HP_t RFID_Status;

void manifold_usart_task(void)
{
	vTaskDelay(200);
	memset(NUC_USART_RxBuf, 0x00, USART_RX_BUF_LENGHT);								
	usart1_init(Usart1_Buf[0], Usart1_Buf[1], USART_RX_BUF_LENGHT);			
	vTaskDelay(200);
	static uint16_t t = 0;
	static uint8_t isDelay = 0;
	while(1)
	{
		if(++t > 799) t = 0;
//		if (isDelay) 
//		{
//			isDelay = 0;
//			vTaskDelay(1);
//		} 
//		else vTaskDelay(2);
		
		//��NUC��λ������������̨��̬��Ϣ������������Ի�ȡ����������Ϣ
		if(t % 7 == 0) 
		{
			NUC_Usart_Tx(CMD_ID_AUTOAIM_DATA_TX);	
			vTaskDelay(1);
		}
		
		// ���Ͳ���ϵͳ����
		if(t % 50 == 0)
		{
			NUC_Usart_Tx(CMD_ID_REFEREE_DATA_TX);	
			vTaskDelay(1);
		}
		
//		if(flag_AUTOAIM_DATA==1) // �Ƿ��յ���������
//		{
//			NUC_Usart_Tx(CMD_ID_CHASSIA_GIMBAL_ANGLE);
//			flag_AUTOAIM_DATA=0;
//			vTaskDelay(1);
//		}
//		//��NUC��λ����������ģʽ��Ϣ��ָ��NUCִ����Ӧ�������㷨
//		if(t % 19 == 0) 
//		{
//			NUC_Usart_Tx(CMD_ID_WORKING_MODE);
//			vTaskDelay(1);
//			isDelay = 1;
//		}	
//		
//		//��NUC��λ�����Ͳ��뿪��������Ϣ����NUCȷ���з��������ɫ�����͵з�����������װ�װ��С
//		if(t % 41 == 0)
//			{
//			NUC_Usart_Tx(CMD_ID_DIAL_SWITCH);
//			vTaskDelay(1);
//				isDelay = 1;

//			}
	}
}

void USART1_IRQHandler_1(void)
{
	static uint16_t RX_Len_Now = 0;
	if(USART1->SR & UART_FLAG_IDLE)
	{
		__HAL_UART_CLEAR_PEFLAG(&huart1);
		__HAL_DMA_DISABLE(huart1.hdmarx);
		RX_Len_Now = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart1.hdmarx);
		__HAL_DMA_SET_COUNTER(huart1.hdmarx, USART_RX_BUF_LENGHT);
		if((huart1.hdmarx->Instance->CR & DMA_SxCR_CT) == RESET)
		{
			huart1.hdmarx->Instance->CR |= DMA_SxCR_CT;
			memcpy(NUC_USART_RxBuf, (char*)Usart1_Buf[0], RX_Len_Now);
		}
		else
		{
			huart1.hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
			memcpy(NUC_USART_RxBuf, (char*)Usart1_Buf[1], RX_Len_Now);
		}
		__HAL_DMA_ENABLE(huart1.hdmarx);
		NUC_Data_Unpack();
	}
}

uint8_t NUC_Data_Unpack(void)
{
	if(NUC_USART_RxBuf[0] != 0xAA) // ��һ��ֵ��Ҫ���������Ȼ���˳�
		return 1;
	switch(NUC_USART_RxBuf[2]) // ���������������жϷ��͵����ķ��������
	{
		case CMD_ID_AUTOAIM_DATA_RX:
		{
			if(NUC_USART_RxBuf[1] != 0x49 )
				return 1;
			memcpy(&AutoAim_Data_Receive, NUC_USART_RxBuf + 3, sizeof(AutoAim_Data_Rx));
			Chassis_Data_Receive.vx = AutoAim_Data_Receive.vx;
			Chassis_Data_Receive.vy = AutoAim_Data_Receive.vy;
			Chassis_Data_Receive.yaw_speed = AutoAim_Data_Receive.yaw_speed;
			Chassis_Data_Receive.pitch_speed = AutoAim_Data_Receive.pitch_speed;
			Rotate_Data_Receive.rotate = AutoAim_Data_Receive.rotate;
//			AutoAim_Data_Receive.Pitch=-AutoAim_Data_Receive.Pitch;
			break;
		}

//		
//		case CMD_ID_MOVE_CMD_DATA_RX:
//		{
//			if(NUC_USART_RxBuf[1] != LENGTH_move_cmd_DATA_RX + 4 )//|| CRC_Calculation(NUC_USART_RxBuf, LENGTH_AUTOAIM_DATA_RX + 3) != NUC_USART_RxBuf[3 + LENGTH_AUTOAIM_DATA_RX])
//				return 1;
//			memcpy(&Move_cmd_Receive, NUC_USART_RxBuf + 3, sizeof(Move_cmd_Data_Rx));
//			Chassis_Data_Receive.vx = Move_cmd_Receive.vx;
//			Chassis_Data_Receive.vy = Move_cmd_Receive.vy;
//			Chassis_Data_Receive.yaw_speed = Move_cmd_Receive.yaw_speed;
//			Chassis_Data_Receive.pitch_speed = Move_cmd_Receive.pitch_speed;
//			Rotate_Data_Receive.rotate = Move_cmd_Receive.rotate;
//			break;
//		}
//		case CMD_ID_CHASSIS_DATA_RX:
//		{
//	//		if(NUC_USART_RxBuf[1]!=LENGTH_CHASSIS_DATA_RX + 4 )//||  CRC_Calculation(NUC_USART_RxBuf, LENGTH_CHASSIS_DATA_RX + 3) != NUC_USART_RxBuf[3 + LENGTH_CHASSIS_DATA_RX])
//	//			return 1;	
//			memcpy(&Chassis_Data_Receive, NUC_USART_RxBuf + 3, sizeof(Chassis_Data_Rx));
//			break;
//		}
//		
//		case CMD_ID_ROTATE_DATA_RX:
//		{
//			if(NUC_USART_RxBuf[1]!= LENGTH_ROTATE_DATA_RX + 4 ||  CRC_Calculation(NUC_USART_RxBuf, LENGTH_ROTATE_DATA_RX + 3) != NUC_USART_RxBuf[3 + LENGTH_ROTATE_DATA_RX])
//				return 1;
//			memcpy(&Rotate_Data_Receive, NUC_USART_RxBuf + 3, sizeof(Rotate_Data_Rx));
//			break;	
//		}
			
		default:
			return 1;
	}
	return 0;
}

void NUC_Usart_Tx(uint8_t cmdid)
{
	
	Protocol_Head.Header = 0xAA;
	Protocol_Head.Cmd_ID = cmdid;
	switch(cmdid){
		case CMD_ID_AUTOAIM_DATA_TX:   // ���ͱ�����̬���ݣ���������
			Protocol_Head.Length = LENGTH_AUTOAIM_DATA_TX + 4;
			memcpy(Usart1_Dma_Txbuf, (uint8_t *)(&Protocol_Head), 3);
			AutoAim_Data_Tramsit.Pitch = INS_angle_deg[1];
			AutoAim_Data_Tramsit.Roll = INS_angle_deg[2];
			AutoAim_Data_Tramsit.Yaw = INS_angle_deg[0];
			
			memcpy(Usart1_Dma_Txbuf + 3, (uint8_t *)(&AutoAim_Data_Tramsit), LENGTH_AUTOAIM_DATA_TX);
			
			Usart1_Dma_Txbuf[LENGTH_AUTOAIM_DATA_TX + 3] = CRC_Calculation(Usart1_Dma_Txbuf, LENGTH_AUTOAIM_DATA_TX + 3);
			//Usart1_Dma_Txbuf[LENGTH_AUTOAIM_DATA_TX + 4] = '\r';
			//Usart1_Dma_Txbuf[LENGTH_AUTOAIM_DATA_TX + 5] = '\n';
			flag_AUTOAIM_DATA=1;
			cnt_AUTOAIM_DATA++;
			HAL_UART_Transmit_DMA(&huart1, Usart1_Dma_Txbuf, LENGTH_AUTOAIM_DATA_TX + 6-2); // �������ݼ��� 3λProtocol_Head �� 1λCRC
			break;

		case CMD_ID_WORKING_MODE:  // ����ģʽ��Ϣ
			Protocol_Head.Length = LENGTH_WORKING_MODE + 4;
			memcpy(Usart1_Dma_Txbuf_WORKING,(uint8_t *)(&Protocol_Head), 3);
			WMode = Autoaim_Armor | Autoaim_Mode;
			memcpy(Usart1_Dma_Txbuf_WORKING + 3, (uint8_t *)(&WMode), LENGTH_WORKING_MODE);
			
			Usart1_Dma_Txbuf_WORKING[LENGTH_WORKING_MODE + 3] = CRC_Calculation(Usart1_Dma_Txbuf_WORKING, LENGTH_WORKING_MODE + 3);
			//Usart1_Dma_Txbuf[LENGTH_WORKING_MODE + 4] = '\r';
			//Usart1_Dma_Txbuf[LENGTH_WORKING_MODE + 5] = '\n';
			
			HAL_UART_Transmit_DMA(&huart1, Usart1_Dma_Txbuf_WORKING, LENGTH_WORKING_MODE + 6-2);
			break;
		
		case CMD_ID_DIAL_SWITCH:
			Protocol_Head.Length = LENGTH_DIAL_SWITCH + 4;
			memcpy(Usart1_Dma_Txbuf,(uint8_t *)(&Protocol_Head), 3);
			
			Dial_Switch.Switch_off			= HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_9) ? SWITCH_ON	: SWITCH_OFF ;
			if(Dial_Switch.Switch_off == 2){
				Dial_Switch.Infantry_Armor[0] 	= HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7) ? ENEMY_INFANTRY_ARMOR_BIG	: ENEMY_INFANTRY_ARMOR_SMALL;
				Dial_Switch.Infantry_Armor[1] 	= HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_0) ? ENEMY_INFANTRY_ARMOR_BIG 	: ENEMY_INFANTRY_ARMOR_SMALL;
				Dial_Switch.Infantry_Armor[2] 	= HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_1) ? ENEMY_INFANTRY_ARMOR_BIG	: ENEMY_INFANTRY_ARMOR_SMALL;
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) ? GPIO_PIN_RESET : GPIO_PIN_SET);
		
				memcpy(Usart1_Dma_Txbuf + 3, (uint8_t *)(&Dial_Switch), LENGTH_DIAL_SWITCH);
			
				Usart1_Dma_Txbuf[LENGTH_DIAL_SWITCH + 3] = CRC_Calculation(Usart1_Dma_Txbuf, LENGTH_DIAL_SWITCH + 3);
			
				HAL_UART_Transmit_DMA(&huart1, Usart1_Dma_Txbuf, LENGTH_DIAL_SWITCH + 6-2);
			}
			break;
		
		case CMD_ID_CHASSIA_GIMBAL_ANGLE:
		
			Protocol_Head.Length = LENGTH_CHASSIS_GIMBAL_ANGLE + 4;
			memcpy(Usart1_Dma_Txbuf_CHASSIA_GIMBAL_ANGLE, (uint8_t *)(&Protocol_Head), 3);
			memcpy(Usart1_Dma_Txbuf_CHASSIA_GIMBAL_ANGLE + 3, (uint8_t *)(&Chassis_Gimbal_Angle_Tramsit), LENGTH_CHASSIS_GIMBAL_ANGLE);
		
			Usart1_Dma_Txbuf_CHASSIA_GIMBAL_ANGLE[LENGTH_CHASSIS_GIMBAL_ANGLE + 3] = CRC_Calculation(Usart1_Dma_Txbuf_CHASSIA_GIMBAL_ANGLE, LENGTH_CHASSIS_GIMBAL_ANGLE + 3);
					flag_gimbal_ecd_cnt++;
			HAL_UART_Transmit_DMA(&huart1, Usart1_Dma_Txbuf_CHASSIA_GIMBAL_ANGLE, LENGTH_CHASSIS_GIMBAL_ANGLE + 6-2);
			break;
		
		case CMD_ID_REFEREE_DATA_TX:
			Protocol_Head.Length = LENGTH_REFEREE_DATA_TX + 4;
			memcpy(Usart1_Dma_Txbuf, (uint8_t *)(&Protocol_Head), 3);

			Referee_Data_Tramsit.remain_HP = Game_Robot_State.remain_HP;
			Referee_Data_Tramsit.max_HP = Game_Robot_State.max_HP;
			Referee_Data_Tramsit.game_type = Game_Status.game_type;
			Referee_Data_Tramsit.game_progress = Game_Status.game_progress;
			Referee_Data_Tramsit.stage_remain_time = Game_Status.stage_remain_time;
			Referee_Data_Tramsit.coin_remaining_num = Bullet_Remaining.coin_remaining_num;
			Referee_Data_Tramsit.bullet_remaining_num_17mm = Bullet_Remaining.bullet_remaining_num_17mm;
			
			Referee_Data_Tramsit.red_1_HP = Game_Robot_HP.red_1_robot_HP;
			Referee_Data_Tramsit.red_2_HP = Game_Robot_HP.red_2_robot_HP;
			Referee_Data_Tramsit.red_3_HP = Game_Robot_HP.red_3_robot_HP;
			Referee_Data_Tramsit.red_4_HP = Game_Robot_HP.red_4_robot_HP;
			Referee_Data_Tramsit.red_5_HP = Game_Robot_HP.red_5_robot_HP;
			Referee_Data_Tramsit.red_7_HP = Game_Robot_HP.red_7_robot_HP;
			Referee_Data_Tramsit.red_outpost_HP = Game_Robot_HP.red_outpost_HP;
			Referee_Data_Tramsit.red_base_HP = Game_Robot_HP.red_base_HP;
			
			Referee_Data_Tramsit.blue_1_HP = Game_Robot_HP.blue_1_robot_HP;
			Referee_Data_Tramsit.blue_2_HP = Game_Robot_HP.blue_2_robot_HP;
			Referee_Data_Tramsit.blue_3_HP = Game_Robot_HP.blue_3_robot_HP;
			Referee_Data_Tramsit.blue_4_HP = Game_Robot_HP.blue_4_robot_HP;
			Referee_Data_Tramsit.blue_5_HP = Game_Robot_HP.blue_5_robot_HP;
			Referee_Data_Tramsit.blue_7_HP = Game_Robot_HP.blue_7_robot_HP;
			Referee_Data_Tramsit.blue_outpost_HP = Game_Robot_HP.blue_outpost_HP;
			Referee_Data_Tramsit.blue_base_HP = Game_Robot_HP.blue_base_HP;
			
			Referee_Data_Tramsit.rfid_status = RFID_Status.rfid_status;
		
			memcpy(Usart1_Dma_Txbuf + 3, (uint8_t *)(&Referee_Data_Tramsit), LENGTH_REFEREE_DATA_TX);
			
			Usart1_Dma_Txbuf[LENGTH_AUTOAIM_DATA_TX + 3] = CRC_Calculation(Usart1_Dma_Txbuf, LENGTH_REFEREE_DATA_TX + 3);
			HAL_UART_Transmit_DMA(&huart1, Usart1_Dma_Txbuf, LENGTH_REFEREE_DATA_TX + 6-2); // �������ݼ��� 3λProtocol_Head �� 1λCRC
			break;
		
		default:
			return;
	}
}

uint8_t CRC_Calculation(uint8_t *ptr, uint16_t len)  // ����������ݿ��ѭ������У�飨CRC��ֵ �� ������ݴ����������û�д���
{
	uint8_t crc = 0xff;
	while (len--) {
			crc = CRC08_Table[crc ^ *ptr++];
	}
	return crc;
}

