#include "bsp_can.h"
#include "bsp_cap.h"
#include "main.h"
#include "Chassis_Task.h"
#include "Gimbal_Task.h"
#include "Shoot_Task.h"
#include "detect_task.h"
#include "user_common_lib.h"

#define P_MIN -3.1415926f
#define P_MAX 3.1415926f
#define V_MIN -30.0f
#define V_MAX 30.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -30.0f
#define T_MAX 30.0f
#define DM4310_RecID 0x00
#define get_motor_measure(ptr, data)                                   \
	{                                                                  \
		(ptr)->last_ecd = (ptr)->ecd;                                  \
		(ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);           \
		(ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);     \
		(ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]); \
		(ptr)->temperate = (data)[6];                                  \
	}

uint8_t rx_data2[8];
uint8_t rx_data[8];
motor_measure_t motor_measure_chassis[4];
motor_measure_t motor_measure_gimbal[2];
motor_measure_t motor_measure_shoot[3];
DM_motor_data_t DM_pitch_motor_data = {0};
CanTxQueueTypeDef can_tx_queue;
uint32_t id = 0;
int32_t dial_angle = 0;
CAN_RxHeaderTypeDef rx_header;
CAN_RxHeaderTypeDef rx_header2;
void can_filter_init(void)
{
	CAN_FilterTypeDef can_filter_st;
	can_filter_st.FilterActivation = ENABLE;
	can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
	can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
	can_filter_st.FilterIdHigh = 0x0000;
	can_filter_st.FilterIdLow = 0x0000;
	can_filter_st.FilterMaskIdHigh = 0x0000;
	can_filter_st.FilterMaskIdLow = 0x0000;
	can_filter_st.FilterBank = 0;
	can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
	HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_TX_MAILBOX_EMPTY);

	can_filter_st.SlaveStartFilterBank = 14;
	can_filter_st.FilterBank = 14;
	HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
	HAL_CAN_Start(&hcan2);
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if (hcan == &hcan1)
	{
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

		if (rx_header.StdId != CAN_3508_M1_ID && rx_header.StdId != CAN_3508_M2_ID && rx_header.StdId != CAN_3508_M3_ID && rx_header.StdId != CAN_3508_M4_ID)
			id = rx_header.StdId;

		switch (rx_header.StdId)
		{
		case CAN_3508_M1_ID:
		case CAN_3508_M2_ID:
		case CAN_3508_M3_ID:
		case CAN_3508_M4_ID:
		{
			static uint8_t i = 0;
			i = rx_header.StdId - CAN_3508_M1_ID;
			get_motor_measure(&motor_measure_chassis[i], rx_data);
			int16_t temp1 = motor_measure_chassis[i].ecd - motor_measure_chassis[i].last_ecd;
			int16_t temp2 = temp1 + (temp1 < 0 ? 8192 : -8192);
			motor_measure_chassis[i].code += abs(temp2) < abs(temp1) ? temp2 : temp1;
			detect_hook(CHASSIS_MOTOR1_TOE + i);

			break;
		}
		case CAN_6020_M1_ID:
		{
			static uint8_t i = 0;
			i = rx_header.StdId - CAN_6020_M1_ID;

			get_motor_measure(&motor_measure_gimbal[i], rx_data);

			break;
		}

		case CAN_CAP_RX_ID:
		{
			update_cap(rx_data);
			break;
		}

		default:
		{
			break;
		}
		}
	}
	if (hcan == &hcan2)
	{
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header2, rx_data2);
		switch (rx_header2.StdId)
		{
		case DM4310_RecID:
		{
			DM_pitch_motor_data.id = (rx_data2[0]) & 0x0F;
			DM_pitch_motor_data.state = (rx_data2[0]) >> 4;
			DM_pitch_motor_data.p_int = (rx_data2[1] << 8) | rx_data2[2];
			DM_pitch_motor_data.v_int = (rx_data2[3] << 4) | (rx_data2[4] >> 4);
			DM_pitch_motor_data.t_int = ((rx_data2[4] & 0xF) << 8) | rx_data2[5];
			DM_pitch_motor_data.pos = uint_to_float(DM_pitch_motor_data.p_int, P_MIN, P_MAX, 16) * 57.3248408; // (-3.1415926,3.1415926)
			DM_pitch_motor_data.vel = uint_to_float(DM_pitch_motor_data.v_int, V_MIN, V_MAX, 12);
			DM_pitch_motor_data.toq = uint_to_float(DM_pitch_motor_data.t_int, T_MIN, T_MAX, 12); // (-18.0,18.0)
			DM_pitch_motor_data.Tmos = (float)(rx_data2[6]);
			DM_pitch_motor_data.Tcoil = (float)(rx_data2[7]);

			break;
		}
		case CAN_2006_M1_ID:
		{
			get_motor_measure(&motor_measure_shoot[2], rx_data2);
			if (motor_measure_shoot[2].ecd - motor_measure_shoot[2].last_ecd > 4096)
				dial_angle += -8192 + motor_measure_shoot[2].ecd - motor_measure_shoot[2].last_ecd;
			else if (motor_measure_shoot[2].ecd - motor_measure_shoot[2].last_ecd < -4096)
				dial_angle += 8192 + motor_measure_shoot[2].ecd - motor_measure_shoot[2].last_ecd;
			else
				dial_angle += motor_measure_shoot[2].ecd - motor_measure_shoot[2].last_ecd;

			break;
		}
		case CAN_3508_M5_ID:
		case CAN_3508_M6_ID:
		{
			static uint8_t i = 0;
			i = rx_header2.StdId - CAN_3508_M5_ID;

			get_motor_measure(&motor_measure_shoot[i], rx_data2);
			break;
		}
		}
	}
}

void CAN_TxQueue_Init()
{
	memset(can_tx_queue.can_msg_buffer, 0, sizeof(can_tx_queue.can_msg_buffer));
	can_tx_queue.element_number = 0;
	can_tx_queue.head = 0;
	can_tx_queue.tail = 0;
}

void CAN_TxQueue_Push(CAN_TxHeaderTypeDef *pHeader, uint8_t *pData)
{
	__disable_irq();
	can_tx_queue.tail = (can_tx_queue.head + can_tx_queue.element_number) % TX_QUEUE_SIZE;
	can_tx_queue.can_msg_buffer[can_tx_queue.tail].tx_header = *pHeader;
	memcpy(can_tx_queue.can_msg_buffer[can_tx_queue.tail].data, pData, pHeader->DLC);
	if (can_tx_queue.element_number == TX_QUEUE_SIZE)
	{
		can_tx_queue.head = (can_tx_queue.head + 1) % TX_QUEUE_SIZE;
	}
	else
	{
		can_tx_queue.element_number++;
	}
	__enable_irq();
}

int CAN_TxQueue_Pop(CAN_TxHeaderTypeDef *pHeader, uint8_t *pData)
{
	__disable_irq();
	if (can_tx_queue.element_number == 0)
	{
		__enable_irq();
		return -1;
	}
	*pHeader = can_tx_queue.can_msg_buffer[can_tx_queue.head].tx_header;
	memcpy(pData, can_tx_queue.can_msg_buffer[can_tx_queue.head].data, pHeader->DLC);
	can_tx_queue.head = (can_tx_queue.head + 1) % TX_QUEUE_SIZE;
	can_tx_queue.element_number--;
	__enable_irq();
	return 0;
}
void Process_TxQueue(CAN_HandleTypeDef *hcan)
{
	CAN_TxHeaderTypeDef tx_header;
	uint8_t tx_data[8];
	uint32_t send_mail_box;
	// 尝试发送队列中的消息
	while (HAL_CAN_GetTxMailboxesFreeLevel(hcan) > 0)
	{
		if (CAN_TxQueue_Pop(&tx_header, tx_data) == 0)
		{
			HAL_CAN_AddTxMessage(hcan, &tx_header, tx_data, &send_mail_box);
		}
		else
			break;
	}
}
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
	if (hcan == &hcan1)
		Process_TxQueue(hcan);
}

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan)
{
	if (hcan == &hcan1)
		Process_TxQueue(hcan);
}

void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan)
{
	if (hcan == &hcan1)
		Process_TxQueue(hcan);
}

void CAN_Chassis_CMD(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4) //-16384,+16384
{
	CAN_TxHeaderTypeDef chassis_tx_message;
	uint8_t chassis_can_send_data[8];
	uint32_t send_mail_box;
	chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID;
	chassis_tx_message.IDE = CAN_ID_STD;
	chassis_tx_message.RTR = CAN_RTR_DATA;
	chassis_tx_message.DLC = 0x08;
	chassis_can_send_data[0] = motor1 >> 8;
	chassis_can_send_data[1] = motor1;
	chassis_can_send_data[2] = motor2 >> 8;
	chassis_can_send_data[3] = motor2;
	chassis_can_send_data[4] = motor3 >> 8;
	chassis_can_send_data[5] = motor3;
	chassis_can_send_data[6] = motor4 >> 8;
	chassis_can_send_data[7] = motor4;

	HAL_StatusTypeDef status;
	status = HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
	if (status != HAL_OK)
	{
		// can发送失败送入can发送队列中
		CAN_TxQueue_Push(&chassis_tx_message, chassis_can_send_data);
	}
}

void CAN_Cap_CMD(float data1, float data2, float data3, float data4)
{
	CAN_TxHeaderTypeDef cap_tx_message;
	uint8_t cap_can_send_data[8];
	uint32_t send_mail_box;
	cap_tx_message.StdId = CAN_CAP_TX_ID;
	cap_tx_message.IDE = CAN_ID_STD;
	cap_tx_message.RTR = CAN_RTR_DATA;
	cap_tx_message.DLC = 0x08;

	uint16_t temp;

	temp = data1 * 100;
	cap_can_send_data[0] = temp;
	cap_can_send_data[1] = temp >> 8;
	temp = data2 * 100;
	cap_can_send_data[2] = temp;
	cap_can_send_data[3] = temp >> 8;
	temp = data3 * 100;
	cap_can_send_data[4] = temp;
	cap_can_send_data[5] = temp >> 8;
	temp = data4 * 100;
	cap_can_send_data[6] = temp;
	cap_can_send_data[7] = temp >> 8;

	HAL_StatusTypeDef status;
	status = HAL_CAN_AddTxMessage(&CHASSIS_CAN, &cap_tx_message, cap_can_send_data, &send_mail_box);
	if (status != HAL_OK)
	{
		// can发送失败送入can发送队列中
		CAN_TxQueue_Push(&cap_tx_message, cap_can_send_data);
	}
}

void CAN_Gimbal_CMD(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4) //-30000,+30000
{
	CAN_TxHeaderTypeDef gimbal_tx_message;
	uint8_t gimbal_can_send_data[8];
	uint32_t send_mail_box;
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

	HAL_StatusTypeDef status;
	status = HAL_CAN_AddTxMessage(&GIMBAL_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
	if (status != HAL_OK)
	{
		// can发送失败送入can发送队列中
		CAN_TxQueue_Push(&gimbal_tx_message, gimbal_can_send_data);
	}
}

void CAN_Shoot_CMD(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4) //-30000,+30000
{
	CAN_TxHeaderTypeDef shoot_tx_message;
	uint8_t shoot_can_send_data[8];
	uint32_t send_mail_box;
	shoot_tx_message.StdId = CAN_SHOOT_ALL_ID;
	shoot_tx_message.IDE = CAN_ID_STD;
	shoot_tx_message.RTR = CAN_RTR_DATA;
	shoot_tx_message.DLC = 0x08;
	shoot_can_send_data[0] = motor1 >> 8;
	shoot_can_send_data[1] = motor1;
	shoot_can_send_data[2] = motor2 >> 8;
	shoot_can_send_data[3] = motor2;
	shoot_can_send_data[4] = motor3 >> 8;
	shoot_can_send_data[5] = motor3;
	shoot_can_send_data[6] = motor4 >> 8;
	shoot_can_send_data[7] = motor4;

	HAL_CAN_AddTxMessage(&SHOOT_CAN, &shoot_tx_message, shoot_can_send_data, &send_mail_box);
}
