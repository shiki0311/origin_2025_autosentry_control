/********************************************************************************************************************************************
 * @file: bsp_can.c
 * @author: Shiki
 * @date: 2025.7.12
 * @brief:	哨兵2025赛季CAN总线支持包，实现CAN接收和发送函数.
 * *******************************************************************************************************************************************
 * @attention: 1.哨兵can报文发送的流程是在各个task计算出需要向电机或其他设备（超电）需要发送的数据后，在xxx_task.c中调用
 *             Allocate_Can_Buffer（）或者Ctrl_DM_Motor()。调用规则为如果是向达秒电机发送can报文，则调用Ctrl_DM_Motor()，
 *             其他直接调用Allocate_Can_Buffer()。这两个函数的最终目的是将各个task要发送的can报文填充入对应的can报文缓冲区（CanTxMsgTypeDef类型的全局结构体变量)
 *             最后can报文会在定时器中断中统一发送。
 *
 *             2.如果要控制每一种can报文的发送频率，可以在定时器中断回调函数修改频率。
 *
 *			   3.如果can报文因为没有空闲邮箱而发送失败，会送入基于freertos创建的队列在有空闲邮箱时尝试重新发送。
 **********************************************************************************************************************************************/

#include "bsp_can.h"
#include "bsp_cap.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "main.h"
#include "detect_task.h"
#include "user_common_lib.h"
#include "string.h"

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
#define CAN_6020_YAW_ID 0x205

#define CAN_CAP_RX_ID 0x130

#define CAN_3508_CHASSIS_MOTOR1_ID 0x201
#define CAN_3508_CHASSIS_MOTOR2_ID 0x202
#define CAN_3508_CHASSIS_MOTOR3_ID 0x203
#define CAN_3508_CHASSIS_MOTOR4_ID 0x204

#define CAN_3508_FRIC_MOTOR1_ID 0x207
#define CAN_3508_FRIC_MOTOR2_ID 0x208
#define CAN_2006_DIAL_MOTOR_ID 0x206

#define CAN_GIMBAL_YAW_ID 0x1FF
#define CAN_CAP_TX_ID 0x140
#define CAN_CHASSIS_ALL_ID 0x200
#define CAN_SHOOT_ALL_ID 0x1FF

#define SHOOT_CAN hcan2
#define GIMBAL_PITCH_CAN hcan2
#define GIMBAL_YAW_CAN hcan1
#define CHASSIS_CAN hcan1
#define CAP_CAN hcan1


#define TIM_DIV2 1

#define CAN_TX_QUEUE_LENGTH 64

#define get_motor_measure(ptr, data)                                   \
	{                                                                  \
		(ptr)->last_ecd = (ptr)->ecd;                                  \
		(ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);           \
		(ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);     \
		(ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]); \
		(ptr)->temperate = (data)[6];                                  \
	}

motor_measure_t motor_measure_chassis[4];
motor_measure_t motor_measure_gimbal[2];
motor_measure_t motor_measure_shoot[3];
DM_motor_data_t DM_pitch_motor_data = {0};
int32_t dial_angle = 0;
CAN_RxHeaderTypeDef rx_header; // debug用，看can接收正不正常
/*********************************************CAN发送队列*********************************************************************/
QueueHandle_t CAN1_resend_queue; // CAN1消息队列句柄,此队列用于储存CAN1第一次发送失败的消息
QueueHandle_t CAN2_resend_queue; // CAN2消息队列句柄，此队列用于储存CAN2第一次发送失败的消息

#define SHOOT_CAN_RESEND_QUEUE CAN2_resend_queue
#define GIMBAL_PITCH_CAN_RESEND_QUEUE CAN2_resend_queue
#define GIMBAL_YAW_CAN_RESEND_QUEUE CAN1_resend_queue
#define CHASSIS_CAN_RESEND_QUEUE CAN1_resend_queue
#define CAP_CAN_RESEND_QUEUE CAN1_resend_queue
/*********************************************CAN发送缓冲区*********************************************************************/
CanTxMsgTypeDef cap_send_buffer;		  // 超电can报文全局缓冲区
CanTxMsgTypeDef chassis_send_buffer;	  // 底盘轮电机can报文全局缓冲区
CanTxMsgTypeDef gimbal_yaw_send_buffer;	  // yaw轴6020 can报文全局缓冲区
CanTxMsgTypeDef gimbal_pitch_send_buffer; // pitch轴达秒 can报文全局缓冲区
CanTxMsgTypeDef shoot_send_buffer;		  // 发射机构can报文全局缓冲区

/**
 * @description: 配置can过滤器，开启can外设以及需要的中断
 * @return 无
 */
void Can_Filter_Init(void)
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
	// HAL_CAN_ActivateNotification(&hcan1, CAN_IT_TX_MAILBOX_EMPTY);

	can_filter_st.SlaveStartFilterBank = 14;
	can_filter_st.FilterBank = 14;
	HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
	HAL_CAN_Start(&hcan2);
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/**
 * @description: CAN缓冲区初始化
 * @return {*} 无
 */
void Can_Buffer_Init(void)
{
	// 定义缓冲区与对应StdId的映射关系（数组批量处理）
	struct
	{
		CanTxMsgTypeDef *buffer;
		uint32_t stdId;
	} buffer_list[] = {
		{&cap_send_buffer, CAN_CAP_TX_ID},
		{&chassis_send_buffer, CAN_CHASSIS_ALL_ID},
		{&gimbal_yaw_send_buffer, CAN_GIMBAL_YAW_ID},
		{&gimbal_pitch_send_buffer, GIMBAL_PITCH_DM_SendID},
		{&shoot_send_buffer, CAN_SHOOT_ALL_ID}};

	// 遍历数组，批量初始化所有缓冲区
	for (size_t i = 0; i < sizeof(buffer_list) / sizeof(buffer_list[0]); i++)
	{
		buffer_list[i].buffer->tx_header.IDE = CAN_ID_STD;					  // 标准帧
		buffer_list[i].buffer->tx_header.RTR = CAN_RTR_DATA;				  // 数据帧
		buffer_list[i].buffer->tx_header.DLC = 0x08;						  // 数据长度8字节
		buffer_list[i].buffer->tx_header.StdId = buffer_list[i].stdId;		  // CAN Id
		memset(buffer_list[i].buffer->data, 0, sizeof(buffer_list[i].buffer->data)); // 数据缓冲区清零
	}
}

/**
 * @description: 基于freertos创建CAN发送队列
 * @return 无
 */
void Create_Can_Send_Queues()
{
	taskENTER_CRITICAL(); // 进入临界区

	CAN1_resend_queue = xQueueCreate(CAN_TX_QUEUE_LENGTH, sizeof(CanTxMsgTypeDef));
	CAN2_resend_queue = xQueueCreate(CAN_TX_QUEUE_LENGTH, sizeof(CanTxMsgTypeDef));

	taskEXIT_CRITICAL(); // 退出临界区

	if (CAN1_resend_queue == NULL || CAN2_resend_queue == NULL)

		Error_Handler();
}
/*********************************************CAN接收函数*********************************************************************/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	uint8_t rx_data[8];
	if (hcan == &hcan1)
	{
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

		switch (rx_header.StdId)
		{
		case CAN_3508_CHASSIS_MOTOR1_ID:
		case CAN_3508_CHASSIS_MOTOR2_ID:
		case CAN_3508_CHASSIS_MOTOR3_ID:
		case CAN_3508_CHASSIS_MOTOR4_ID:
		{
			static uint8_t i = 0;
			i = rx_header.StdId - CAN_3508_CHASSIS_MOTOR1_ID;
			get_motor_measure(&motor_measure_chassis[i], rx_data);
			int16_t temp1 = motor_measure_chassis[i].ecd - motor_measure_chassis[i].last_ecd;
			int16_t temp2 = temp1 + (temp1 < 0 ? 8192 : -8192);
			motor_measure_chassis[i].code += abs(temp2) < abs(temp1) ? temp2 : temp1;
			detect_hook(CHASSIS_MOTOR1_TOE + i);

			break;
		}
		case CAN_6020_YAW_ID:
		{
			static uint8_t i = 0;
			i = rx_header.StdId - CAN_6020_YAW_ID;

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
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
		switch (rx_header.StdId)
		{
		case DM4310_RecID:
		{
			DM_pitch_motor_data.id = (rx_data[0]) & 0x0F;
			DM_pitch_motor_data.state = (rx_data[0]) >> 4;
			DM_pitch_motor_data.p_int = (rx_data[1] << 8) | rx_data[2];
			DM_pitch_motor_data.v_int = (rx_data[3] << 4) | (rx_data[4] >> 4);
			DM_pitch_motor_data.t_int = ((rx_data[4] & 0xF) << 8) | rx_data[5];
			DM_pitch_motor_data.pos = uint_to_float(DM_pitch_motor_data.p_int, -12.5, 12.5, 16) * 57.3248408; // (-3.1415926,3.1415926)
			DM_pitch_motor_data.vel = uint_to_float(DM_pitch_motor_data.v_int, V_MIN, V_MAX, 12);
			DM_pitch_motor_data.toq = uint_to_float(DM_pitch_motor_data.t_int, T_MIN, T_MAX, 12); // (-18.0,18.0)
			DM_pitch_motor_data.Tmos = (float)(rx_data[6]);
			DM_pitch_motor_data.Tcoil = (float)(rx_data[7]);

			break;
		}
		case CAN_2006_DIAL_MOTOR_ID:
		{
			get_motor_measure(&motor_measure_shoot[2], rx_data);
			if (motor_measure_shoot[2].ecd - motor_measure_shoot[2].last_ecd > 4096)
				dial_angle += -8192 + motor_measure_shoot[2].ecd - motor_measure_shoot[2].last_ecd;
			else if (motor_measure_shoot[2].ecd - motor_measure_shoot[2].last_ecd < -4096)
				dial_angle += 8192 + motor_measure_shoot[2].ecd - motor_measure_shoot[2].last_ecd;
			else
				dial_angle += motor_measure_shoot[2].ecd - motor_measure_shoot[2].last_ecd;

			break;
		}
		case CAN_3508_FRIC_MOTOR1_ID:
		case CAN_3508_FRIC_MOTOR2_ID:
		{
			static uint8_t i = 0;
			i = rx_header.StdId - CAN_3508_FRIC_MOTOR1_ID;

			get_motor_measure(&motor_measure_shoot[i], rx_data);
			break;
		}
		}
	}
}

/*********************************************************填充CAN消息缓冲区************************************************************************/
/**
 * @description: 将待发送的can报文送入对应的缓冲区，在CAN_TX_TimerIRQHandler()函数中将缓冲区中的can报文统一定时发送
 * @return 无
 * @param {int16_t} data1/2/3/4  标准can数据帧的数据域，一共八个字节，这边用int16_t接收是考虑兼容大疆系列电机can数据发送协议，对于别的can报文需要做一些类型转换处理
 * @param {CAN_TX_ID} can_id can报文id
 * @attention 将DM电机can数据帧送入缓冲区需调用Ctrl_DM_Motor（）函数，原因是达秒电机的can数据帧传输的数据有五个，需要先在Ctrl_DM_Motor（）进行处理后再在其内部调用Allocate_Can_Buffer（）
 */
void Allocate_Can_Buffer(int16_t data1, int16_t data2, int16_t data3, int16_t data4, CAN_CMD_ID can_cmd_id)
{
	CanTxMsgTypeDef *pTxMsg = NULL;						  // 声明缓冲区指针
	int16_t data_array[4] = {data1, data2, data3, data4}; // 数据打包成数组，方便循环处理

	// 根据命令ID，让指针指向对应的缓冲区
	switch (can_cmd_id)
	{
	case CAN_CHASSIS_CMD:
		pTxMsg = &chassis_send_buffer;
		break;
	case CAN_GIMBAL_YAW_CMD:
		pTxMsg = &gimbal_yaw_send_buffer;
		break;
	case CAN_GIMBAL_PITCH_CMD:
		pTxMsg = &gimbal_pitch_send_buffer;
		break;
	case CAN_SHOOT_CMD:
		pTxMsg = &shoot_send_buffer;
		break;
	case CAN_CAP_CMD:
		pTxMsg = &cap_send_buffer;
		break;
	default:
		return; // 无效命令，直接返回
	}

	// 统一处理数据填充（超电命令和其他命令分开处理）
	if (can_cmd_id == CAN_CAP_CMD)
	{
		// 处理CAP命令的特殊逻辑
		for (int i = 0; i < 4; i++)
		{
			uint16_t temp = data_array[i] * 100;
			pTxMsg->data[2 * i] = temp & 0xFF;			  // 低8位
			pTxMsg->data[2 * i + 1] = (temp >> 8) & 0xFF; // 高8位
		}
	}
	else
	{
		// 处理其他命令的通用逻辑（直接拆分int16_t）
		for (int i = 0; i < 4; i++)
		{
			pTxMsg->data[2 * i] = (data_array[i] >> 8) & 0xFF; // 高8位
			pTxMsg->data[2 * i + 1] = data_array[i] & 0xFF;	   // 低8位
		}
	}
}


void Ctrl_DM_Motor(uint16_t id, float _pos, float _vel, float _KP, float _KD, float _torq) // can2
{
	uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
	pos_tmp = float_to_uint(_pos, -12.5, 12.5, 16);
	vel_tmp = float_to_uint(_vel, -45, 45, 12);
	kp_tmp = float_to_uint(_KP, KP_MIN, KP_MAX, 12);
	kd_tmp = float_to_uint(_KD, KD_MIN, KD_MAX, 12);
	tor_tmp = float_to_uint(_torq, T_MIN, T_MAX, 12);

	uint16_t data1, data2, data3, data4;
	data1 = pos_tmp;
	data2 = ((vel_tmp >> 4) << 8) | ((vel_tmp & 0x0F) << 4 | (kp_tmp >> 8));
	data3 = ((kp_tmp & 0xFF) << 8) | (kd_tmp >> 4);
	data4 = ((kd_tmp & 0x0F) << 12) | tor_tmp;

	Allocate_Can_Buffer(data1, data2, data3, data4, CAN_GIMBAL_PITCH_CMD);
}


/*********************************************定时发送CAN报文的中断回调函数***********************************************************/
/**
 * @description: tim5定时器计数溢出中断回调函数，用于以固定频率发送can报文，函数体每2ms执行一次
 * @return {*}
 */
void CAN_TX_TimerIRQHandler()
{
	static uint8_t div = 0; // 在函数整体执行频率（500hz）的基础上分频，使不同can报文适配相应的不同频率
	uint32_t send_mail_box;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE; // 调度标志

	// 500hz 发送can报文
	if (HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_send_buffer.tx_header, chassis_send_buffer.data, &send_mail_box) != HAL_OK)
	{
		xQueueSendFromISR(CHASSIS_CAN_RESEND_QUEUE, &chassis_send_buffer, &xHigherPriorityTaskWoken); // 如果发送失败送入队列等待重新发送
	}
	if (HAL_CAN_AddTxMessage(&GIMBAL_YAW_CAN, &gimbal_yaw_send_buffer.tx_header, gimbal_yaw_send_buffer.data, &send_mail_box) != HAL_OK)
	{
		xQueueSendFromISR(GIMBAL_YAW_CAN_RESEND_QUEUE, &gimbal_yaw_send_buffer, &xHigherPriorityTaskWoken); // 如果发送失败送入队列等待重新发送
	}
	if (HAL_CAN_AddTxMessage(&GIMBAL_PITCH_CAN, &gimbal_pitch_send_buffer.tx_header, gimbal_pitch_send_buffer.data, &send_mail_box) != HAL_OK)
	{
		xQueueSendFromISR(GIMBAL_PITCH_CAN_RESEND_QUEUE, &gimbal_pitch_send_buffer, &xHigherPriorityTaskWoken); // 如果发送失败送入队列等待重新发送
	}
	if (HAL_CAN_AddTxMessage(&SHOOT_CAN, &shoot_send_buffer.tx_header, shoot_send_buffer.data, &send_mail_box) != HAL_OK)
	{
		xQueueSendFromISR(SHOOT_CAN_RESEND_QUEUE, &shoot_send_buffer, &xHigherPriorityTaskWoken); // 如果发送失败送入队列等待重新发送
	}

	// // 250hz 发送can报文
	if (div == TIM_DIV2)
	{
		if (HAL_CAN_AddTxMessage(&CAP_CAN, &cap_send_buffer.tx_header, cap_send_buffer.data, &send_mail_box) != HAL_OK)
		{
			xQueueSendFromISR(CAP_CAN_RESEND_QUEUE, &cap_send_buffer, &xHigherPriorityTaskWoken); // 如果发送失败送入队列等待重新发送
		}
	}

	div++;
	div = div % 2;

	if (xHigherPriorityTaskWoken == pdTRUE)
	{
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken); // 触发任务切换
	}
}
/*********************************************定时检测需要重新发送的CAN消息的中断回调函数***********************************************************/

/**
 * @description: tim3定时器计数溢出中断回调函数，用于检测CAN1和CAN2有无第一次发送失败的消息，如有尝试重发。每10ms执行一次
 * @return {*}
 */
void CAN_Resend_Timer_IRQHandler()
{
	CanTxMsgTypeDef resend_msg;
	uint32_t send_mail_box;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE; // 调度标志

	// 尝试发送队列中的消息
	while ((HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0 && uxQueueMessagesWaitingFromISR(CAN1_resend_queue) > 0) || (HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) > 0 && uxQueueMessagesWaitingFromISR(CAN2_resend_queue) > 0))
	{
		if ((HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0 && uxQueueMessagesWaitingFromISR(CAN1_resend_queue) > 0))
		{
			xQueueReceiveFromISR(CAN1_resend_queue, &resend_msg, &xHigherPriorityTaskWoken);
			HAL_CAN_AddTxMessage(&hcan1, &resend_msg.tx_header, resend_msg.data, &send_mail_box);
		}

		if ((HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) > 0 && uxQueueMessagesWaitingFromISR(CAN2_resend_queue) > 0))
		{
			xQueueReceiveFromISR(CAN2_resend_queue, &resend_msg, &xHigherPriorityTaskWoken);
			HAL_CAN_AddTxMessage(&hcan2, &resend_msg.tx_header, resend_msg.data, &send_mail_box);
		}
	}

	if (xHigherPriorityTaskWoken == pdTRUE)
	{
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken); // 触发任务切换
	}
}

/*********************************************DM使能和失能函数***********************************************************/
void enable_DM(uint8_t id, uint8_t ctrl_mode)
{
	uint8_t TX_Data[8];
	uint32_t send_mail_box;
	CAN_TxHeaderTypeDef Tx_Msg;

	if (ctrl_mode == 1)
		Tx_Msg.StdId = 0x000 + GIMBAL_PITCH_DM_SendID;
	else if (ctrl_mode == 2)
		Tx_Msg.StdId = 0x100 + GIMBAL_PITCH_DM_SendID;
	else if (ctrl_mode == 3)
		Tx_Msg.StdId = 0x200 + GIMBAL_PITCH_DM_SendID;
	Tx_Msg.IDE = CAN_ID_STD;
	Tx_Msg.RTR = CAN_RTR_DATA;
	Tx_Msg.DLC = 8;

	TX_Data[0] = 0xff;
	TX_Data[1] = 0xff;
	TX_Data[2] = 0xff;
	TX_Data[3] = 0xff;
	TX_Data[4] = 0xff;
	TX_Data[5] = 0xff;
	TX_Data[6] = 0xff;
	TX_Data[7] = 0xfc;

	HAL_CAN_AddTxMessage(&hcan2, &Tx_Msg, TX_Data, &send_mail_box);
}

void disable_DM(uint8_t id, uint8_t ctrl_mode)
{
	uint8_t TX_Data[8];
	uint32_t send_mail_box;
	CAN_TxHeaderTypeDef Tx_Msg;

	if (ctrl_mode == 1)
		Tx_Msg.StdId = 0x000 + GIMBAL_PITCH_DM_SendID;
	else if (ctrl_mode == 2)
		Tx_Msg.StdId = 0x100 + GIMBAL_PITCH_DM_SendID;
	else if (ctrl_mode == 3)
		Tx_Msg.StdId = 0x200 + GIMBAL_PITCH_DM_SendID;

	Tx_Msg.IDE = CAN_ID_STD;
	Tx_Msg.RTR = CAN_RTR_DATA;
	Tx_Msg.DLC = 8;

	TX_Data[0] = 0xff;
	TX_Data[1] = 0xff;
	TX_Data[2] = 0xff;
	TX_Data[3] = 0xff;
	TX_Data[4] = 0xff;
	TX_Data[5] = 0xff;
	TX_Data[6] = 0xff;
	TX_Data[7] = 0xfd;

	HAL_CAN_AddTxMessage(&hcan2, &Tx_Msg, TX_Data, &send_mail_box);
}

