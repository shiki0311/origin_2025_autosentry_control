/***************************************************************************************************
 * @file: DIY_can_queue.c
 * @author: Shiki
 * @date: 2025.4
 * @brief: Shiki年少无知的时候手搓的循环队列，用于存储CAN消息，现已弃用，改用FreeRTOS的队列
 * @attention:
 ***************************************************************************************************/


#include "DIY_can_queue.h"
#include "main.h"

CanTxQueueTypeDef can_tx_queue;

/*********************************************CAN发送循环队列函数库************************************************************/
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
    gimbal_can_send_data[4] = motor3 >> 8;
    gimbal_can_send_data[3] = motor2;
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

/************************************************备份，后面会删除 ******************************************/
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

void Ctrl_DM_Motor(uint16_t id, float _pos, float _vel, float _KP, float _KD, float _torq) // can2
{
    uint8_t TX_Data[8];
    uint32_t send_mail_box;
    CAN_TxHeaderTypeDef Tx_Msg;

    Tx_Msg.StdId = id;
    Tx_Msg.IDE = CAN_ID_STD;
    Tx_Msg.RTR = CAN_RTR_DATA;
    Tx_Msg.DLC = 8;

    uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
    pos_tmp = float_to_uint(_pos, -12.5, 12.5, 16);
    vel_tmp = float_to_uint(_vel, -45, 45, 12);
    kp_tmp = float_to_uint(_KP, KP_MIN, KP_MAX, 12);
    kd_tmp = float_to_uint(_KD, KD_MIN, KD_MAX, 12);
    tor_tmp = float_to_uint(_torq, T_MIN, T_MAX, 12);

    TX_Data[0] = (pos_tmp >> 8);
    TX_Data[1] = pos_tmp;
    TX_Data[2] = (vel_tmp >> 4);
    TX_Data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
    TX_Data[4] = kp_tmp;
    TX_Data[5] = (kd_tmp >> 4);
    TX_Data[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
    TX_Data[7] = tor_tmp;

    HAL_CAN_AddTxMessage(&GIMBAL_CAN, &Tx_Msg, TX_Data, &send_mail_box);
}

void enable_DM(uint8_t id, uint8_t ctrl_mode)
{
    uint8_t TX_Data[8];
    uint32_t send_mail_box;
    CAN_TxHeaderTypeDef Tx_Msg;

    if (ctrl_mode == 1)
        Tx_Msg.StdId = 0x000 + DM4310_SendID;
    else if (ctrl_mode == 2)
        Tx_Msg.StdId = 0x100 + DM4310_SendID;
    else if (ctrl_mode == 3)
        Tx_Msg.StdId = 0x200 + DM4310_SendID;
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
        Tx_Msg.StdId = 0x000 + DM4310_SendID;
    else if (ctrl_mode == 2)
        Tx_Msg.StdId = 0x100 + DM4310_SendID;
    else if (ctrl_mode == 3)
        Tx_Msg.StdId = 0x200 + DM4310_SendID;

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