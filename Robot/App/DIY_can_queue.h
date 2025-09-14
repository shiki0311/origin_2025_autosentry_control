/***************************************************************************************************
 * @file: DIY_can_queue.h
 * @author: Shiki
 * @date: 2025.4
 * @brief: Shiki年少无知的时候手搓的循环队列，用于存储CAN消息，现已弃用，改用FreeRTOS的队列
 * @attention:
 ***************************************************************************************************/

#ifndef __DIY_CAN_QUEUE_H
#define __DIY_CAN_QUEUE_H

#include "bsp_can.h"
#include "can.h"

#define TX_QUEUE_SIZE 64

typedef struct
{
    CanTxMsgTypeDef can_msg_buffer[TX_QUEUE_SIZE];
    uint8_t head;
    uint8_t tail;
    uint8_t element_number;
} CanTxQueueTypeDef;

void CAN_TxQueue_Init();

#endif