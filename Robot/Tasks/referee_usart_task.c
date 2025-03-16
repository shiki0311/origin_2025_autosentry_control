#include "referee_usart_task.h"
#include "usart.h"
#include "crcs.h"
#include "fifo.h"
#include "protocol.h"
#include "referee.h"
#include "shoot_task.h"
#include "Gimbal_Task.h"
#include "Chassis_Task.h"
#include "bsp_cap.h"
#include "bsp_can.h"
/* Private define ------------------------------------------------------------*/
#define Referee_FIFOInit fifo_s_init
#define Max(a,b) ((a) > (b) ? (a) : (b))

#define ROBOT_ID_MYSELF Robot_ID_Red_Hero

/* Private variables ---------------------------------------------------------*/
/* 裁判系统串口双缓冲区 */
uint8_t Referee_Buffer[2][REFEREE_USART_RX_BUF_LENGHT];

/* 裁判系统接收数据队列 */
fifo_s_t Referee_FIFO;
uint8_t Referee_FIFO_Buffer[REFEREE_FIFO_BUF_LENGTH];

/* protocol解析包结构体 */
unpack_data_t Referee_Unpack_OBJ;

/* 动态UI数据变量 */
uint8_t UI_AutoAim_Flag = 0;    //是否开启自瞄标志位
float   UI_Kalman_Speed = 0;    //卡尔曼预测速度
float   UI_Gimbal_Pitch = 0.0f; //云台Pitch轴角度
float   UI_Gimbal_Yaw   = 0.0f; //云台Yaw轴角度
uint8_t UI_Capacitance  = 10;   //电容剩余容量
uint8_t UI_fric_is_on   = 0;    //摩擦轮是否开启


uint8_t UI_Capacitance_Color;

uint16_t LENTH_REFEREE_BUF=0;

float UI_Capacitance_Value=100.00f;	//超级电容剩余容量百分比

_Bool FrictionChange		=0;
_Bool FrictionFlag			=0;
_Bool AutoAimingChange	=0;
_Bool AutoAimingFlag		=0;
_Bool SpirallingChange	=0;
_Bool SpirallingFlag		=1;

void referee_usart_task(void const * argument)
{

	/* 裁判系统初始化 */
	Referee_StructInit();
	Referee_UARTInit(Referee_Buffer[0], Referee_Buffer[1], REFEREE_USART_RX_BUF_LENGHT);
	Referee_FIFOInit(&Referee_FIFO, Referee_FIFO_Buffer, REFEREE_FIFO_BUF_LENGTH);
	vTaskDelay(111);
	
	while(1)
	{
		/* 解析裁判系统数据 */
		vTaskDelay(10);
		Referee_UnpackFifoData(&Referee_Unpack_OBJ, &Referee_FIFO);
		
		CAN_Cap_CMD(Game_Robot_State.chassis_power_limit,init_chassis_power,Power_Heat_Data.buffer_energy,0);
		UI_Gimbal_Pitch=DM_pitch_motor_data.INS_angle; //从gimbal_m6020[1]改成damiao的
		UI_fric_is_on = fric_state;
		UI_Capacitance=cap_data.cap_per * 100;
		
	
	}
}

void USART6_IRQHandler_0(void)
{
    static volatile uint8_t res;
    if(USART6->SR & UART_FLAG_IDLE)
    {
        __HAL_UART_CLEAR_PEFLAG(&huart6);

        static uint16_t this_time_rx_len = 0;

        if ((huart6.hdmarx->Instance->CR & DMA_SxCR_CT) == RESET)
        {
            __HAL_DMA_DISABLE(huart6.hdmarx);
            this_time_rx_len = REFEREE_USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
            __HAL_DMA_SET_COUNTER(huart6.hdmarx, REFEREE_USART_RX_BUF_LENGHT);
            huart6.hdmarx->Instance->CR |= DMA_SxCR_CT;
            __HAL_DMA_ENABLE(huart6.hdmarx);
            fifo_s_puts(&Referee_FIFO, (char*)Referee_Buffer[0], this_time_rx_len);
        }
        else
        {
            __HAL_DMA_DISABLE(huart6.hdmarx);
            this_time_rx_len = REFEREE_USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
						LENTH_REFEREE_BUF=this_time_rx_len;
            __HAL_DMA_SET_COUNTER(huart6.hdmarx, REFEREE_USART_RX_BUF_LENGHT);
            huart6.hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
            __HAL_DMA_ENABLE(huart6.hdmarx);
            fifo_s_puts(&Referee_FIFO, (char*)Referee_Buffer[1], this_time_rx_len);
        }
    }
}

void Referee_IRQHandler(void)
{
	if(Referee_UART.Instance->SR & UART_FLAG_RXNE)
	{
	  __HAL_UART_CLEAR_IDLEFLAG(&Referee_UART);
	}
	else if(Referee_UART.Instance->SR & UART_FLAG_IDLE)
	{
		static uint16_t Size = 0;
		
		/* 清空标志位 */
		__HAL_UART_CLEAR_IDLEFLAG(&Referee_UART);
		
		if ((Referee_UART.hdmarx->Instance->CR & DMA_SxCR_CT) == RESET)
		{
			/* 重置DMA并切换缓冲区 */
			__HAL_DMA_DISABLE(Referee_UART.hdmarx);
			Size = 512 - Referee_UART.hdmarx->Instance->NDTR;
//			Size =  Referee_UART.hdmarx->Instance->NDTR;
			LENTH_REFEREE_BUF=Size;
			Referee_UART.hdmarx->Instance->NDTR = REFEREE_USART_RX_BUF_LENGHT;
			Referee_UART.hdmarx->Instance->CR |= DMA_SxCR_CT;
			__HAL_DMA_ENABLE(Referee_UART.hdmarx);
			
			/* 将数据添加到队列 */
			fifo_s_puts(&Referee_FIFO, (char *)Referee_Buffer[0], Size);
		}
		else
		{
			/* 重置DMA并切换缓冲区 */
			__HAL_DMA_DISABLE(Referee_UART.hdmarx);
//			Size = Referee_UART.hdmarx->Instance->NDTR;
				Size = 512 - Referee_UART.hdmarx->Instance->NDTR;
			LENTH_REFEREE_BUF=Size;
			Referee_UART.hdmarx->Instance->NDTR = REFEREE_USART_RX_BUF_LENGHT;
			Referee_UART.hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
			__HAL_DMA_ENABLE(Referee_UART.hdmarx);
			
			/* 将数据添加到队列 */
			fifo_s_puts(&Referee_FIFO, (char *)Referee_Buffer[1], Size);
		}
		return;
	}
	
	HAL_UART_IRQHandler(&Referee_UART);
}
