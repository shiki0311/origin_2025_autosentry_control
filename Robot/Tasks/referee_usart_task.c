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
/* ����ϵͳ����˫������ */
uint8_t Referee_Buffer[2][REFEREE_USART_RX_BUF_LENGHT];

/* ����ϵͳ�������ݶ��� */
fifo_s_t Referee_FIFO;
uint8_t Referee_FIFO_Buffer[REFEREE_FIFO_BUF_LENGTH];

/* protocol�������ṹ�� */
unpack_data_t Referee_Unpack_OBJ;

/* ��̬UI���ݱ��� */
uint8_t UI_AutoAim_Flag = 0;    //�Ƿ��������־λ
float   UI_Kalman_Speed = 0;    //������Ԥ���ٶ�
float   UI_Gimbal_Pitch = 0.0f; //��̨Pitch��Ƕ�
float   UI_Gimbal_Yaw   = 0.0f; //��̨Yaw��Ƕ�
uint8_t UI_Capacitance  = 10;   //����ʣ������
uint8_t UI_fric_is_on   = 0;    //Ħ�����Ƿ���


uint8_t UI_Capacitance_Color;

uint16_t LENTH_REFEREE_BUF=0;

float UI_Capacitance_Value=100.00f;	//��������ʣ�������ٷֱ�

_Bool FrictionChange		=0;
_Bool FrictionFlag			=0;
_Bool AutoAimingChange	=0;
_Bool AutoAimingFlag		=0;
_Bool SpirallingChange	=0;
_Bool SpirallingFlag		=1;

void referee_usart_task(void const * argument)
{

	/* ����ϵͳ��ʼ�� */
	Referee_StructInit();
	Referee_UARTInit(Referee_Buffer[0], Referee_Buffer[1], REFEREE_USART_RX_BUF_LENGHT);
	Referee_FIFOInit(&Referee_FIFO, Referee_FIFO_Buffer, REFEREE_FIFO_BUF_LENGTH);
	vTaskDelay(111);
	
	while(1)
	{
		/* ��������ϵͳ���� */
		vTaskDelay(10);
		Referee_UnpackFifoData(&Referee_Unpack_OBJ, &Referee_FIFO);
		Sentry_PushUp_Cmd(&Sentry_Auto_Cmd_Send, Game_Robot_State.robot_id);
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
		
		/* ��ձ�־λ */
		__HAL_UART_CLEAR_IDLEFLAG(&Referee_UART);
		
		if ((Referee_UART.hdmarx->Instance->CR & DMA_SxCR_CT) == RESET)
		{
			/* ����DMA���л������� */
			__HAL_DMA_DISABLE(Referee_UART.hdmarx);
			Size = 512 - Referee_UART.hdmarx->Instance->NDTR;
//			Size =  Referee_UART.hdmarx->Instance->NDTR;
			LENTH_REFEREE_BUF=Size;
			Referee_UART.hdmarx->Instance->NDTR = REFEREE_USART_RX_BUF_LENGHT;
			Referee_UART.hdmarx->Instance->CR |= DMA_SxCR_CT;
			__HAL_DMA_ENABLE(Referee_UART.hdmarx);
			
			/* ���������ӵ����� */
			fifo_s_puts(&Referee_FIFO, (char *)Referee_Buffer[0], Size);
		}
		else
		{
			/* ����DMA���л������� */
			__HAL_DMA_DISABLE(Referee_UART.hdmarx);
//			Size = Referee_UART.hdmarx->Instance->NDTR;
				Size = 512 - Referee_UART.hdmarx->Instance->NDTR;
			LENTH_REFEREE_BUF=Size;
			Referee_UART.hdmarx->Instance->NDTR = REFEREE_USART_RX_BUF_LENGHT;
			Referee_UART.hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
			__HAL_DMA_ENABLE(Referee_UART.hdmarx);
			
			/* ���������ӵ����� */
			fifo_s_puts(&Referee_FIFO, (char *)Referee_Buffer[1], Size);
		}
		return;
	}
	
	HAL_UART_IRQHandler(&Referee_UART);
}
