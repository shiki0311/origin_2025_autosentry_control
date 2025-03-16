/**
  ******************************************************************************
  * @file    refereetask.c
  * @author  Karolance Future
  * @version V1.1.0
  * @date    2022/03/21
  * @brief   
  ******************************************************************************
  * @attention
	*
	* @notes
  *
  ******************************************************************************
  */
	
/* Private includes ----------------------------------------------------------*/
#include "refereetask.h"
#include "usart.h"
#include "crcs.h"
#include "fifo.h"
#include "protocol.h"
#include "referee.h"
#include "arm_math.h"

/* Private define ------------------------------------------------------------*/
#define Referee_FIFOInit fifo_s_init
#define Robot_ID_Current Robot_ID_Blue_Hero
#define Max(a,b) ((a) > (b) ? (a) : (b))

/* Private variables ---------------------------------------------------------*/
TaskHandle_t RefereeTask_Handle;

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

/* 中央标尺高度变量 */
uint16_t y01 = 380;
uint16_t y02 = 330;
uint16_t y03 = 280;
uint16_t y04 = 230;

/* Private user Functions ----------------------------------------------------*/
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
			Size = REFEREE_USART_RX_BUF_LENGHT - Referee_UART.hdmarx->Instance->NDTR;
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
			Size = REFEREE_USART_RX_BUF_LENGHT - Referee_UART.hdmarx->Instance->NDTR;
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

/* Private user Task ---------------------------------------------------------*/
void RefereeTask(void const * argument)
{
	/* 动态UI控制变量 */
	uint16_t UI_PushUp_Counter = 261;
	float    Capacitance_X;
	
	/* 裁判系统初始化 */
	Referee_StructInit();
	Referee_UARTInit(Referee_Buffer[0], Referee_Buffer[1], REFEREE_USART_RX_BUF_LENGHT);
	Referee_FIFOInit(&Referee_FIFO, Referee_FIFO_Buffer, REFEREE_FIFO_BUF_LENGTH);
	vTaskDelay(300);
	
	/* new UI */
	while(1)
	{
		/* 解析裁判系统数据 */
		vTaskDelay(10);
		Referee_UnpackFifoData(&Referee_Unpack_OBJ, &Referee_FIFO);
		
		/* UI更新 */
		UI_PushUp_Counter++;
		if(UI_PushUp_Counter % 301 == 0) //静态UI预绘制 中央标尺1
		{
			UI_Draw_Line(&UI_Graph7.Graphic[0], "001", UI_Graph_Add, 0, UI_Color_Green, 1,  840,   y01,  920,   y01); //第一行左横线
			UI_Draw_Line(&UI_Graph7.Graphic[1], "002", UI_Graph_Add, 0, UI_Color_Green, 1,  950,   y01,  970,   y01); //第一行十字横
			UI_Draw_Line(&UI_Graph7.Graphic[2], "003", UI_Graph_Add, 0, UI_Color_Green, 1, 1000,   y01, 1080,   y01); //第一行右横线
			UI_Draw_Line(&UI_Graph7.Graphic[3], "004", UI_Graph_Add, 0, UI_Color_Green, 1,  960,y01-10,  960,y01+10); //第一行十字竖
			UI_Draw_Line(&UI_Graph7.Graphic[4], "005", UI_Graph_Add, 0, UI_Color_Green, 1,  870,   y02,  930,   y02); //第二行左横线
			UI_Draw_Line(&UI_Graph7.Graphic[5], "006", UI_Graph_Add, 0, UI_Color_Green, 5,  959,   y02,  960,   y02); //第二行中心点
			UI_Draw_Line(&UI_Graph7.Graphic[6], "007", UI_Graph_Add, 0, UI_Color_Green, 1,  990,   y02, 1050,   y02); //第二行右横线
			UI_PushUp_Graphs(7, &UI_Graph7, Robot_ID_Current);
			continue;
		}
		if(UI_PushUp_Counter % 311 == 0) //静态UI预绘制 中央标尺2
		{
			UI_Draw_Line(&UI_Graph7.Graphic[0], "008", UI_Graph_Add, 0, UI_Color_Green, 1,  900,   y03,  940,   y03); //第三行左横线
			UI_Draw_Line(&UI_Graph7.Graphic[1], "009", UI_Graph_Add, 0, UI_Color_Green, 5,  959,   y03,  960,   y03); //第三行中心点
			UI_Draw_Line(&UI_Graph7.Graphic[2], "010", UI_Graph_Add, 0, UI_Color_Green, 1,  980,   y03, 1020,   y03); //第三行右横线
			UI_Draw_Line(&UI_Graph7.Graphic[3], "011", UI_Graph_Add, 0, UI_Color_Green, 1,  930,   y04,  950,   y04); //第四行左横线
			UI_Draw_Line(&UI_Graph7.Graphic[4], "012", UI_Graph_Add, 0, UI_Color_Green, 5,  959,   y04,  960,   y04); //第四行中心点
			UI_Draw_Line(&UI_Graph7.Graphic[5], "013", UI_Graph_Add, 0, UI_Color_Green, 1,  970,   y04,  990,   y04); //第四行右横线
			UI_Draw_Line(&UI_Graph7.Graphic[6], "014", UI_Graph_Add, 0, UI_Color_Green, 1,  960,y04-10,  960,y04-30); //第四行下竖线
			UI_PushUp_Graphs(7, &UI_Graph7, Robot_ID_Current);
			continue;
		}
		if(UI_PushUp_Counter % 321 == 0) //静态UI预绘制 小陀螺预警线
		{
			UI_Draw_Line(&UI_Graph5.Graphic[0], "101", UI_Graph_Add, 1, UI_Color_Yellow, 2,  630,   30,  780,  100);
			UI_Draw_Line(&UI_Graph5.Graphic[1], "102", UI_Graph_Add, 1, UI_Color_Yellow, 2,  780,  100,  930,  100);
			UI_Draw_Line(&UI_Graph5.Graphic[2], "103", UI_Graph_Add, 1, UI_Color_Yellow, 2,  990,  100, 1140,  100);
			UI_Draw_Line(&UI_Graph5.Graphic[3], "104", UI_Graph_Add, 1, UI_Color_Yellow, 2, 1140,  100, 1290,   30);
			UI_Draw_Line(&UI_Graph5.Graphic[4], "105", UI_Graph_Add, 1, UI_Color_Yellow, 5,  959,  100,  960,  100);
			UI_PushUp_Graphs(5, &UI_Graph5, Robot_ID_Current);
			continue;
		}
		if(UI_PushUp_Counter % 331 == 0) //动态UI预绘制 图形
		{
			UI_Draw_Float (&UI_Graph2.Graphic[0], "201", UI_Graph_Add, 2, UI_Color_Yellow, 22, 3, 3, 1355, 632, 0.000f);   //Pith轴角度
			UI_Draw_Line  (&UI_Graph2.Graphic[1], "202", UI_Graph_Add, 2, UI_Color_Orange, 20, 1829, 330, 1870, 334);      //电容容量
			UI_PushUp_Graphs(2, &UI_Graph2, Robot_ID_Current);
			continue;
		}
		if(UI_PushUp_Counter % 341 == 0) //动态UI预绘制 字符串1
		{
			UI_Draw_String(&UI_String.String,     "203", UI_Graph_Add, 2, UI_Color_Black,  22, 8, 3,  400, 632, "Fric OFF"); //摩擦轮是否开启
			UI_PushUp_String(&UI_String, Robot_ID_Current);
			continue;
		}
		if(UI_PushUp_Counter % 351 == 0) //动态UI更新 字符串1
		{
			if(UI_fric_is_on == 1) UI_Draw_String(&UI_String.String, "203", UI_Graph_Change, 2, UI_Color_Main,  22, 8, 3,  400, 632, "Fric  ON");
			if(UI_fric_is_on == 0) UI_Draw_String(&UI_String.String, "203", UI_Graph_Change, 2, UI_Color_Black, 22, 8, 3,  400, 632, "Fric OFF");
			UI_PushUp_String(&UI_String, Robot_ID_Current);
		}
		if(UI_PushUp_Counter % 10 == 0)  //动态UI更新 图形
		{
			/* Pitch轴当前角度 */
			UI_Draw_Float(&UI_Graph2.Graphic[0], "201", UI_Graph_Change, 2, UI_Color_Yellow, 22, 3, 3, 1355, 632, UI_Gimbal_Pitch);
			
			/* 超级电容容量 */
			UI_Capacitance = Max(UI_Capacitance, 30);
			Capacitance_X  = 1870.0f - 4.1f * UI_Capacitance;
			if(50 < UI_Capacitance && UI_Capacitance <= 100) UI_Draw_Line(&UI_Graph2.Graphic[1], "202", UI_Graph_Change, 2, UI_Color_Green , 20, Capacitance_X, 334, 1870, 334);
			if(35 < UI_Capacitance && UI_Capacitance <=  50) UI_Draw_Line(&UI_Graph2.Graphic[1], "202", UI_Graph_Change, 2, UI_Color_Yellow, 20, Capacitance_X, 334, 1870, 334);
			if(0  < UI_Capacitance && UI_Capacitance <=  35) UI_Draw_Line(&UI_Graph2.Graphic[1], "202", UI_Graph_Change, 2, UI_Color_Orange, 20, Capacitance_X, 334, 1870, 334);
			
			UI_PushUp_Graphs(2, &UI_Graph2, Robot_ID_Current);
			continue;
		}
	}
}

/* Private user debug --------------------------------------------------------*/
