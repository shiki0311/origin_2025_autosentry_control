#include "bsp_can.h"
#include "main.h"
#include "Chassis_Task.h"
#include "Gimbal_Task.h"
#include "Shoot_Task.h"
#include "detect_task.h"
#include "bsp_cap.h"

#define P_MIN -3.1415926f
#define P_MAX 3.1415926f
#define V_MIN -30.0f
#define V_MAX 30.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -10.0f
#define T_MAX 10.0f

#define  DM4310_RecID 0x00

#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

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


    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

uint8_t rx_data2[8];
uint8_t rx_data[8];
motor_measure_t motor_measure_chassis[4];
motor_measure_t motor_measure_gimbal[2];
motor_measure_t motor_measure_shoot[2];
DM_motor_data_t DM_pitch_motor_data;  //¥Ô√Îpitch

uint32_t id=0;
bool_t flag_code[5]={1,1,1,1,1};
int32_t dial_angle=0;
CAN_RxHeaderTypeDef rx_header;
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if(hcan==&hcan1)
	{
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

		if(rx_header.StdId!=CAN_3508_M1_ID&&rx_header.StdId!=CAN_3508_M2_ID&&rx_header.StdId!=CAN_3508_M3_ID&&rx_header.StdId!=CAN_3508_M4_ID)
			id=rx_header.StdId;
		
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
				
				int16_t temp1=motor_measure_chassis[i].ecd-motor_measure_chassis[i].last_ecd;
				
				int16_t temp2=temp1+(temp1<0?8192:-8192);
				
				motor_measure_chassis[i].code+=abs(temp2)<abs(temp1)?temp2:temp1;
				
				if(flag_code[i]==0)
				{
					motor_measure_chassis[i].code=0;
					flag_code[i]=1;
						}	
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
			case CAN_2006_M1_ID:
			{
				get_motor_measure(&motor_measure_shoot[0], rx_data);
				if(motor_measure_shoot[0].ecd-motor_measure_shoot[0].last_ecd>4096)
					dial_angle+=-8192+motor_measure_shoot[0].ecd-motor_measure_shoot[0].last_ecd;		
				else if(motor_measure_shoot[0].ecd-motor_measure_shoot[0].last_ecd<-4096)
					dial_angle+=8192+motor_measure_shoot[0].ecd-motor_measure_shoot[0].last_ecd;
				else
					dial_angle+=motor_measure_shoot[0].ecd-motor_measure_shoot[0].last_ecd;
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
	if(hcan==&hcan2)
	{
		CAN_RxHeaderTypeDef rx_header2;
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header2, rx_data2);
		switch (rx_header2.StdId)
		{		
			case DM4310_RecID:
			{
				DM_pitch_motor_data.id = (rx_data2[0])&0x0F;
				DM_pitch_motor_data.state = (rx_data2[0])>>4;
				DM_pitch_motor_data.p_int=(rx_data2[1]<<8)|rx_data2[2];
				DM_pitch_motor_data.v_int=(rx_data2[3]<<4)|(rx_data2[4]>>4);
				DM_pitch_motor_data.t_int=((rx_data2[4]&0xF)<<8)|rx_data2[5];
				DM_pitch_motor_data.pos = uint_to_float(DM_pitch_motor_data.p_int, P_MIN, P_MAX, 16)*57.3248408; // (-3.1415926,3.1415926)
				DM_pitch_motor_data.vel = uint_to_float(DM_pitch_motor_data.v_int, V_MIN, V_MAX, 12);
				DM_pitch_motor_data.toq = uint_to_float(DM_pitch_motor_data.t_int, T_MIN, T_MAX, 12);  // (-18.0,18.0)
				DM_pitch_motor_data.Tmos = (float)(rx_data2[6]);
				DM_pitch_motor_data.Tcoil = (float)(rx_data2[7]);
			}
		}
	}
}

float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

int float_to_uint(float x, float x_min, float x_max, int bits)
{
	float span = x_max - x_min;
	float offset = x_min;
	return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}