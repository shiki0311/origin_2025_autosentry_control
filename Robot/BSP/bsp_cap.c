#include "bsp_cap.h"
#include "Chassis_Task.h"

cap_measure_t cap_data;
uint8_t cap_recieve_flag=0;
void update_cap(uint8_t * data)
{
    cap_data.cap_per=(float)((data)[1]<<8|(data)[0])/32768.0f;
    cap_data.chassis_power=(float)((data)[3]<<8|(data)[2])/100.0f;
    cap_recieve_flag=1;
    uint16_t * pPowerdata = (uint16_t *) data;
}