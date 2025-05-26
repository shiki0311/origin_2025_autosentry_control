#include <math.h>
#include <stdio.h>
#include "Nmanifold_usbd_task.h"
#include "SolveTrajectory.h"

struct SolveTrajectoryParams st;
struct tar_pos tar_position[4]; //���ֻ���Ŀ�װ�װ�
float t = 0.5f; // ����ʱ�� �������������

float aim_x = 0, aim_y = 0, aim_z = 0; // aim point ��㣬������λ�����п��ӻ�
float pitch_tra = 0 , yaw_tra = 0; //pitch,yaw����
int idx = 0;
float distance_xy=0,distance_xyz=0;

void SolvaTrajectoryInit(void)
{
		st.k = 0.092;
    st.bullet_type =  BULLET_17;
    st.current_v = 28;
    st.current_pitch = 0;
    st.current_yaw = 0;

    // �Է�������������������ϵ�µ�xyz���� ����ʮ��Ϊ���յ�NUC����
//    st.xw = AutoAim_Data_Receive.xw; 
//    st.yw = AutoAim_Data_Receive.yw;
//    st.zw = AutoAim_Data_Receive.zw;
//    st.vxw = AutoAim_Data_Receive.vxw;
//    st.vyw = AutoAim_Data_Receive.vyw;
//    st.vzw = AutoAim_Data_Receive.vzw;
//    st.v_yaw = AutoAim_Data_Receive.v_yaw;      //����yawת��
//    st.tar_yaw = AutoAim_Data_Receive.tar_yaw;   //װ�װ�yawλ��
//    st.r1 = AutoAim_Data_Receive.r1;            //Ŀ�����ĵ�ǰ��װ�װ�ľ���
//    st.r2 = AutoAim_Data_Receive.r2;            //Ŀ�����ĵ�����װ�װ�ľ���
		
		distance_xy = sqrt((st.xw*st.xw)+(st.yw*st.yw)); // ˮƽ����
		distance_xyz = sqrt((st.xw*st.xw)+(st.yw*st.yw)+(st.zw*st.zw)); // ֱ�߾���
	
    st.dz = 0.1;            //��һ��װ�װ�����ڱ�����װ�װ�ĸ߶Ȳ�
    st.bias_time = 130;     //ƫ��ʱ��
    st.s_bias = 0.10133;    //ǹ��ǰ�Ƶľ���
    st.z_bias = 0.25265;    //yaw������ǹ��ˮƽ��Ĵ�ֱ����
    st.armor_id = ARMOR_INFANTRY3;
		
//		if(AutoAim_Data_Receive.armor_num == 3) st.armor_num = ARMOR_NUM_OUTPOST;
//		else if(AutoAim_Data_Receive.armor_num == 4) st.armor_num = ARMOR_NUM_NORMAL;
//		else if(AutoAim_Data_Receive.armor_num == 2) st.armor_num = ARMOR_NUM_BALANCE;
		
}

/*
@brief �����������������ģ��
@param s:m ����
@param v:m/s �ٶ�
@param angle:rad �Ƕ�
@return z:m
*/
float monoDirectionalAirResistanceModel(float s, float v, float angle)
{
    float z;
    t = (float)((exp(st.k * s) - 1) / (st.k * v * cos(angle))); // �����ŵķ���ʱ��
    if(t < 0)
    {
        //���س��������̣������������t��Ϊ����������t
        t = 0;
        return 0;
    }
    //zΪ����v��angleʱ�ĸ߶�
    z = (float)(v * sin(angle) * t - GRAVITY * t * t / 2);
    //printf("model %f %f\n", t, z);
    return z;
}

/*
@brief pitch�����
@param s:m ����
@param z:m �߶�
@param v:m/s
@return angle_pitch:rad
*/
float pitchTrajectoryCompensation(float s, float z, float v)
{
    float z_temp, z_actual, dz;
    float angle_pitch;
    int i = 0;
    z_temp = z;
    // iteration
    for (i = 0; i < 20; i++) // ��ʮ�ε����õ�
    {
        angle_pitch = atan2(z_temp, s); // rad
        z_actual = monoDirectionalAirResistanceModel(s, v, angle_pitch); // �½��ĸ߶�
        if(z_actual == 0)
        {
            angle_pitch = 0;
            break;
        }
        dz = 0.3*(z - z_actual);
        z_temp = z_temp + dz;
 
        if (fabsf(dz) < 0.00001)
        {
            break;
        }
    }
    return angle_pitch;
}

void autoSolveTrajectory(void)
{

    // ����Ԥ��
  float timeDelay = st.bias_time/1000.0 + t; 
  st.tar_yaw += st.v_yaw * timeDelay; // 

    //�����Ŀ�װ�װ��λ��
    //װ�װ�id˳��,���Ŀ�װ�װ�Ϊ��,��ʱ����
    //      2
    //   3     1
    //      0
	int use_1 = 1;
	int i = 0;
	int idx = 0; // ѡ���װ�װ�
	
	//armor_num = ARMOR_NUM_BALANCE ƽ�ⲽ��
	if (st.armor_num == ARMOR_NUM_BALANCE) 
	{
			for (i = 0; i<2; i++) {
					float tmp_yaw = st.tar_yaw + i * PI;
					float r = st.r1;
					tar_position[i].x = st.xw - r*cos(tmp_yaw);
					tar_position[i].y = st.yw - r*sin(tmp_yaw);
					tar_position[i].z = st.zw;
					tar_position[i].yaw = tmp_yaw;
			}

			float yaw_diff_min = fabsf(yaw_tra - tar_position[0].yaw);

			//ֻ���ж�����װ�װ弴��
			float temp_yaw_diff = fabsf(yaw_tra - tar_position[1].yaw);
			if (temp_yaw_diff < yaw_diff_min)
			{
					yaw_diff_min = temp_yaw_diff;
					idx = 1;
			}
	} 
	else if (st.armor_num == ARMOR_NUM_OUTPOST)  //ǰ��վ
	{  
			for (i = 0; i<3; i++) {
					float tmp_yaw = st.tar_yaw + i * 2.0 * PI/3.0;  // 2/3PI
					float r =  (st.r1 + st.r2)/2;   //???r1=r2 ???????
					tar_position[i].x = st.xw - r*cos(tmp_yaw);
					tar_position[i].y = st.yw - r*sin(tmp_yaw);
					tar_position[i].z = st.zw;
					tar_position[i].yaw = tmp_yaw;
			}

			//ѡ������װ�װ� �жϾ���
			float dis_diff_min = sqrt(tar_position[0].x * tar_position[0].x + tar_position[0].y * tar_position[0].y);
			
			for (i = 1; i<3; i++)
			{
				float temp_dis_diff = sqrt(tar_position[i].x * tar_position[0].x + tar_position[i].y * tar_position[0].y);
				if (temp_dis_diff < dis_diff_min)
				{
					dis_diff_min = temp_dis_diff;
					idx = i;
				}
			}
	} 
	else // ��������
	{

			for (i = 0; i<4; i++) {
					float tmp_yaw = st.tar_yaw + i * PI/2.0; //90
					if(tmp_yaw>PI) tmp_yaw -= 2*PI;
					float r = use_1 ? st.r1 : st.r2;
					tar_position[i].x = st.xw - r*cos(tmp_yaw);
					tar_position[i].y = st.yw - r*sin(tmp_yaw);
					tar_position[i].z = use_1 ? st.zw : st.zw + st.dz;
					tar_position[i].yaw = tmp_yaw;
					use_1 = !use_1;
			}
			
			// ѡ������װ�װ� �жϾ���
			float dis_diff_min = sqrt(tar_position[0].x * tar_position[0].x + tar_position[0].y * tar_position[0].y);
			
			for (i = 1; i<4; i++)
			{
				float temp_dis_diff = sqrt(tar_position[i].x * tar_position[0].x + tar_position[i].y * tar_position[0].y);
				if (temp_dis_diff < dis_diff_min)
				{
					dis_diff_min = temp_dis_diff;
					idx = i;
				}
				
			}
			//����ǹ�ܵ�Ŀ��װ�װ�yaw��С���Ǹ�װ�װ�
//			float yaw_diff_min = fabsf(yaw_tra - tar_position[0].yaw);
//			for (i = 1; i<4; i++) {
//					float temp_yaw_diff = fabsf(yaw_tra - tar_position[i].yaw);
//					if (temp_yaw_diff < yaw_diff_min)
//					{
//							yaw_diff_min = temp_yaw_diff;
//							idx = i;
//					}
//					idx =0;
//			}
	}
	
	// �����Զ�ʧ�߼�
//	if(AutoAim_Data_Receive.v_yaw < -0.1) // ˳ʱ��ת
//	{
//		if(AutoAim_Data_Receive.v_yaw < -3)
//		{
//			if(tar_position[idx].yaw < -0.3) idx++;
////			if(tar_position[idx].yaw < -0.3) idx=1;
////			else idx = 0;		
//		}
//		else
//		{
//			if(tar_position[idx].yaw < -0.6) idx++;
////			if(tar_position[idx].yaw < -0.7) idx=1;
////			else idx = 0;	
//		}
//	}
//	
//	else if (AutoAim_Data_Receive.v_yaw >=0.1) // ��ʱ��ת
//	{
//		float boundary=0;
//		
//		if(AutoAim_Data_Receive.v_yaw >3 ) boundary=0.3;
//		else boundary=0.6;
//		
//		if(AutoAim_Data_Receive.armor_num == 3)
//		{
//			if(tar_position[idx].yaw > boundary) idx = 2;
//			else idx = 0;
////			if(tar_position[idx].yaw > boundary)
////			{
////				for(int i=1;i<=2;i++)
////				{
////					idx++;
////					if(idx>2) idx=0;
////				}
////			}				
//			
//		}
//		else if(AutoAim_Data_Receive.armor_num == 4)
//		{
//			if(tar_position[idx].yaw > boundary) idx = 3;
//			else idx = 0;
//		}
//		else if (AutoAim_Data_Receive.armor_num == 2)
//		{
//			if(tar_position[idx].yaw > boundary) idx = 1;
//			else idx = 0;
//		}
//	}

	
	aim_z = tar_position[idx].z + st.vzw * timeDelay;
	aim_x = tar_position[idx].x + st.vxw * timeDelay;
	aim_y = tar_position[idx].y + st.vyw * timeDelay;
	//���Ÿ���ʵ��ֵ�޸�
	float temp_pitch = -pitchTrajectoryCompensation(sqrt((aim_x) * (aim_x) + (aim_y) * (aim_y)) - st.s_bias,
					aim_z + st.z_bias, st.current_v);
	if(temp_pitch)
			pitch_tra = temp_pitch;
	if(aim_x || aim_y)
			yaw_tra = (float)(atan2(aim_y, aim_x));
}

void autoSolveTrajectory_highspeed(void)
{
	
	aim_z = st.zw ;
	aim_x = st.xw ;
	aim_y = st.yw ;
	//���Ÿ���ʵ��ֵ�޸�
	float temp_pitch = -pitchTrajectoryCompensation(sqrt((aim_x) * (aim_x) + (aim_y) * (aim_y)) - st.s_bias,
					aim_z + st.z_bias, st.current_v);
	if(temp_pitch)
			pitch_tra = temp_pitch;
	if(aim_x || aim_y)
			yaw_tra = (float)(atan2(aim_y, aim_x));
}

