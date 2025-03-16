#include <math.h>
#include <stdio.h>
#include "Nmanifold_usbd_task.h"
#include "SolveTrajectory.h"

struct SolveTrajectoryParams st;
struct tar_pos tar_position[4]; //最多只有四块装甲板
float t = 0.5f; // 飞行时间 后续会迭代计算

float aim_x = 0, aim_y = 0, aim_z = 0; // aim point 落点，传回上位机进行可视化
float pitch_tra = 0 , yaw_tra = 0; //pitch,yaw弧度
int idx = 0;
float distance_xy=0,distance_xyz=0;

void SolvaTrajectoryInit(void)
{
		st.k = 0.092;
    st.bullet_type =  BULLET_17;
    st.current_v = 28;
    st.current_pitch = 0;
    st.current_yaw = 0;

    // 对方车辆中心在自身坐标系下的xyz距离 以下十个为接收的NUC数据
//    st.xw = AutoAim_Data_Receive.xw; 
//    st.yw = AutoAim_Data_Receive.yw;
//    st.zw = AutoAim_Data_Receive.zw;
//    st.vxw = AutoAim_Data_Receive.vxw;
//    st.vyw = AutoAim_Data_Receive.vyw;
//    st.vzw = AutoAim_Data_Receive.vzw;
//    st.v_yaw = AutoAim_Data_Receive.v_yaw;      //车体yaw转速
//    st.tar_yaw = AutoAim_Data_Receive.tar_yaw;   //装甲板yaw位置
//    st.r1 = AutoAim_Data_Receive.r1;            //目标中心到前后装甲板的距离
//    st.r2 = AutoAim_Data_Receive.r2;            //目标中心到左右装甲板的距离
		
		distance_xy = sqrt((st.xw*st.xw)+(st.yw*st.yw)); // 水平距离
		distance_xyz = sqrt((st.xw*st.xw)+(st.yw*st.yw)+(st.zw*st.zw)); // 直线距离
	
    st.dz = 0.1;            //另一对装甲板相对于被跟踪装甲板的高度差
    st.bias_time = 130;     //偏置时间
    st.s_bias = 0.10133;    //枪口前推的距离
    st.z_bias = 0.25265;    //yaw轴电机到枪口水平面的垂直距离
    st.armor_id = ARMOR_INFANTRY3;
		
//		if(AutoAim_Data_Receive.armor_num == 3) st.armor_num = ARMOR_NUM_OUTPOST;
//		else if(AutoAim_Data_Receive.armor_num == 4) st.armor_num = ARMOR_NUM_NORMAL;
//		else if(AutoAim_Data_Receive.armor_num == 2) st.armor_num = ARMOR_NUM_BALANCE;
		
}

/*
@brief 单方向空气阻力弹道模型
@param s:m 距离
@param v:m/s 速度
@param angle:rad 角度
@return z:m
*/
float monoDirectionalAirResistanceModel(float s, float v, float angle)
{
    float z;
    t = (float)((exp(st.k * s) - 1) / (st.k * v * cos(angle))); // 计算大概的飞行时间
    if(t < 0)
    {
        //严重超出最大射程，浮点数溢出，t变为负数，重置t
        t = 0;
        return 0;
    }
    //z为给定v与angle时的高度
    z = (float)(v * sin(angle) * t - GRAVITY * t * t / 2);
    //printf("model %f %f\n", t, z);
    return z;
}

/*
@brief pitch轴解算
@param s:m 距离
@param z:m 高度
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
    for (i = 0; i < 20; i++) // 二十次迭代得到
    {
        angle_pitch = atan2(z_temp, s); // rad
        z_actual = monoDirectionalAirResistanceModel(s, v, angle_pitch); // 下降的高度
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

    // 线性预测
  float timeDelay = st.bias_time/1000.0 + t; 
  st.tar_yaw += st.v_yaw * timeDelay; // 

    //计算四块装甲板的位置
    //装甲板id顺序,以四块装甲板为例,逆时针编号
    //      2
    //   3     1
    //      0
	int use_1 = 1;
	int i = 0;
	int idx = 0; // 选择的装甲板
	
	//armor_num = ARMOR_NUM_BALANCE 平衡步兵
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

			//只需判断两块装甲板即可
			float temp_yaw_diff = fabsf(yaw_tra - tar_position[1].yaw);
			if (temp_yaw_diff < yaw_diff_min)
			{
					yaw_diff_min = temp_yaw_diff;
					idx = 1;
			}
	} 
	else if (st.armor_num == ARMOR_NUM_OUTPOST)  //前哨站
	{  
			for (i = 0; i<3; i++) {
					float tmp_yaw = st.tar_yaw + i * 2.0 * PI/3.0;  // 2/3PI
					float r =  (st.r1 + st.r2)/2;   //???r1=r2 ???????
					tar_position[i].x = st.xw - r*cos(tmp_yaw);
					tar_position[i].y = st.yw - r*sin(tmp_yaw);
					tar_position[i].z = st.zw;
					tar_position[i].yaw = tmp_yaw;
			}

			//选择最优装甲板 判断距离
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
	else // 正常兵种
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
			
			// 选择最优装甲板 判断距离
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
			//计算枪管到目标装甲板yaw最小的那个装甲板
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
	
	// 防沉迷丢失逻辑
//	if(AutoAim_Data_Receive.v_yaw < -0.1) // 顺时针转
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
//	else if (AutoAim_Data_Receive.v_yaw >=0.1) // 逆时针转
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
	//符号根据实际值修改
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
	//符号根据实际值修改
	float temp_pitch = -pitchTrajectoryCompensation(sqrt((aim_x) * (aim_x) + (aim_y) * (aim_y)) - st.s_bias,
					aim_z + st.z_bias, st.current_v);
	if(temp_pitch)
			pitch_tra = temp_pitch;
	if(aim_x || aim_y)
			yaw_tra = (float)(atan2(aim_y, aim_x));
}

