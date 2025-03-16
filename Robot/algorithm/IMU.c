#include "IMU.h"
#include "mytype.h"
#include "math.h"
static float q0q1,q0q2,q1q1,q1q3,q2q2,q2q3,q3q3,q1q2,q0q3;//q0q0,
float att_matrix[3][3]; //必须由姿态解算算出该矩阵
static float vec_err[VEC_XYZ];
static float vec_err_i[VEC_XYZ];//误差积分
static u16 reset_cnt;
static float imu_reset_val;	
u8 G_reset=1;
_imu_st imu_data =  {1,0,0,0,
					{0,0,0},
					{0,0,0},
					{0,0,0},
					{0,0,0},
					{0,0,0},
					{0,0,0},
					 0,0,0};

#define USE_LENGTH_LIM

/*参考坐标，定义为ANO坐标*

俯视，机头方向为x正方向
     +x
     |
 +y--|--
     |
		 
*/	

void IMU_update(float dT,float gyr[VEC_XYZ], s32 acc[VEC_XYZ],_imu_st *imu)
{
//	const float kp = 0.2f,ki = 0.001f;
//	const float kmp = 0.1f;
	
	static float kp_use = 0.2f,ki_use = 0.05f;
	float acc_norm_l,acc_norm_l_recip,q_norm_l;//加速度向量模
	float acc_norm[VEC_XYZ];//单位化后的加速度
	float d_angle[VEC_XYZ];//滤波后的角速度
	
//		q0q0 = imu->w * imu->w;							
		q0q1 = imu->w * imu->x;
		q0q2 = imu->w * imu->y;
		q1q1 = imu->x * imu->x;
		q1q3 = imu->x * imu->z;
		q2q2 = imu->y * imu->y;
		q2q3 = imu->y * imu->z;
		q3q3 = imu->z * imu->z;
		q1q2 = imu->x * imu->y;
		q0q3 = imu->w * imu->z;//计算四元数乘积，便于后续调用
	
		for(u8 i = 0;i<3;i++)
		{			
			imu->gra_acc[i] = acc[i];//加速度数据保存在结构体
		}
 
		acc_norm_l_recip = my_sqrt_reciprocal(my_pow(imu->gra_acc[X]) + my_pow(imu->gra_acc[Y]) + my_pow(imu->gra_acc[Z]));//计算加速度向量模的倒数
		acc_norm_l = safe_div(1,acc_norm_l_recip,0);
		
		// 加速度计的读数单位化。
		for(u8 i = 0;i<3;i++)
		{
			acc_norm[i] = imu->gra_acc[i] *acc_norm_l_recip;//三轴分量除以模长
		}
		
	//四元数表示的旋转矩阵第一列
	// 载体坐标下的x方向向量，单位化。
    att_matrix[0][0] = imu->x_vec[X] = 1 - (2*q2q2 + 2*q3q3);
    att_matrix[0][1] = imu->x_vec[Y] = 2*q1q2 - 2*q0q3;
    att_matrix[0][2] = imu->x_vec[Z] = 2*q1q3 + 2*q0q2;
	//四元数表示的旋转矩阵第二列		
	// 载体坐标下的y方向向量，单位化。
    att_matrix[1][0] = imu->y_vec[X] = 2*q1q2 + 2*q0q3;
    att_matrix[1][1] = imu->y_vec[Y] = 1 - (2*q1q1 + 2*q3q3);
    att_matrix[1][2] = imu->y_vec[Z] = 2*q2q3 - 2*q0q1;
	//四元数表示的旋转矩阵第三列
	// 载体坐标下的z方向向量（等效重力向量、重力加速度向量），单位化。
    att_matrix[2][0] = imu->z_vec[X] = 2*q1q3 - 2*q0q2;
    att_matrix[2][1] = imu->z_vec[Y] = 2*q2q3 + 2*q0q1;
    att_matrix[2][2] = imu->z_vec[Z] = 1 - (2*q1q1 + 2*q2q2);
		  				
    // 测量值与等效重力向量的叉积(计算向量误差),旋转矩阵第三列
    vec_err[X] =  (acc_norm[Y] * imu->z_vec[Z] - imu->z_vec[Y] * acc_norm[Z]);
    vec_err[Y] = -(acc_norm[X] * imu->z_vec[Z] - imu->z_vec[X] * acc_norm[Z]);
    vec_err[Z] = -(acc_norm[Y] * imu->z_vec[X] - imu->z_vec[Y] * acc_norm[X]);

	
		for(u8 i = 0;i<3;i++)
		{		
			if(acc_norm_l>1060 || acc_norm_l<900)
			{
				vec_err[X] = vec_err[Y] = vec_err[Z] = 0;
			}
		//误差积分
			vec_err_i[i] +=  LIMIT(vec_err[i],-0.1f,0.1f) *dT *ki_use;

		
	// 构造增量旋转（含融合纠正）。	
	//    d_angle[X] = (gyr[X] + (vec_err[X]  + vec_err_i[X]) * kp_use - mag_yaw_err *imu->z_vec[X] *kmp_use *RAD_PER_DEG) * dT / 2 ;
	//    d_angle[Y] = (gyr[Y] + (vec_err[Y]  + vec_err_i[Y]) * kp_use - mag_yaw_err *imu->z_vec[Y] *kmp_use *RAD_PER_DEG) * dT / 2 ;
	//    d_angle[Z] = (gyr[Z] + (vec_err[Z]  + vec_err_i[Z]) * kp_use - mag_yaw_err *imu->z_vec[Z] *kmp_use *RAD_PER_DEG) * dT / 2 ;
					
			d_angle[i] = (gyr[i] + (vec_err[i]  + vec_err_i[i]) * kp_use ) * dT / 2 ;

		}
    // 计算姿态。
		//龙哥库塔更新四元数
    imu->w = imu->w            - imu->x*d_angle[X] - imu->y*d_angle[Y] - imu->z*d_angle[Z];
    imu->x = imu->w*d_angle[X] + imu->x            + imu->y*d_angle[Z] - imu->z*d_angle[Y];
    imu->y = imu->w*d_angle[Y] - imu->x*d_angle[Z] + imu->y            + imu->z*d_angle[X];
    imu->z = imu->w*d_angle[Z] + imu->x*d_angle[Y] - imu->y*d_angle[X] + imu->z;
		
		q_norm_l = my_sqrt_reciprocal(imu->w*imu->w + imu->x*imu->x + imu->y*imu->y + imu->z*imu->z);
    imu->w *= q_norm_l;
    imu->x *= q_norm_l;
    imu->y *= q_norm_l;
    imu->z *= q_norm_l;//单位化		
		
		if(G_reset == 0)//正常修正
			{			
				kp_use = 0.2f;
				ki_use = 0.01f;
			}
			else//快速修正，通过增量进行对准
			{
				kp_use = 10.0f;
				ki_use = 0.0f;

				imu_reset_val = (ABS(vec_err[X]) + ABS(vec_err[Y]));
				
				imu_reset_val = LIMIT(imu_reset_val,0,1.0f);
				
				if((imu_reset_val < 0.05f))
				{
					//计时
					reset_cnt += 2;
					if(reset_cnt>400)
					{
						reset_cnt = 0;
						G_reset = 0;//已经对准，清除复位标记
					}
				}
				else
				{
					reset_cnt = 0;
				}
			}
}
	
static float t_temp;
void calculate_RPY(void)
{
	///////////////////////输出姿态角///////////////////////////////
	
		t_temp = LIMIT(1 - my_pow(att_matrix[2][0]),0,1);
		
		//imu_data.pit = asin(2*q1q3 - 2*q0q2)*57.30f;
	
		if(ABS(imu_data.z_vec[Z])>0.05f)//避免奇点的运算
		{
			imu_data.pit =  fast_atan2(att_matrix[2][0],my_sqrt(t_temp))*57.30f;
			imu_data.rol =  fast_atan2(att_matrix[2][1], att_matrix[2][2])*57.30f; 
//			imu_data.yaw = -fast_atan2(att_matrix[1][0], att_matrix[0][0])*57.30f; 
		}
}

