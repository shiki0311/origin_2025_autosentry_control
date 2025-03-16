#ifndef __IMU_H
#define __IMU_H
#include "mytype.h"

extern u8 G_reset;
enum
{
	X = 0,
	Y = 1,
	Z = 2,
	VEC_XYZ,
};

typedef struct
{
	float w;//q0;
	float x;//q1;
	float y;//q2;
	float z;//q3;

	float x_vec[VEC_XYZ];
	float y_vec[VEC_XYZ];
	float z_vec[VEC_XYZ];
	float hx_vec[VEC_XYZ];

	float a_acc[VEC_XYZ];
	float w_acc[VEC_XYZ];
	float h_acc[VEC_XYZ];
	
	float w_mag[VEC_XYZ];
	
	float gacc_deadzone[VEC_XYZ];
	
	float obs_acc_w[VEC_XYZ];
	float obs_acc_a[VEC_XYZ];
	float gra_acc[VEC_XYZ];
	
	float est_acc_a[VEC_XYZ];
	float est_acc_h[VEC_XYZ];
	float est_acc_w[VEC_XYZ];
	
	float est_speed_h[VEC_XYZ];
	float est_speed_w[VEC_XYZ];

	
	float rol;
	float pit;
	float yaw;
} _imu_st ;
extern _imu_st imu_data;

void IMU_update(float dT,float gyr[VEC_XYZ], s32 acc[VEC_XYZ],_imu_st *imu);
void calculate_RPY(void);



#endif

