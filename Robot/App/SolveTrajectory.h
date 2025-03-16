#ifndef __SOLVETRAJECTORY_H__
#define __SOLVETRAJECTORY_H__

#ifndef PI
#define PI 3.1415926535f
#endif
#define GRAVITY 9.78

enum ARMOR_ID
{
    ARMOR_OUTPOST = 0,
    ARMOR_HERO = 1,
    ARMOR_ENGINEER = 2,
    ARMOR_INFANTRY3 = 3,
    ARMOR_INFANTRY4 = 4,
    ARMOR_INFANTRY5 = 5,
    ARMOR_GUARD = 6,
    ARMOR_BASE = 7
};

enum ARMOR_NUM
{
    ARMOR_NUM_BALANCE = 2,
    ARMOR_NUM_OUTPOST = 3,
    ARMOR_NUM_NORMAL = 4
};

enum BULLET_TYPE
{
    BULLET_17 = 0,
    BULLET_42 = 1
};

struct SolveTrajectoryParams
{
    float k;             //����ϵ����Ӱ�쵯��ģ�ͣ�������׹ʱ��

    //�������
    enum BULLET_TYPE bullet_type;  //������������� 0-���� 1-Ӣ��
    float current_v;      //��ǰ����
    float current_pitch;  //��ǰpitch
    float current_yaw;    //��ǰyaw

    //Ŀ�����
    float xw;             //ROS����ϵ�µ�x
    float yw;             //ROS����ϵ�µ�y
    float zw;             //ROS����ϵ�µ�z
    float vxw;            //ROS����ϵ�µ�vx
    float vyw;            //ROS����ϵ�µ�vy
    float vzw;            //ROS����ϵ�µ�vz
    float tar_yaw;        //Ŀ��yaw
    float v_yaw;          //Ŀ��yaw�ٶ�
    float r1;             //Ŀ�����ĵ�ǰ��װ�װ�ľ���
    float r2;             //Ŀ�����ĵ�����װ�װ�ľ���
    float dz;             //��һ��װ�װ�����ڱ�����װ�װ�ĸ߶Ȳ�
    int bias_time;        //ƫ��ʱ��
    float s_bias;         //ǹ��ǰ�Ƶľ���
    float z_bias;         //yaw������ǹ��ˮƽ��Ĵ�ֱ����
    enum ARMOR_ID armor_id;     //װ�װ�����  0-outpost 6-guard 7-base
                                //1-Ӣ�� 2-���� 3-4-5-���� 
    enum ARMOR_NUM armor_num;   //װ�װ�����  2-balance 3-outpost 4-normal
};

//Ŀ��װ�װ���Ϣ
struct tar_pos
{
    float x;           //װ�װ�����������ϵ�µ�x
    float y;           //װ�װ�����������ϵ�µ�y
    float z;           //װ�װ�����������ϵ�µ�z
    float yaw;         //װ�װ�����ϵ�������������ϵ��yaw��
};

extern struct SolveTrajectoryParams st;
extern struct tar_pos tar_position[4]; //���ֻ���Ŀ�װ�װ�

extern float aim_x, aim_y , aim_z ; // aim point ��㣬������λ�����п��ӻ�
extern float pitch_tra ,yaw_tra; //pitch,yaw����
extern int idx ;
extern float distance_xy,distance_xyz;

extern float monoDirectionalAirResistanceModel(float s, float v, float angle);
extern float pitchTrajectoryCompensation(float s, float y, float v);
extern void autoSolveTrajectory(void);
void autoSolveTrajectory_highspeed(void);

#endif /*__SOLVETRAJECTORY_H__*/
