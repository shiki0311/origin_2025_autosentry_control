/****************************************************************
 * @file: 	Gimbal_Task.c
 * @author: Shiki
 * @date:	2025.6.18
 * @brief:	哨兵云台任务
 * @attention:
 ******************************************************************/
#include "Gimbal_Task.h"
#include "INS_Task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "remote_control.h"
#include "bsp_can.h"
#include "main.h"
#include "arm_math.h"
#include "Cboard_To_Nuc_usbd_communication.h"
#include "referee.h"
#include "Vofa_send.h"
#include "detect_task.h"
#include "user_common_lib.h"
#include "Shoot_Task.h"
#include "tim.h"

#define ANGLE_TO_RAD 0.01745f
#define RAD_TO_ANGLE 57.295779f

#define PITCH_ECD_ANGLE_MAX 27280 // 27280
#define PITCH_ECD_ANGLE_MIN 24600 // 24600

/****************************************重力补偿参数和速度环前馈系数*******************************************************/
#define YAW_MOTOR_FF 4.0f
#define PITCH_MOTOR_FF 1.8f
#define PITCH_MOTOR_GRAVITY_STATIC_COMPENSATE (1.3f)  // 用于补偿重力，pitch轴与地面平行时抵消重力所需的力矩
#define PITCH_MOTOR_GRAVITY_DYNAMIC_COMPENSATE (1.5f) // 用于补偿重力，pitch轴与地面不平行时抵消重力所需的偏置力矩系数
/**************************************************************************************************************************/
#define YAW_MOTOR_SPEED_PID_KP 800.0f
#define YAW_MOTOR_SPEED_PID_KI 20.0f // 80.0f
#define YAW_MOTOR_SPEED_PID_KD 200.0f
#define YAW_MOTOR_SPEED_PID_MAX_OUT 30000.0f
#define YAW_MOTOR_SPEED_PID_MAX_IOUT 10000.0f

// #define YAW_MOTOR_ANGLE_PID_KP 10.5f
// #define YAW_MOTOR_ANGLE_PID_KI 0.0f
// #define YAW_MOTOR_ANGLE_PID_KD 350.0f
// #define YAW_MOTOR_ANGLE_PID_MAX_OUT 1200.0f
// #define YAW_MOTOR_ANGLE_PID_MAX_IOUT 50.0f

#define YAW_MOTOR_ANGLE_PID_KP 7.5f
#define YAW_MOTOR_ANGLE_PID_KI 0.0f
#define YAW_MOTOR_ANGLE_PID_KD 100.0f
#define YAW_MOTOR_ANGLE_PID_MAX_OUT 1200.0f
#define YAW_MOTOR_ANGLE_PID_MAX_IOUT 50.0f

#define YAW_MOTOR_AUTO_AIM_PID_KP 25.0f
#define YAW_MOTOR_AUTO_AIM_PID_KI 0.0f
#define YAW_MOTOR_AUTO_AIM_PID_KD 50.0f
#define YAW_MOTOR_AUTO_AIM_PID_MAX_OUT 1200.0f
#define YAW_MOTOR_AUTO_AIM_PID_MAX_IOUT 0.0f

#define PITCH_MOTOR_SPEED_PID_KP 5.0f
#define PITCH_MOTOR_SPEED_PID_KI 0.0f
#define PITCH_MOTOR_SPEED_PID_KD 3.0f
#define PITCH_MOTOR_SPEED_PID_MAX_OUT 10.0f
#define PITCH_MOTOR_SPEED_PID_MAX_IOUT 1.0f

#define PITCH_MOTOR_ANGLE_PID_KP 0.4f // 0.2f
#define PITCH_MOTOR_ANGLE_PID_KI 0.0f
#define PITCH_MOTOR_ANGLE_PID_KD 10.0f // 3.0f
#define PITCH_MOTOR_ANGLE_PID_MAX_OUT 4.5f
#define PITCH_MOTOR_ANGLE_PID_MAX_IOUT 1.0f

// #define PITCH_MOTOR_AUTO_AIM_PID_KP 0.7f
// #define PITCH_MOTOR_AUTO_AIM_PID_KI 0.00000f // 0.0005f
// #define PITCH_MOTOR_AUTO_AIM_PID_KD 10.0f
// #define PITCH_MOTOR_AUTO_AIM_PID_MAX_OUT 20.0f
// #define PITCH_MOTOR_AUTO_AIM_PID_MAX_IOUT 0.0f

#define PITCH_MOTOR_AUTO_AIM_PID_KP 0.4f
#define PITCH_MOTOR_AUTO_AIM_PID_KI 0.00000f // 0.0005f
#define PITCH_MOTOR_AUTO_AIM_PID_KD 4.0f
#define PITCH_MOTOR_AUTO_AIM_PID_MAX_OUT 20.0f
#define PITCH_MOTOR_AUTO_AIM_PID_MAX_IOUT 0.0f

/****************************云台模式处理函数声明，原因是使用函数名给函数指针赋值之前，该函数必须已经被声明***************************************/
void gimbal_autoaim_handler(void);
void gimbal_remote_control_handler(void);
void gimbal_nav_handler(void);
void gimbal_safe_handler(void);
/*********************************************************全局变量及常量区***************************************************************/
gimbal_command_t gimbal_commands[] = {{AUTOAIM, gimbal_autoaim_handler}, {GIMBAL_REMOTE_CONTROL, gimbal_remote_control_handler}, {NAV, gimbal_nav_handler}, {GIMBAL_SAFE, gimbal_safe_handler}}; // 初始化云台控制命令数组
gimbal_motor_t gimbal_m6020[2] = {0};
gimbal_mode_t gimbal_mode = GIMBAL_SAFE; // 设置成全局变量便于调试观察
float yaw_angle_err = 0;                 // 仅用于调试时候观测yaw角度偏差，不参与云台控制
float pitch_angle_err = 0;               // 仅用于调试时候观测pitch角度偏差，不参与云台控制

/**
 * @description: 通过检查INS_angle_set确保yaw_angle_error在一百八十度以内,不会出现转动角度超出180度的情况，保证每次转到目标位置都是最短路径
 * @return {*}
 * @param {float} 目标角度
 * @param {float} 当前角度
 */
static float check_INS_angleset_to_keep_err_in_180(float target, float current) // 只有yaw轴需要，pitch活动范围不会超过180度
{
    float err = target - current;
    if (err > 180)
        target -= 360;
    else if (err < -180)
        target += 360;
    return target;
}

void Gimbal_Motor_Pid_Init(void)
{
    const static fp32 yaw_motor_speed_pid[3] = {YAW_MOTOR_SPEED_PID_KP, YAW_MOTOR_SPEED_PID_KI, YAW_MOTOR_SPEED_PID_KD};
    const static fp32 yaw_motor_angle_pid[3] = {YAW_MOTOR_ANGLE_PID_KP, YAW_MOTOR_ANGLE_PID_KI, YAW_MOTOR_ANGLE_PID_KD};
    const static fp32 yaw_motor_auto_aim_pid[3] = {YAW_MOTOR_AUTO_AIM_PID_KP, YAW_MOTOR_AUTO_AIM_PID_KI, YAW_MOTOR_AUTO_AIM_PID_KD};

    const static fp32 pitch_motor_speed_pid[3] = {PITCH_MOTOR_SPEED_PID_KP, PITCH_MOTOR_SPEED_PID_KI, PITCH_MOTOR_SPEED_PID_KD};
    const static fp32 pitch_motor_angle_pid[3] = {PITCH_MOTOR_ANGLE_PID_KP, PITCH_MOTOR_ANGLE_PID_KI, PITCH_MOTOR_ANGLE_PID_KD};
    const static fp32 pitch_motor_auto_aim_pid[3] = {PITCH_MOTOR_AUTO_AIM_PID_KP, PITCH_MOTOR_AUTO_AIM_PID_KI, PITCH_MOTOR_AUTO_AIM_PID_KD};

    PID_init(&gimbal_m6020[0].speed_pid, PID_POSITION, yaw_motor_speed_pid, YAW_MOTOR_SPEED_PID_MAX_OUT, YAW_MOTOR_SPEED_PID_MAX_IOUT);
    PID_init(&gimbal_m6020[0].angle_pid, PID_POSITION, yaw_motor_angle_pid, YAW_MOTOR_ANGLE_PID_MAX_OUT, YAW_MOTOR_ANGLE_PID_MAX_IOUT);
    PID_init(&gimbal_m6020[0].auto_aim_pid, PID_POSITION, yaw_motor_auto_aim_pid, YAW_MOTOR_AUTO_AIM_PID_MAX_OUT, YAW_MOTOR_AUTO_AIM_PID_MAX_IOUT);

    PID_init(&DM_pitch_motor_data.speed_pid, PID_POSITION, pitch_motor_speed_pid, PITCH_MOTOR_SPEED_PID_MAX_OUT, PITCH_MOTOR_SPEED_PID_MAX_IOUT);
    PID_init(&DM_pitch_motor_data.angle_pid, PID_POSITION, pitch_motor_angle_pid, PITCH_MOTOR_ANGLE_PID_MAX_OUT, PITCH_MOTOR_ANGLE_PID_MAX_IOUT);
    PID_init(&DM_pitch_motor_data.auto_aim_pid, PID_POSITION, pitch_motor_auto_aim_pid, PITCH_MOTOR_AUTO_AIM_PID_MAX_OUT, PITCH_MOTOR_AUTO_AIM_PID_MAX_IOUT);
}

void Gimbal_Motor_Data_Update(void)
{
    gimbal_m6020[0].INS_speed_last = gimbal_m6020[0].INS_speed;
    gimbal_m6020[0].INS_speed_set_last = gimbal_m6020[0].INS_speed_set;
    gimbal_m6020[0].INS_speed = bmi088_real_data.gyro[2] * RAD_TO_ANGLE;
    gimbal_m6020[0].INS_angle = INS_angle_deg[0];
    gimbal_m6020[0].ENC_angle = motor_measure_gimbal[0].ecd;
    gimbal_m6020[0].ENC_speed = motor_measure_gimbal[0].speed_rpm;

    DM_pitch_motor_data.INS_speed_last = DM_pitch_motor_data.INS_speed;
    DM_pitch_motor_data.INS_speed = bmi088_real_data.gyro[1];
    DM_pitch_motor_data.INS_angle = INS_angle_deg[2];

    yaw_angle_err = gimbal_m6020[0].INS_angle_set - gimbal_m6020[0].INS_angle;
    pitch_angle_err = DM_pitch_motor_data.INS_angle_set - DM_pitch_motor_data.INS_angle;
}

static gimbal_mode_t Gimbal_Mode_Update()
{
    bool_t check_autoaim = (AutoAim_Data_Receive.yaw_aim != 0 || AutoAim_Data_Receive.pitch_aim != 0); // 是否满足自瞄模式，下面以此类推
    bool_t check_rc_ctrl = (rc_ctrl.rc.s[1] == RC_SW_MID);
    bool_t check_nav = (rc_ctrl.rc.s[1] == RC_SW_UP);
    bool_t check_safe = ((rc_ctrl.rc.s[1] == RC_SW_DOWN) || toe_is_error(DBUS_TOE));

    if (check_safe)
    {
        return GIMBAL_SAFE; // 失能模式的优先级最高，需要优先判断
    }
    else if (check_autoaim)
    {
        return AUTOAIM; // 自瞄的优先级第二高
    }
    else if (check_nav)
    {
        return NAV;
    }
    else if (check_rc_ctrl)
    {
        return GIMBAL_REMOTE_CONTROL; // 遥控的优先级最低
    }
    else
        return GIMBAL_SAFE;
}

/**
 * @description: 检查自瞄目标是否丢失，若丢失则yaw电机原地停两秒防止敌人再次出现，复活赛弃用，联盟赛可开启
 * @return none
 */
void Check_Yaw_LostTarget_Wait()
{
    static uint32_t zero_speed_start_time = 0;
    static uint8_t zero_speed_flag = 0;
    static float auto_aim_yaw_last = 0;
    if (AutoAim_Data_Receive.yaw_aim == 0 && auto_aim_yaw_last != 0)
    {
        zero_speed_start_time = xTaskGetTickCount();
        zero_speed_flag = 1;
    }
    if (zero_speed_flag && (xTaskGetTickCount() - zero_speed_start_time <= pdMS_TO_TICKS(2000)))
    {
        gimbal_m6020[0].INS_speed_set = 0;
    }
    else
    {
        zero_speed_flag = 0;
    }
    auto_aim_yaw_last = AutoAim_Data_Receive.yaw_aim;
}

/**
 * @description: pitch轴重力补偿，解算出的目标电流值叠加在最后speed pid输出的目标电流上
 * @return pitch轴电机重力补偿的电流值
 */
float Pitch_Gravity_Compensation(void)
{
    return PITCH_MOTOR_GRAVITY_DYNAMIC_COMPENSATE * arm_sin_f32(DM_pitch_motor_data.INS_angle / 57.3) + PITCH_MOTOR_GRAVITY_STATIC_COMPENSATE;
}

/**
 * @description: 检查pitch当前有无越过电子限位，不用陀螺仪角度检查是因为陀螺仪会零漂，用电机自身编码器检查更准确
 * @return 无
 */
void Check_Pitch_Electronic_Limit(gimbal_motor_control_mode_t mode)
{
    if ((DM_pitch_motor_data.p_int < PITCH_ECD_ANGLE_MIN && DM_pitch_motor_data.INS_speed_set > 0) || (DM_pitch_motor_data.p_int > PITCH_ECD_ANGLE_MAX && DM_pitch_motor_data.INS_speed_set < 0))
    {
        if (mode == POSITION)
            DM_pitch_motor_data.INS_angle_set = DM_pitch_motor_data.INS_angle; // 位置模式下一越过电子限位，pitch轴目标角度就设置为当前角度
        else if (mode == SPEED)
            DM_pitch_motor_data.INS_speed_set = 0; // 速度模式下一越过电子限位，pitch轴目标速度就设置为0
    }
}
/**
 * @description: 用于计算导航模式下pitch轴上下摆动巡航的目标角度
 * @return pitch目标角度
 */
float Pitch_Updown(void)
{
    static float auto_pitch_watch = 0;
    static uint8_t updown_switch_flag = 0;
    uint8_t speed_state = AutoAim_Data_Receive.pitch_speed ? HIT_OUTPOST : HIT_ROBOT;
    const PitchSwingParams swing_params[2] = {{-10.0f, 25.0f, 0.08f}, {-24.0f, -15.0f, 0.04f}};

    if (AutoAim_Data_Receive.yaw_rotate_flag == 0 && yaw_rotate_flag_last != 0) // 保证大回环后先pitch先往下动，防止瞄不到离自己近的车
    {
        updown_switch_flag = 0;
        auto_pitch_watch = DM_pitch_motor_data.INS_angle;
    }

    if (updown_switch_flag == 0)
    {
        auto_pitch_watch += swing_params[speed_state].step;
        if (auto_pitch_watch >= swing_params[speed_state].max_angle)
        {
            updown_switch_flag = 1;
            auto_pitch_watch = swing_params[speed_state].max_angle;
        }
    }
    else
    {
        auto_pitch_watch -= swing_params[speed_state].step;
        if (auto_pitch_watch <= swing_params[speed_state].min_angle)
        {
            updown_switch_flag = 0;
            auto_pitch_watch = swing_params[speed_state].min_angle;
        }
    }
    return auto_pitch_watch;
}

/**
 * @description: 根据当前电机的控制模式（控制位置或者速度）选择不同pid计算逻辑最终算出电机目标电流,把pid指针作为参数传入是因为位置模式下会传入不同的pid
 * @return 无
 */
void Calculate_Gimbal_Motor_Target_Current(pid_type_def *gimbal_motor_pid, gimbal_motor_control_mode_t mode, gimbal_motor_id_t motor_id)
{
    switch (motor_id)
    {
    case PITCH_MOTOR:
        if (mode == POSITION)
        {
            PID_calc(gimbal_motor_pid, DM_pitch_motor_data.INS_angle, DM_pitch_motor_data.INS_angle_set);
            DM_pitch_motor_data.INS_speed_set = gimbal_motor_pid->out + (DM_pitch_motor_data.INS_speed - DM_pitch_motor_data.INS_speed_last) * PITCH_MOTOR_FF; // 目标速度为pid输出加上前馈
            PID_calc(&DM_pitch_motor_data.speed_pid, DM_pitch_motor_data.INS_speed, DM_pitch_motor_data.INS_speed_set);
            DM_pitch_motor_data.target_current = -DM_pitch_motor_data.speed_pid.out + Pitch_Gravity_Compensation(); // 有个负号是因为DM4310的电流正方向与陀螺仪速度极性相反，电流使电机加速时陀螺仪速度为负，所以要加个负号让它们极性一致，否则pid不收敛
        }
        else if (mode == SPEED)
        {
            PID_calc(gimbal_motor_pid, DM_pitch_motor_data.INS_speed, DM_pitch_motor_data.INS_speed_set);
            DM_pitch_motor_data.target_current = -gimbal_motor_pid->out + Pitch_Gravity_Compensation();
        }
        break;
    case YAW_MOTOR:
        if (mode == POSITION)
        {
            PID_calc(gimbal_motor_pid, gimbal_m6020[0].INS_angle, gimbal_m6020[0].INS_angle_set);
            gimbal_m6020[0].INS_speed_set = gimbal_motor_pid->out;
            PID_calc(&gimbal_m6020[0].speed_pid, gimbal_m6020[0].INS_speed, gimbal_m6020[0].INS_speed_set);
            gimbal_m6020[0].give_current = gimbal_m6020[0].speed_pid.out + (gimbal_m6020[0].INS_speed_set - gimbal_m6020[0].INS_speed_set_last) * YAW_MOTOR_FF;
        }
        else if (mode == SPEED)
        {
            PID_calc(gimbal_motor_pid, gimbal_m6020[0].INS_speed, gimbal_m6020[0].INS_speed_set);
            gimbal_m6020[0].give_current = gimbal_motor_pid->out;
        }
        break;

    default:
        break;
    }
}
/**
 * @description: 失能模式下的控制函数，直接对云台电机电流置零
 * @return 无
 */
void gimbal_safe_handler(void)
{
    gimbal_m6020[0].give_current = 0;
    DM_pitch_motor_data.target_current = 0;
}

/**
 * @description: 自瞄模式下的控制函数，两个轴电机都是位置控制
 * @return 无
 */
void gimbal_autoaim_handler(void)
{
    gimbal_motor_control_mode_t yaw_mode = POSITION, pitch_mode = POSITION;

    gimbal_m6020[0].INS_angle_set = check_INS_angleset_to_keep_err_in_180(AutoAim_Data_Receive.yaw_aim, gimbal_m6020[0].INS_angle);
    DM_pitch_motor_data.INS_angle_set = -AutoAim_Data_Receive.pitch_aim; // 不知道为什么自瞄那边传过来的目标pitch角跟实际符号是反的，先加个负号凑合用
    Check_Pitch_Electronic_Limit(pitch_mode);

    if (!AutoAim_Data_Receive.yaw_rotate_flag) // 正常自瞄，不需要yaw轴大回环锁背后敌人,使用auto_aim_pid
    {
        Calculate_Gimbal_Motor_Target_Current(&gimbal_m6020[0].auto_aim_pid, yaw_mode, YAW_MOTOR);
    }
    else // 需要大回环，用普通的angle_pid防止超调
    {
        Calculate_Gimbal_Motor_Target_Current(&gimbal_m6020[0].angle_pid, yaw_mode, YAW_MOTOR);
    }

    Calculate_Gimbal_Motor_Target_Current(&DM_pitch_motor_data.auto_aim_pid, pitch_mode, PITCH_MOTOR);
}

/**
 * @description: 导航模式下的控制函数，pitch轴电机是位置控制，yaw轴电机是速度控制
 * @return 无
 */
void gimbal_nav_handler(void)
{
    gimbal_motor_control_mode_t yaw_mode = SPEED, pitch_mode = POSITION;

    gimbal_m6020[0].INS_speed_set = AutoAim_Data_Receive.yaw_speed * RAD_TO_ANGLE; // 导航模式下yaw轴正常巡逻
    DM_pitch_motor_data.INS_angle_set = Pitch_Updown();
    Check_Pitch_Electronic_Limit(pitch_mode);

    Calculate_Gimbal_Motor_Target_Current(&gimbal_m6020[0].speed_pid, yaw_mode, YAW_MOTOR);
    Calculate_Gimbal_Motor_Target_Current(&DM_pitch_motor_data.angle_pid, pitch_mode, PITCH_MOTOR);
}

/**
 * @description: 遥控模式下的控制函数，速度模式和位置模式根据不同条件切换
 * @return 无
 */
void gimbal_remote_control_handler(void)
{
    static gimbal_motor_control_mode_t yaw_mode, yaw_mode_last, pitch_mode, pitch_mode_last;
    yaw_mode_last = yaw_mode;
    pitch_mode_last = pitch_mode;
    yaw_mode = (abs(rc_ctrl.rc.ch[0]) > 10) ? SPEED : POSITION;
    pitch_mode = (abs(rc_ctrl.rc.ch[1]) > 5) ? SPEED : POSITION;

    if (yaw_mode == SPEED)
    {
        gimbal_m6020[0].INS_speed_set = -(float)rc_ctrl.rc.ch[0] / 660.0f * 5.0f * RAD_TO_ANGLE;
        Calculate_Gimbal_Motor_Target_Current(&gimbal_m6020[0].speed_pid, SPEED, YAW_MOTOR);
    }
    else if (yaw_mode == POSITION)
    {
        if (yaw_mode_last == SPEED)
        {
            gimbal_m6020[0].INS_angle_set = gimbal_m6020[0].INS_angle;
        }
        gimbal_m6020[0].INS_angle_set = check_INS_angleset_to_keep_err_in_180(gimbal_m6020[0].INS_angle_set, gimbal_m6020[0].INS_angle);
        Calculate_Gimbal_Motor_Target_Current(&gimbal_m6020[0].angle_pid, POSITION, YAW_MOTOR);
    }
    if (pitch_mode == SPEED)
    {
        DM_pitch_motor_data.INS_speed_set = (float)rc_ctrl.rc.ch[1] / 660.0f * 5.0f;
        Check_Pitch_Electronic_Limit(SPEED);
        Calculate_Gimbal_Motor_Target_Current(&DM_pitch_motor_data.speed_pid, SPEED, PITCH_MOTOR);
    }
    else if (pitch_mode == POSITION)
    {
        if (pitch_mode_last == SPEED)
        {
            DM_pitch_motor_data.INS_angle_set = DM_pitch_motor_data.INS_angle;
        }
        Check_Pitch_Electronic_Limit(POSITION);
        Calculate_Gimbal_Motor_Target_Current(&DM_pitch_motor_data.auto_aim_pid, POSITION, PITCH_MOTOR);
    }
}

/**
 * @brief  根据不同云台模式执行对应云台控制函数，每个控制函数最后会解算出当前pitch轴电机和yaw轴电机的目标电流
 *         注意:1.除了gimbal_safe_handler外每个控制函数都会先选择yaw和pitch的被控变量（位置或者速度），并设定被控变量目标值，然后调用Calculate_Gimbal_Target_Current函数通过pid计算出当前云台电机的目标电流
 *              2.gimbal_safe_handler会直接将云台的yaw轴和pitch轴的电流设置为0
 * @return 无
 */
void choose_gimbal_handler(gimbal_mode_t mode)
{
    int size = sizeof(gimbal_commands) / sizeof(gimbal_command_t);
    for (int i = 0; i < size; i++) // 寻找匹配当前模式的控制函数
    {
        if (gimbal_commands[i].mode == mode)
        {
            gimbal_commands[i].handler();
            return;
        }
    }
}
void Check_DM_Auto_Enable()
{
    static uint8_t enable_send_count = 0;
    static uint8_t gimbal_output_last = 0;
    if (Game_Robot_State.power_management_gimbal_output && !gimbal_output_last)
    {
        enable_send_count = 5;
        HAL_Delay(1000);
    }
    gimbal_output_last = Game_Robot_State.power_management_gimbal_output;

    while (enable_send_count > 0)
    {
        enable_DM(GIMBAL_PITCH_DM_SendID, 0x01);
        enable_send_count--;
    }
}

void Gimbal_Task(void const *argument)
{
    Gimbal_Motor_Pid_Init();
    vTaskDelay(200);
		
    enable_DM(GIMBAL_PITCH_DM_SendID, 0x01);
    gimbal_mode = GIMBAL_SAFE;

    HAL_TIM_Base_Start_IT(&htim5); // 开启tim5中断，用于定时发送can报文，保险起见在can_filter_init()中启动can外设后再发can报文（理论上不用这样，hal库有保护）
    HAL_TIM_Base_Start_IT(&htim3); // 启动tim3中断，用于定时检测有没有发送失败的can报文并进行重发

    while (1)
    {
        Check_DM_Auto_Enable();
        Gimbal_Motor_Data_Update();
        gimbal_mode = Gimbal_Mode_Update();
        choose_gimbal_handler(gimbal_mode);
        Allocate_Can_Buffer(gimbal_m6020[0].give_current, 0, 0, 0, CAN_GIMBAL_YAW_CMD);
        Ctrl_DM_Motor(GIMBAL_PITCH_DM_SendID, 0, 0, 0, 0, DM_pitch_motor_data.target_current);
        Allocate_Can_Buffer(0, shoot_m2006[0].target_current, shoot_motor_3508[0].target_current, shoot_motor_3508[1].target_current, CAN_SHOOT_CMD);

        // CAN_Gimbal_CMD(gimbal_m6020[0].give_current, 0, 0, 0);
        // CAN_Shoot_CMD(0, shoot_m2006[0].target_current, shoot_motor_3508[0].target_current, shoot_motor_3508[1].target_current);

            //		Vofa_Send_Data4((float)dial_stop_cnt,motor_measure_shoot[2].given_current,shoot_m2006[0].target_current,0);
            //	Vofa_Send_Data4(AutoAim_Data_Receive.yaw_aim,gimbal_m6020[0].INS_angle,(float)AutoAim_Data_Receive.fire_or_not,0);
            //		Vofa_Send_Data4((float)DM_pitch_motor_data.INS_angle_set,(float)DM_pitch_motor_data.INS_angle,DM_pitch_motor_data.INS_speed,DM_pitch_motor_data.INS_speed_set);
            vTaskDelay(1);
    }
}
