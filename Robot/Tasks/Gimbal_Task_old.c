/****************************************************************
 * @file: 	Gimbal_Task.c
 * @author: Shiki
 * @date:	2025.6.18
 * @brief:	哨兵云台任务
 * @attention:
 ******************************************************************/
#include "Gimbal_Task.h"
#include "INS_Task.h"
#include "Shoot_Task.h"
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

#define ANGLE_TO_RAD 0.01745f
#define RAD_TO_ANGLE 57.295779f

#define PITCH_ECD_ANGLE_MAX 27280 // 27800
#define PITCH_ECD_ANGLE_MIN 24600 // 25000

/****************************************重力补偿参数和自瞄前馈系数*******************************************************/
#define YAW_MOTOR_AUTO_AIM_FF 2.5f
#define PITCH_MOTOR_AUTO_AIM_FF 1.8f
#define PITCH_MOTOR_GRAVITY_STATIC_COMPENSATE (2.5f)  // 用于补偿重力，pitch轴与地面平行时抵消重力所需的力矩
#define PITCH_MOTOR_GRAVITY_DYNAMIC_COMPENSATE (1.1f) // 用于补偿重力，pitch轴与地面不平行时抵消重力所需的偏置力矩系数
/**************************************************************************************************************************/
#define YAW_MOTOR_SPEED_PID_KP 800.0f
#define YAW_MOTOR_SPEED_PID_KI 0.0f // 80.0f
#define YAW_MOTOR_SPEED_PID_KD 200.0f
#define YAW_MOTOR_SPEED_PID_MAX_OUT 30000.0f
#define YAW_MOTOR_SPEED_PID_MAX_IOUT 10000.0f

#define YAW_MOTOR_ANGLE_PID_KP 11.0f
#define YAW_MOTOR_ANGLE_PID_KI 0.0f
#define YAW_MOTOR_ANGLE_PID_KD 400.0f
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

#define PITCH_MOTOR_ANGLE_PID_KP 0.2f // 0.2f
#define PITCH_MOTOR_ANGLE_PID_KI 0.0f
#define PITCH_MOTOR_ANGLE_PID_KD 10.0f // 3.0f
#define PITCH_MOTOR_ANGLE_PID_MAX_OUT 4.5f
#define PITCH_MOTOR_ANGLE_PID_MAX_IOUT 1.0f

#define PITCH_MOTOR_AUTO_AIM_PID_KP 0.7f
#define PITCH_MOTOR_AUTO_AIM_PID_KI 0.00000f // 0.0005f
#define PITCH_MOTOR_AUTO_AIM_PID_KD 10.0f
#define PITCH_MOTOR_AUTO_AIM_PID_MAX_OUT 20.0f
#define PITCH_MOTOR_AUTO_AIM_PID_MAX_IOUT 0.0f

/************************全局变量及常量区*****************************/
// gimbal_command_t gimbal_commands[] = {{AUTOAIM, gimbal_autoaim_handler}, {GIMBAL_REMOTE_CONTROL, gimbal_remote_control_handler}, {NAV, gimbal_nav_handler}, {GIMBAL_SAFE, gimbal_safe_handler}}; // 初始化云台控制命令数组
gimbal_motor_t gimbal_m6020[2] = {0};
float yaw_angle_err = 0;
float pitch_angle_err = 0;
uint8_t yaw_rotate_flag = 0;
/*******************************************************************/
float Pitch_Updown(void);

static float angle_error_calc(float target, float current)
{
    float err = target - current;
    if (err > 180)
        err -= 360;
    else if (err < -180)
        err += 360;
    return err;
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
    gimbal_m6020[0].INS_speed = bmi088_real_data.gyro[2] * RAD_TO_ANGLE;
    gimbal_m6020[0].INS_angle = INS_angle_deg[0];
    gimbal_m6020[0].ENC_angle = motor_measure_gimbal[0].ecd;
    gimbal_m6020[0].ENC_speed = motor_measure_gimbal[0].speed_rpm;

    DM_pitch_motor_data.INS_speed_last = DM_pitch_motor_data.INS_speed;
    DM_pitch_motor_data.INS_speed = bmi088_real_data.gyro[1];
    DM_pitch_motor_data.INS_angle = INS_angle_deg[2];
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
        zero_speed_flag = 0; //
    }
    auto_aim_yaw_last = AutoAim_Data_Receive.yaw_aim;
}
void Yaw_Motor_Control(void)
{
    static uint8_t yaw_mode = 0, yaw_mode_last = 0;
    if ((AutoAim_Data_Receive.yaw_aim != 0 || AutoAim_Data_Receive.pitch_aim != 0) && !AutoAim_Data_Receive.yaw_rotate_flag)
    {
        yaw_angle_err = angle_error_calc(AutoAim_Data_Receive.yaw_aim, gimbal_m6020[0].INS_angle);
        if (abs(yaw_angle_err) < 100.0f)
        {
            PID_calc(&gimbal_m6020[0].auto_aim_pid, yaw_angle_err, 0);
            gimbal_m6020[0].INS_speed_set = (-gimbal_m6020[0].auto_aim_pid.out) + (gimbal_m6020[0].INS_speed - gimbal_m6020[0].INS_speed_last) * YAW_MOTOR_AUTO_AIM_FF; // ??0.8?????yaw?????????
            gimbal_m6020[0].INS_angle_set = AutoAim_Data_Receive.yaw_aim;
            yaw_mode = yaw_mode_last = 1;
        }
    }
    else if (rc_ctrl.rc.s[1] == RC_SW_MID)
    {
        yaw_mode_last = yaw_mode;
        if (rc_ctrl.rc.ch[0] > 10 || rc_ctrl.rc.ch[0] < -10)
        {
            yaw_mode = 0; // 0????
        }
        else
        {
            yaw_mode = 1; // 1?????
        }

        if (yaw_mode == 0)
        {
            gimbal_m6020[0].INS_speed_set = -(float)rc_ctrl.rc.ch[0] / 660.0f * 5.0f * RAD_TO_ANGLE;
        }
        else if (yaw_mode == 1 && yaw_mode_last == 0) // yaw????????????????
        {
            gimbal_m6020[0].INS_angle_set = gimbal_m6020[0].INS_angle;
        }

        if (yaw_mode == 1)
        {
            yaw_angle_err = angle_error_calc(gimbal_m6020[0].INS_angle_set, gimbal_m6020[0].INS_angle);
            PID_calc(&gimbal_m6020[0].angle_pid, yaw_angle_err, 0);
            gimbal_m6020[0].INS_speed_set = -gimbal_m6020[0].angle_pid.out;
        }
    }
    else if (rc_ctrl.rc.s[1] == RC_SW_UP) // ??????yaw?????
    {
        if (AutoAim_Data_Receive.yaw_rotate_flag == 1) // 背后相机检测到目标，需要转180度，使用另一套位置pid防止超调
        {
            yaw_angle_err = angle_error_calc(AutoAim_Data_Receive.yaw_aim, gimbal_m6020[0].INS_angle);
            PID_calc(&gimbal_m6020[0].angle_pid, yaw_angle_err, 0);
            gimbal_m6020[0].INS_speed_set = -gimbal_m6020[0].angle_pid.out;
            PID_calc(&gimbal_m6020[0].speed_pid, gimbal_m6020[0].INS_speed, gimbal_m6020[0].INS_speed_set);
            gimbal_m6020[0].give_current = gimbal_m6020[0].speed_pid.out;
            return;
        }
        else
        {
            gimbal_m6020[0].INS_speed_set = AutoAim_Data_Receive.yaw_speed * RAD_TO_ANGLE; // 导航模式下yaw轴正常巡逻
            // Check_Yaw_LostTarget_Wait();
        }
    }
    PID_calc(&gimbal_m6020[0].speed_pid, gimbal_m6020[0].INS_speed, gimbal_m6020[0].INS_speed_set);
    gimbal_m6020[0].give_current = gimbal_m6020[0].speed_pid.out;
}

fp32 Pitch_Gravity_Compensation(void)
{
    return PITCH_MOTOR_GRAVITY_DYNAMIC_COMPENSATE * arm_sin_f32(DM_pitch_motor_data.INS_angle / 57.3) + PITCH_MOTOR_GRAVITY_STATIC_COMPENSATE;
}

void Pitch_Motor_Control(void)
{
    static uint8_t pitch_mode = 0, pitch_mode_last = 0; // 0:speed,1:angle

    // ??????
    if (AutoAim_Data_Receive.yaw_aim != 0 || AutoAim_Data_Receive.pitch_aim != 0)
    {

        pitch_angle_err = (-AutoAim_Data_Receive.pitch_aim) - DM_pitch_motor_data.INS_angle;
        PID_calc(&DM_pitch_motor_data.auto_aim_pid, pitch_angle_err, 0);

        DM_pitch_motor_data.INS_speed_set = (-DM_pitch_motor_data.auto_aim_pid.out) + (DM_pitch_motor_data.INS_speed - DM_pitch_motor_data.INS_speed_last) * PITCH_MOTOR_AUTO_AIM_FF;
        DM_pitch_motor_data.INS_angle_set = AutoAim_Data_Receive.pitch_aim;

        pitch_mode = pitch_mode_last = 1;
    }

    // ?????????
    else if (rc_ctrl.rc.s[1] == RC_SW_MID)
    {
        pitch_mode_last = pitch_mode;
        if ((rc_ctrl.rc.ch[1] > 5 || rc_ctrl.rc.ch[1] < -5))
        {
            pitch_mode = 0;
        }
        else
        {
            pitch_mode = 1;
        }

        if (pitch_mode == 0)
        {
            DM_pitch_motor_data.INS_speed_set = (float)rc_ctrl.rc.ch[1] / 660.0f * 5.0f;
        }
        else if (pitch_mode == 1 && pitch_mode_last == 0)
        {
            DM_pitch_motor_data.INS_angle_set = DM_pitch_motor_data.INS_angle;
        }

        if (pitch_mode == 1)
        {
            PID_calc(&DM_pitch_motor_data.auto_aim_pid, DM_pitch_motor_data.INS_angle, DM_pitch_motor_data.INS_angle_set);
            DM_pitch_motor_data.INS_speed_set = DM_pitch_motor_data.auto_aim_pid.out;
        }
    }
    else if (rc_ctrl.rc.s[1] == RC_SW_UP) // ??????pitch?????
    {
        fp32 pitch_up_down_aim = Pitch_Updown();
        PID_calc(&DM_pitch_motor_data.angle_pid, DM_pitch_motor_data.INS_angle, pitch_up_down_aim);
        DM_pitch_motor_data.INS_speed_set = DM_pitch_motor_data.angle_pid.out;
    }

    // DM????????
    if ((DM_pitch_motor_data.p_int > PITCH_ECD_ANGLE_MAX || DM_pitch_motor_data.p_int < PITCH_ECD_ANGLE_MIN) && (DM_pitch_motor_data.p_int < 50000))
    {
        if (DM_pitch_motor_data.p_int < PITCH_ECD_ANGLE_MIN && DM_pitch_motor_data.INS_speed_set > 0)
        {
            DM_pitch_motor_data.INS_speed_set = 0;
            DM_pitch_motor_data.INS_angle_set = DM_pitch_motor_data.INS_angle;
        }
        if (DM_pitch_motor_data.p_int > PITCH_ECD_ANGLE_MAX && DM_pitch_motor_data.INS_speed_set < 0)
        {
            DM_pitch_motor_data.INS_speed_set = 0;
            DM_pitch_motor_data.INS_angle_set = DM_pitch_motor_data.INS_angle;
        }
    }
    PID_calc(&DM_pitch_motor_data.speed_pid, DM_pitch_motor_data.INS_speed, DM_pitch_motor_data.INS_speed_set);
    DM_pitch_motor_data.target_current = -DM_pitch_motor_data.speed_pid.out + Pitch_Gravity_Compensation();
}

float Pitch_Updown(void)
{
    static float auto_pitch_watch = 0;
    static uint8_t updown_switch_flag = 0;
    uint8_t speed_state = AutoAim_Data_Receive.pitch_speed ? 1 : 0;

    const PitchSwingParams swing_params[2] = {{-10.0f, 25.0f, 0.06f}, {-24.0f, -15.0f, 0.04f}};

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
        enable_DM(DM4310_SendID, 0x01);
        enable_send_count--;
    }
}

void Gimbal_Task(void const *argument)
{
    Gimbal_Motor_Pid_Init();
    enable_DM(DM4310_SendID, 0x01);
    rc_ctrl.rc.s[1] = RC_SW_DOWN;
    vTaskDelay(200);

    while (1)
    {
        Check_DM_Auto_Enable();
        Gimbal_Motor_Data_Update();
        //	angle_calculate();

        Yaw_Motor_Control();
        Pitch_Motor_Control();

        if (rc_ctrl.rc.s[1] == RC_SW_DOWN)
        {
            CAN_Gimbal_CMD(0, 0, 0, 0);
            ctrl_motor(DM4310_SendID, 0, 0, 0, 0, 0);
            CAN_Shoot_CMD(0, 0, shoot_motor_3508[0].target_current, shoot_motor_3508[1].target_current);
        }
        else
        {
            CAN_Gimbal_CMD(gimbal_m6020[0].give_current, 0, 0, 0);
            ctrl_motor(DM4310_SendID, 0, 0, 0, 0, DM_pitch_motor_data.target_current);
            CAN_Shoot_CMD(0, shoot_m2006[0].target_current, shoot_motor_3508[0].target_current, shoot_motor_3508[1].target_current);
        }

        //		Vofa_Send_Data4((float)dial_stop_cnt,motor_measure_shoot[2].given_current,shoot_m2006[0].target_current,0);
        //	Vofa_Send_Data4(AutoAim_Data_Receive.yaw_aim,gimbal_m6020[0].INS_angle,(float)AutoAim_Data_Receive.fire_or_not,0);
        //		Vofa_Send_Data4((float)DM_pitch_motor_data.INS_angle_set,(float)DM_pitch_motor_data.INS_angle,DM_pitch_motor_data.INS_speed,DM_pitch_motor_data.INS_speed_set);
        vTaskDelay(1);
    }
}
