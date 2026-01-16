#include "GimbalTask.h"
#include "Motor_Typdef.h"
#include "robot_config.h"
#include "SignalGenerator.h"
#include "ChasisController.h"
#include "GimbalController.h"
#include "pc_serial.h"
#include "bsp_PWM.h"
SawToothWave saw_tooth_wave;

extern Motor_Communication motor_communication[8];
extern uint8_t Gimbal_Flag;
extern Chassis_Control_t chassis_controller;  // 引用底盘控制器
GimbalController gimbal_controller;
extern PCRecvData pc_recv_data;

/**
 * @brief 云台PID初始化(仅Pitch值)
 * @param[in] void
 */
void GimbalPidInit()
{
    PID_Init(&gimbal_controller.pitch_angle_pid, 300.0f, 0, 0.0f, 32.0f, 1.0, 0.0f, 0, 0, 0, 0.02f, 1, NONE);
    PID_Init(&gimbal_controller.pitch_speed_pid, 9500.0f, 3.0f, 0.0f, -20.0f, 0.0f, 0, 0, 0, 0.0018, 0, 1, Integral_Limit | Trapezoid_Intergral);

    PID_Init(&gimbal_controller.yaw_angle_pid, 500.0, 0, 0, 5.0f, 0.0f, 0.0f, 0, 0, 0.0, 0.02f, 1, DerivativeFilter);
    PID_Init(&gimbal_controller.yaw_speed_pid, 16000, 1600, 0.0, 120.0f, 0.0f, 0.0f, 0, 0, 0.0018, 0, 1, Integral_Limit | Trapezoid_Intergral);

    // 跟踪微分器
    TD_Init(&gimbal_controller.pos_yaw_td, 10000, 0.01);
    TD_Init(&gimbal_controller.speed_yaw_td, 90000, 0.01);
}

/**
 * @brief 云台控制
 * @param[in] set_point 角度值设定 度
 */
void PWM_Init()
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    //HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_3);

    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 12000);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 12000);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 0);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 0);
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 12000);
    //__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, 10000);
}

float Gimbal_Pitch_Calculate(float set_point)
{
    // pitch 三环
    gimbal_controller.set_pitch_angle = set_point;
    gimbal_controller.set_pitch_speed = PID_Calculate(&gimbal_controller.pitch_angle_pid, gimbal_controller.pitch_info.angle, set_point);
    gimbal_controller.set_pitch_current = GIMBAL_PITCH_MOTOR_SIGN * PID_Calculate(&gimbal_controller.pitch_speed_pid, gimbal_controller.pitch_recv.speed, gimbal_controller.set_pitch_speed);
    return gimbal_controller.set_pitch_current;
}

float Gimbal_Yaw_Calculate(float set_point)
{
    float angle_error = set_point - gimbal_controller.gyro_yaw_angle;

    // 处理跨零点情况
    while (angle_error > 180.0f) {
        angle_error -= 360.0f;
    }
    while (angle_error < -180.0f) {
        angle_error += 360.0f;
    }

    // 使用处理后的目标角度
    float corrected_target = gimbal_controller.gyro_yaw_angle + angle_error;

    gimbal_controller.set_yaw_speed = PID_Calculate(&gimbal_controller.yaw_angle_pid, gimbal_controller.gyro_yaw_angle, corrected_target);

    gimbal_controller.set_yaw_current = GIMBAL_YAW_MOTOR_SIGN * (PID_Calculate(&gimbal_controller.yaw_speed_pid, gimbal_controller.yaw_recv.speed, gimbal_controller.set_yaw_speed));

    return gimbal_controller.set_yaw_current;
}

void GimbalClear(void)
{
    PID_Clear(&gimbal_controller.pitch_angle_pid);
    PID_Clear(&gimbal_controller.pitch_speed_pid);
    PID_Clear(&gimbal_controller.pitch_current_pid);

    Feedforward_Clear(&gimbal_controller.pitch_speed_forward);
    Feedforward_Clear(&gimbal_controller.pitch_angle_forward);

    gimbal_controller.target_pitch_angle = gimbal_controller.gyro_pitch_angle;
    gimbal_controller.set_pitch_angle = gimbal_controller.gyro_pitch_angle;
    gimbal_controller.set_pitch_speed = 0;
    gimbal_controller.set_pitch_current = 0;
    gimbal_controller.comp_pitch_current = 0;

    // yaw
    PID_Clear(&gimbal_controller.yaw_angle_pid);
    PID_Clear(&gimbal_controller.yaw_speed_pid);

    Feedforward_Clear(&gimbal_controller.yaw_speed_forward);
    Feedforward_Clear(&gimbal_controller.yaw_angle_forward);

    TD_Clear(&gimbal_controller.pos_yaw_td, gimbal_controller.gyro_yaw_angle);

    gimbal_controller.target_yaw_angle = 0; //gimbal_controller.gyro_yaw_angle;
    gimbal_controller.set_yaw_angle = 0;    //gimbal_controller.gyro_yaw_angle;
    gimbal_controller.set_yaw_speed = 0;
    gimbal_controller.set_yaw_current = 0;
}

/**
 * @brief 更新pitch角速度，以及角度(注意需要标定零点)
 */
void updateGyro()
{
    // 注意陀螺仪安装的Pitch和roll轴方向
    gimbal_controller.delta_t = DWT_GetDeltaT(&gimbal_controller.last_cnt);
    gimbal_controller.gyro_pitch_angle = GIMBAL_PITCH_GYRO_SIGN * (INS.Pitch - GIMBAL_PITCH_BIAS);
    float speed = (gimbal_controller.gyro_pitch_angle - gimbal_controller.gyro_last_pitch_angle) / gimbal_controller.delta_t;

    iir(&gimbal_controller.gyro_pitch_speed, speed, 0.5);
    gimbal_controller.gyro_last_pitch_angle = gimbal_controller.gyro_pitch_angle;

    // yaw
    gimbal_controller.gyro_yaw_angle = GIMBAL_YAW_GYRO_SIGN * INS.YawTotalAngle;
    speed = (gimbal_controller.gyro_yaw_angle - gimbal_controller.gyro_last_yaw_angle) / gimbal_controller.delta_t;

    iir(&gimbal_controller.gyro_yaw_speed, speed, 0.4);
    gimbal_controller.gyro_last_yaw_angle = gimbal_controller.gyro_yaw_angle;
}

float cur_pitch;
float cur_yaw;
int init_flag = 0;
float time_now;
int increasing = 1;
float target_high = 280.0f; // 增加目标值60度
float target_low = 80.0f;    // 减少目标值-10度

int increasing1 = 1;
float target_high1 = 3.0f;  // 增加目标值60度
float target_low1 = -3.0f;  // 减少目标值-10度

int init_yaw = 40;

// Pitch轴相对位置控制相关变量
static int pitch_init_flag = 0;           // Pitch轴初始化标志
static float pitch_init_angle = 0.0f;     // Pitch轴初始角度（上电时记录）
static const float PITCH_RELATIVE_MAX = 20.0f;  // 相对上限10度
static const float PITCH_RELATIVE_MIN = -25.0f; // 相对下限-10度

/**
 * @brief 限制Pitch角度在相对范围内
 * @param target_angle 目标角度
 * @return 限制后的角度
 */
float limit_pitch_relative_angle(float target_angle)
{
    if (!pitch_init_flag) {
        return target_angle; // 如果还未初始化，返回原值
    }

    // 计算相对于初始位置的角度
    float relative_angle = target_angle - pitch_init_angle;

    // 限制在±10度范围内
    relative_angle = LIMIT_MAX_MIN(relative_angle, PITCH_RELATIVE_MAX, PITCH_RELATIVE_MIN);

    // 返回限制后的绝对角度
    return pitch_init_angle + relative_angle;
}

void GimbalTask(void *pvParameters)
{
    portTickType xLastWakeTime;
    const portTickType xFrequency = 2; // 1000HZ
    SawToothInit(&saw_tooth_wave, 50, 1, 100, 0);

    GimbalPidInit();
    gimbal_controller.target_yaw_angle = init_yaw;

    DWT_GetDeltaT(&gimbal_controller.last_cnt);

    vTaskDelay(2000);

    // 初始化Pitch轴的初始位置（上电后的第一个位置）
    if (!pitch_init_flag) {
        GimbalupdataSensors(); // 确保获取到有效的传感器数据
        pitch_init_angle = gimbal_controller.pitch_info.angle; // 记录初始角度
        gimbal_controller.target_pitch_angle = pitch_init_angle; // 设置初始目标为当前位置
        pitch_init_flag = 1; // 标记已初始化
    }

    while (1) // 这个里面写解码过程，因为暂时只开了一个串口，就先只写了一个通信，后续用cubemx再开一个接收二维码数据
    {
        xLastWakeTime = xTaskGetTickCount();

        GimbalupdataSensors();
        gimbal_controller.delta_t = DWT_GetDeltaT(&gimbal_controller.last_cnt);

        //根据阶段增加或减少
        switch (remote_controller.gimbal_action)
        {
            case GIBAL_SENTRY_MODE:

                // 检查是否接收到NUC数据
                if (pc_recv_data.is_valid)
                {
                    // 接收到NUC数据，使用NUC指定的目标角度（需要限制在相对范围内）
                    gimbal_controller.target_pitch_angle = limit_pitch_relative_angle(pc_recv_data.pitch);
                    gimbal_controller.target_yaw_angle = pc_recv_data.yaw;
                    if (pc_recv_data.should_shoot)
                    {
                        Set_LED_Brightness(80.0);
                        buzzer_beep(1000, 100);
                    }
                    else
                    {
                        buzzer_off();
                        Set_LED_Off();
                    }
                }
                else
                {
                    // 未接收到NUC数据，检查是否满足自动扫描条件
                    if (chassis_controller.Preset_Time > 4 && chassis_controller.Preset_Time < 4.2)
                    {
                        gimbal_controller.target_yaw_angle = init_yaw;
                    }
                    else
                    {
                        if (chassis_controller.Preset_Time > 4)
                        {
                            // 满足时间条件，执行自动扫描模式
                            // YAW轴左右扫描
                            if (increasing)
                            {
                                gimbal_controller.target_yaw_angle += 0.15;
                                if (gimbal_controller.target_yaw_angle >= target_high)
                                {
                                    increasing = 0;        // 切换到减少阶段
                                    gimbal_controller.target_yaw_angle = target_high; // 确保不超出目标值
                                }
                            }
                            else
                            {
                                gimbal_controller.target_yaw_angle -= 0.15;
                                if (gimbal_controller.target_yaw_angle <= target_low)
                                {
                                    increasing = 1;       // 切换到增加阶段
                                    gimbal_controller.target_yaw_angle = target_low; // 确保不低于目标值
                                }
                            }
                            // PITCH轴上下扫描（可选，根据需要启用）
                            if (increasing1) {
                                float temp_pitch = gimbal_controller.target_pitch_angle + 0.05;
                                temp_pitch = limit_pitch_relative_angle(temp_pitch); // 应用相对限位
                                if (temp_pitch >= (pitch_init_angle + target_high1)) {
                                    increasing1 = 0;        // 切换到减少阶段
                                }
                                gimbal_controller.target_pitch_angle = temp_pitch;
                            }
                            else {
                                float temp_pitch = gimbal_controller.target_pitch_angle - 0.05;
                                temp_pitch = limit_pitch_relative_angle(temp_pitch); // 应用相对限位
                                if (temp_pitch <= (pitch_init_angle + target_low1)) {
                                    increasing1 = 1;       // 切换到增加阶段
                                }
                                gimbal_controller.target_pitch_angle = temp_pitch;
                            }
                        }
                    }
                }

                // 执行云台控制
                cur_pitch = Gimbal_Pitch_Calculate(gimbal_controller.target_pitch_angle);
                cur_yaw = Gimbal_Yaw_Calculate(gimbal_controller.target_yaw_angle);
                execute_gimbal(&gimbal_controller);
                break;

            case GIMBAL_ACT_MODE:
                if (remote_controller.gimbal_action != remote_controller.last_gimbal_action)
                {
                    gimbal_controller.target_yaw_angle = init_yaw;
                    // 模式切换时，确保pitch角度在相对限位范围内
                    gimbal_controller.target_pitch_angle = limit_pitch_relative_angle(gimbal_controller.target_pitch_angle);
                }

                // 对pitch角度应用相对限位（处理来自遥控器或其他输入的角度变化）
                gimbal_controller.target_pitch_angle = limit_pitch_relative_angle(gimbal_controller.target_pitch_angle);

                cur_pitch = Gimbal_Pitch_Calculate(gimbal_controller.target_pitch_angle);
                cur_yaw = Gimbal_Yaw_Calculate(gimbal_controller.target_yaw_angle);
                execute_gimbal(&gimbal_controller);

                break;
        }
        SendtoPC();
        /*  延时  */
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
