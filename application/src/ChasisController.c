#include "ChasisController.h"
#include "GimbalController.h"
Chassis_Control_t chassis_controller;
float yaw_init_angle;

extern GimbalController gimbal_controller;

void InfantryInit(Chassis_Control_t *chassis_controller)
{
    // 功率控制初始化
   // PowerLimitInit(&chassis_controller->power_limiter, 4, M3508, chassis_controller->power_limit_method);

    TD_Init(&chassis_controller->x_v_td, 10000, 0.02); // 约0.5s上升时间
    TD_Init(&chassis_controller->y_v_td, 10000, 0.02);
    TD_Init(&chassis_controller->yaw_v_td, 10000, 0.02);
}

/**
 * @brief  转角限制到±180度
 * @param  输入转角
 * @retval 输出转角
 */
float limit_pi(float in)
{
    while (in < -180.0f || in > 180.0f)
    {
        if (fabs(in - 180.0f) < 1e-4)
        {
            in = 180.0f;
            break;
        }
        else if (in < -180.0f)
        {
            in = in + 360.0f;
        }
        else if (in > 180.0f)
        {
            in = in - 360.0f;
        }
    }
    return in;
}

/**
 * @brief  底盘方向偏差获取
 * @param  目标方向(单位为角度)
 * @retval 方向偏差
 */

float angle_z_err_get(float target_ang, float zeros_angle)
{
    float AngErr_front, AngErr_back, AngErr_left, AngErr_right, minAngle, angleBias = 0.0f;

        AngErr_front = limit_pi(zeros_angle / 22.755555556f - target_ang / 22.755555556f + angleBias);
        AngErr_back = limit_pi(AngErr_front + 180.0f);
        AngErr_left = limit_pi(AngErr_front + GIMBAL_MOTOR_SIGN * 90.0f);
        AngErr_right = limit_pi(AngErr_front - GIMBAL_MOTOR_SIGN * 90.0f);

    return AngErr_front;
}

// 获取控制方向
void getDir()
{
    // 计算与正对情况的夹角，22.755555556f = 8192 / 360.0f，结果转弧度
    float AngErr_front = limit_pi(GIMBAL_FOLLOW_ZERO / 22.755555556f - gimbal_controller.yaw_recv.angle / 22.755555556f) * ANGLE_TO_RAD_COEF;
    chassis_controller.sin_dir = arm_sin_f32(AngErr_front);
    chassis_controller.cos_dir = arm_cos_f32(AngErr_front);
}

void get_sensors_info(Sensors *sensors_info)
{
    for (int i = 0; i < 4; i++)
    {
        M3508_Decode(&sensors_info->wheels_recv[i], &sensors_info->wheels_decode[i], ONLY_SPEED_WITH_REDUCTION, 0.9);
        M3508_Decode(&sensors_info->wheels_recv[i], &sensors_info->wheels_decode_raw[i], ONLY_SPEED_WITHOUT_FILTER_WITH_REDU, 0.9);
    
    }
    motor_communication[0].delta_t = DWT_GetDeltaT(&motor_communication[0].last_cnt);
    chassis_controller.error_angle = angle_z_err_get(gimbal_controller.yaw_recv.angle, GIMBAL_FOLLOW_ZERO) * ANGLE_TO_RAD_COEF;
    getDir();
}


// 加速策略
void wheels_accel(Chassis_Control_t *chassis_controller)
{
    chassis_controller->set_x_v = TD_Calculate(&chassis_controller->x_v_td, chassis_controller->target_x_v);
    chassis_controller->set_y_v = TD_Calculate(&chassis_controller->y_v_td, chassis_controller->target_y_v);
    chassis_controller->set_yaw_v = TD_Calculate(&chassis_controller->yaw_v_td, chassis_controller->target_yaw_v);
}

void chassis_powerdown_control(Chassis_Control_t *chassis_controller)
{
    chassis_controller->set_x_v = 0;
    chassis_controller->set_y_v = 0;
    chassis_controller->set_yaw_v = 0;
    chassis_controller->target_x_v = 0;
    chassis_controller->target_y_v = 0;
    chassis_controller->target_yaw_v = 0;

    for (int i = 0; i < 4; i++)
    {
        chassis_controller->excute_info.steers_set_current[i] = 0;
        chassis_controller->excute_info.wheels_set_current[i] = 0;
    }

    TD_Clear(&chassis_controller->x_v_td, 0);
    TD_Clear(&chassis_controller->y_v_td, 0);
    TD_Clear(&chassis_controller->yaw_v_td, 0);
}

void main_control(Chassis_Control_t *chassis_controller)
{
    static int last_mode = -1;
    
    // 模式切换时重置TD状态
    if (last_mode != remote_controller.control_mode_action)
    {
        TD_Clear(&chassis_controller->x_v_td, chassis_controller->target_x_v);
        TD_Clear(&chassis_controller->y_v_td, chassis_controller->target_y_v);
        TD_Clear(&chassis_controller->yaw_v_td, chassis_controller->target_yaw_v);
        last_mode = remote_controller.control_mode_action;
    }
    
    switch (remote_controller.control_mode_action)
    {
        case NOT_CONTROL_MODE:
            chassis_controller->excute_info.wheels_set_current[0] = 0;
            chassis_controller->excute_info.wheels_set_current[1] = 0;
            chassis_controller->excute_info.wheels_set_current[2] = 0;
            chassis_controller->excute_info.wheels_set_current[3] = 0;
            break;
        case NOT_FOLLOW_GIMBAL:
            omni_chassis_control();
            break;
        case FOLLOW_GIMBAL:
            omni_follow_control_without_chassis();
            break;
        case CV_ROTATE:
            omni_rotate_control();
            break;
        case CHANGE_SPEED_FOLLOW:
            omni_follow_control();
            break;
        case ROTATE_WITH_WORLD_COORDINATE:
            omni_rotate_with_world_coordinate();
            break;
    }
}

static CAN_TxHeaderTypeDef  CAN1FrictSend;
static uint8_t Frict_send_data[8];
void CAN1_3508_Motor_Send(short a,short b,short c,short d)
{
    uint32_t send_mail_box;
    CAN1FrictSend.IDE = CAN_ID_STD;
    CAN1FrictSend.RTR = CAN_RTR_DATA;
    CAN1FrictSend.DLC = 0x08;
    CAN1FrictSend.StdId = 0x200;
    
    a=LIMIT_MAX_MIN(a,16000,-16000);
    b=LIMIT_MAX_MIN(b,16000,-16000);
    c=LIMIT_MAX_MIN(c,16000,-16000);
    d=LIMIT_MAX_MIN(d,16000,-16000);
    Frict_send_data[0] = (unsigned char)((a>>8)&0xff);
    Frict_send_data[1] = (unsigned char)(a&0xff);
    Frict_send_data[2] = (unsigned char)((b>>8)&0xff);
    Frict_send_data[3] = (unsigned char)(b&0xff);
    Frict_send_data[4] = (unsigned char)((c>>8)&0xff);
    Frict_send_data[5] = (unsigned char)(c&0xff);
    Frict_send_data[6] = (unsigned char)((d>>8)&0xff);
    Frict_send_data[7] = (unsigned char)(d&0xff);
    HAL_CAN_AddTxMessage(&hcan1, &CAN1FrictSend, Frict_send_data, &send_mail_box);

}


void execute_control(ExcuteTorque *torque)
{
		uint32_t motor_wait;
        CAN1_3508_Motor_Send( torque->wheels_set_current[0], torque->wheels_set_current[1], torque->wheels_set_current[2], torque->wheels_set_current[3]);
        // 轮

//        CanSend(&hcan1 , torque->wheels_send_data, C620_STD_ID_1_4 ,&motor_tx_header[0], &motor_wait);
//        M3508_SendPack(torque->wheels_send_data, C620_STD_ID_5_8, DJI_3508_MOTORS_2 - 0x204, torque->wheels_set_current[1], SEND_CURRENT);	
//        CanSend(&hcan1 , torque->wheels_send_data, C620_STD_ID_5_8 ,&motor_tx_header[3], &motor_wait);	    
	
	
	
	
	
    
}
