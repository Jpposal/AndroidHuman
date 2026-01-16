#include "omni.h"
#include "accel.h"
#include "ChasisController.h"

void omni_pid_init()
{
    // ??PID???
    PID_Init(&chassis_controller.turn_pid, 50, 1, 0, 10, 0.1, 0, 0, 0, 0.001, 0.002, 1, DerivativeFilter);

    // ????PID
    PID_Init(&chassis_controller.wheels_pid[LEFT_UP_OMNI_WHEEL], C620_MAX_SEND_CURRENT, C620_MAX_SEND_CURRENT, 0, 10.0f, 0.0f, 0, 0, 0, 0, 0, 1, NONE);
    PID_Init(&chassis_controller.wheels_pid[RIGHT_UP_OMNI_WHEEL], C620_MAX_SEND_CURRENT, C620_MAX_SEND_CURRENT, 0, 10.0f, 0.0f, 0, 0, 0, 0, 0, 1, NONE);
    PID_Init(&chassis_controller.wheels_pid[LEFT_DOWN_OMNI_WHEEL], C620_MAX_SEND_CURRENT, C620_MAX_SEND_CURRENT, 0, 10.0f, 0.0f, 0, 0, 0, 0, 0, 1, NONE);
    PID_Init(&chassis_controller.wheels_pid[RIGHT_DOWN_OMNI_WHEEL], C620_MAX_SEND_CURRENT, C620_MAX_SEND_CURRENT, 0, 10.0f, 0.0f, 0, 0, 0, 0, 0, 1, NONE);
}

// ????
void omni_pos_kinematics()
{
    // ??????,???m/s???
    float speed1 = chassis_controller.sensors_info.wheels_decode[LEFT_UP_OMNI_WHEEL].speed;
    float speed2 = chassis_controller.sensors_info.wheels_decode[RIGHT_UP_OMNI_WHEEL].speed;
    float speed3 = chassis_controller.sensors_info.wheels_decode[LEFT_DOWN_OMNI_WHEEL].speed;
    float speed4 = chassis_controller.sensors_info.wheels_decode[RIGHT_DOWN_OMNI_WHEEL].speed;

    // ???(y),??(x),???(yaw)
    chassis_controller.y_v = (-speed1 + speed2 - speed3 + speed4) / 4 * OMNI_DEGEREE_S_TO_MS * _DIVIDE_SQRT_2;
    chassis_controller.x_v = (-speed1 - speed2 + speed3 + speed4) / 4 * OMNI_DEGEREE_S_TO_MS * _DIVIDE_SQRT_2;
    chassis_controller.yaw_v = (speed1 + speed2 + speed3 + speed4) / 4 / OMNI_RADIUS * OMNI_DEGEREE_S_TO_MS; // rad/s
}

// ???? X?
void omni_inv_kinematics()
{
    // ??????,????/s???
    chassis_controller.wheels_set_v[LEFT_UP_OMNI_WHEEL] = (-chassis_controller.set_x_v * SQRT_2 - chassis_controller.set_y_v * SQRT_2 + chassis_controller.set_yaw_v * OMNI_RADIUS) * OMNI_SPEED_TO_DEGEREE_S;
    chassis_controller.wheels_set_v[RIGHT_UP_OMNI_WHEEL] = (-chassis_controller.set_x_v * SQRT_2 + chassis_controller.set_y_v * SQRT_2 + chassis_controller.set_yaw_v * OMNI_RADIUS) * OMNI_SPEED_TO_DEGEREE_S;
    chassis_controller.wheels_set_v[LEFT_DOWN_OMNI_WHEEL] = (chassis_controller.set_x_v * SQRT_2 + chassis_controller.set_y_v * SQRT_2 + chassis_controller.set_yaw_v * OMNI_RADIUS) * OMNI_SPEED_TO_DEGEREE_S;
    chassis_controller.wheels_set_v[RIGHT_DOWN_OMNI_WHEEL] = (chassis_controller.set_x_v * SQRT_2 - chassis_controller.set_y_v * SQRT_2 + chassis_controller.set_yaw_v * OMNI_RADIUS) * OMNI_SPEED_TO_DEGEREE_S;
}

// 世界坐标系逆运动学解算（考虑底盘旋转角度）
void omni_inv_kinematics_world_coordinate(float world_x_v, float world_y_v, float yaw_v, float chassis_angle)
{
    // 将世界坐标系速度转换到底盘坐标系
    float cos_angle = arm_cos_f32(chassis_angle);
    float sin_angle = arm_sin_f32(chassis_angle);
    
    float chassis_x_v = world_x_v * cos_angle + world_y_v * sin_angle;
    float chassis_y_v = -world_x_v * sin_angle + world_y_v * cos_angle;
    
    // 使用转换后的底盘坐标系速度进行逆运动学解算
    chassis_controller.wheels_set_v[LEFT_UP_OMNI_WHEEL] = (-chassis_x_v * SQRT_2 - chassis_y_v * SQRT_2 + yaw_v * OMNI_RADIUS) * OMNI_SPEED_TO_DEGEREE_S;
    chassis_controller.wheels_set_v[RIGHT_UP_OMNI_WHEEL] = (-chassis_x_v * SQRT_2 + chassis_y_v * SQRT_2 + yaw_v * OMNI_RADIUS) * OMNI_SPEED_TO_DEGEREE_S;
    chassis_controller.wheels_set_v[LEFT_DOWN_OMNI_WHEEL] = (chassis_x_v * SQRT_2 + chassis_y_v * SQRT_2 + yaw_v * OMNI_RADIUS) * OMNI_SPEED_TO_DEGEREE_S;
    chassis_controller.wheels_set_v[RIGHT_DOWN_OMNI_WHEEL] = (chassis_x_v * SQRT_2 - chassis_y_v * SQRT_2 + yaw_v * OMNI_RADIUS) * OMNI_SPEED_TO_DEGEREE_S;
}



void omni_chassis_control()
{
    omni_pos_kinematics();

    chassis_controller.set_x_v = chassis_controller.target_x_v;
    chassis_controller.set_y_v = chassis_controller.target_y_v;
	chassis_controller.set_yaw_v = chassis_controller.target_yaw_v;
    wheels_accel(&chassis_controller);
    omni_inv_kinematics();
    for (int i = 0; i < 4; i++)
    {
        chassis_controller.excute_info.wheels_set_current[i] = PID_Calculate(&chassis_controller.wheels_pid[i], chassis_controller.sensors_info.wheels_decode[i].speed, chassis_controller.wheels_set_v[i]);
    }
}

// 速度平滑函数
void velocity_smooth_control(float target_x, float target_y, float target_yaw)
{
    // 平滑系数，越小越平滑 (建议0.05-0.2)
    float alpha = 0.1f;
    
    // 一阶低通滤波
    chassis_controller.vel_smoother.smooth_x_v = alpha * target_x + (1 - alpha) * chassis_controller.vel_smoother.smooth_x_v;
    chassis_controller.vel_smoother.smooth_y_v = alpha * target_y + (1 - alpha) * chassis_controller.vel_smoother.smooth_y_v;
    chassis_controller.vel_smoother.smooth_yaw_v = alpha * target_yaw + (1 - alpha) * chassis_controller.vel_smoother.smooth_yaw_v;
}

float x_vel_test = 0;
float x_vel_test_0 = 0;
float x_vel_test_1 = 0;
float y_vel_test = 0;
float y_vel_test_0 = 0;
float y_vel_test_1 = 0;
// 底盘跟随策略
void omni_follow_control()
{
    omni_pos_kinematics();

    chassis_controller.set_x_v = chassis_controller.target_x_v * chassis_controller.cos_dir + chassis_controller.target_y_v * chassis_controller.sin_dir;
	chassis_controller.set_y_v = chassis_controller.target_y_v * chassis_controller.cos_dir - chassis_controller.target_x_v * chassis_controller.sin_dir;
    
    float limited_error_angle = chassis_controller.error_angle;
    const float MAX_ERROR_ANGLE = 45.0f;
    limited_error_angle = LIMIT_MAX_MIN(limited_error_angle, MAX_ERROR_ANGLE, -MAX_ERROR_ANGLE);
    
    chassis_controller.target_pid_yaw_v = GIMBAL_MOTOR_SIGN * PID_Calculate(&chassis_controller.turn_pid, limited_error_angle, 0);
    
    const float MAX_YAW_SPEED = 5.0f;
    chassis_controller.target_pid_yaw_v = LIMIT_MAX_MIN(chassis_controller.target_pid_yaw_v, MAX_YAW_SPEED, -MAX_YAW_SPEED);
    
    chassis_controller.set_yaw_v = -chassis_controller.target_pid_yaw_v;
	    
		velocity_smooth_control(chassis_controller.set_x_v, 
                           chassis_controller.set_y_v, 
                           chassis_controller.set_yaw_v);

	chassis_controller.set_x_v = chassis_controller.vel_smoother.smooth_x_v;
    chassis_controller.set_y_v = chassis_controller.vel_smoother.smooth_y_v;
    chassis_controller.set_yaw_v = chassis_controller.vel_smoother.smooth_yaw_v;
    
    omni_inv_kinematics();

    for (int i = 0; i < 4; i++)
    {
        chassis_controller.excute_info.wheels_set_current[i] = PID_Calculate(&chassis_controller.wheels_pid[i], chassis_controller.sensors_info.wheels_decode[i].speed, chassis_controller.wheels_set_v[i]);
    }

}
void omni_follow_control_without_chassis()
{
    // 正运动学解算
    omni_pos_kinematics();

    chassis_controller.set_x_v = chassis_controller.target_x_v * chassis_controller.cos_dir - chassis_controller.target_y_v * chassis_controller.sin_dir;
    chassis_controller.set_y_v = chassis_controller.target_y_v * chassis_controller.cos_dir + chassis_controller.target_x_v * chassis_controller.sin_dir;
    chassis_controller.set_yaw_v = 0;

    // 逆运动学解算
    omni_inv_kinematics();

    // PID 计算
    for (int i = 0; i < 4; i++)
    {
        chassis_controller.excute_info.wheels_set_current[i] = PID_Calculate(&chassis_controller.wheels_pid[i], chassis_controller.sensors_info.wheels_decode[i].speed, chassis_controller.wheels_set_v[i]);
    }
}
void omni_rotate_control()
{
    // 正运动学解算
    omni_pos_kinematics();

    chassis_controller.set_x_v = chassis_controller.target_x_v * chassis_controller.cos_dir - chassis_controller.target_y_v * chassis_controller.sin_dir;
    chassis_controller.set_y_v = chassis_controller.target_y_v * chassis_controller.cos_dir + chassis_controller.target_x_v * chassis_controller.sin_dir;
	chassis_controller.set_yaw_v = chassis_controller.target_yaw_v;

    // 逆运动学解算
    omni_inv_kinematics();

    // PID 计算
    for (int i = 0; i < 4; i++)
    {
        chassis_controller.excute_info.wheels_set_current[i] = PID_Calculate(&chassis_controller.wheels_pid[i], chassis_controller.sensors_info.wheels_decode[i].speed, chassis_controller.wheels_set_v[i]);
    }
}

// 底盘自转同时沿世界坐标系平移控制
void omni_rotate_with_world_coordinate()
{
    // 正运动学解算
    omni_pos_kinematics();

    // 获取底盘的累计旋转角度
    static float chassis_world_angle = 0.0f;
    chassis_world_angle += chassis_controller.yaw_v * chassis_controller.delta_t; // 积分获得底盘角度
    
    // 应用速度平滑处理到世界坐标系速度
    velocity_smooth_control(chassis_controller.target_x_v, 
                           chassis_controller.target_y_v, 
                           chassis_controller.target_yaw_v);

    float smooth_world_x_v = chassis_controller.vel_smoother.smooth_x_v;
    float smooth_world_y_v = chassis_controller.vel_smoother.smooth_y_v;
    float smooth_yaw_v = chassis_controller.vel_smoother.smooth_yaw_v;

    // 使用世界坐标系逆运动学解算
    omni_inv_kinematics_world_coordinate(smooth_world_x_v, smooth_world_y_v, smooth_yaw_v, chassis_world_angle);

    // PID 计算
    for (int i = 0; i < 4; i++)
    {
        chassis_controller.excute_info.wheels_set_current[i] = PID_Calculate(&chassis_controller.wheels_pid[i], chassis_controller.sensors_info.wheels_decode[i].speed, chassis_controller.wheels_set_v[i]);
    }
}

/**
 * @brief 角度误差平滑处理函数
 * @param error_angle 原始角度误差
 * @param max_angle 最大允许角度
 * @param smooth_zone 平滑过渡区间
 * @return 平滑后的角度误差
 */
float smooth_angle_error(float error_angle, float max_angle, float smooth_zone)
{
    float abs_error = fabsf(error_angle);
    float sign = error_angle >= 0 ? 1.0f : -1.0f;
    
    if (abs_error <= smooth_zone) {
        // 线性区间，直接返回
        return error_angle;
    } else if (abs_error <= max_angle) {
        // 平滑过渡区间，使用三次多项式平滑
        float t = (abs_error - smooth_zone) / (max_angle - smooth_zone);
        float smooth_factor = 1.0f - t * t * (3.0f - 2.0f * t); // 3t?-2t?
        float smooth_error = smooth_zone + (max_angle - smooth_zone) * (1.0f - smooth_factor);
        return sign * smooth_error;
    } else {
        // 超出最大角度，直接限制
        return sign * max_angle;
    }
}

/**
 * @brief 转向速度渐变控制
 * @param current_speed 当前转向速度
 * @param target_speed 目标转向速度  
 * @param max_accel 最大角加速度
 * @param dt 时间步长
 * @return 限制后的转向速度
 */
float yaw_speed_ramp_control(float current_speed, float target_speed, float max_accel, float dt)
{
    static float last_speed = 0.0f;
    
    float speed_diff = target_speed - last_speed;
    float max_change = max_accel * dt;
    
    if (speed_diff > max_change) {
        last_speed += max_change;
    } else if (speed_diff < -max_change) {
        last_speed -= max_change;
    } else {
        last_speed = target_speed;
    }
    
    return last_speed;
}