#include "mecanum.h"
//#include "PowerLimit.h"
#include "ChasisController.h"

void mecanum_pid_init()
{
    // 转向PID初始化
    PID_Init(&chassis_controller.turn_pid, 50, 0, 0, 5, 0.1, 0, 0, 0, 0.001, 0.009, 1, DerivativeFilter);

//    // 底盘跟随前馈初始化
//    chassis_controller.Mecanum_Follow_FF_Coefficient[0] = -0.9f;
//    chassis_controller.Mecanum_Follow_FF_Coefficient[1] = -0.0f;
//    chassis_controller.Mecanum_Follow_FF_Coefficient[2] = -0.0f;
//    Feedforward_Init(&chassis_controller.Mecanum_Follow_FF, 1.0f, chassis_controller.Mecanum_Follow_FF_Coefficient, 0.01, 0, 0);

    // 轮子控制PID
    PID_Init(&chassis_controller.wheels_pid[LEFT_UP_MECANUM_WHEEL], C620_MAX_SEND_CURRENT, 0, 0, 5.0f, 0.6f, 0, 0, 0, 0, 0, 1, NONE);
    PID_Init(&chassis_controller.wheels_pid[RIGHT_UP_MECANUM_WHEEL], C620_MAX_SEND_CURRENT, 0, 0, 5.0f, 0.6f, 0, 0, 0, 0, 0, 1, NONE);
    PID_Init(&chassis_controller.wheels_pid[LEFT_DOWN_MECANUM_WHEEL], C620_MAX_SEND_CURRENT, 0, 0, 5.0f, 0.6f, 0, 0, 0, 0, 0, 1, NONE);
    PID_Init(&chassis_controller.wheels_pid[RIGHT_DOWN_MECANUM_WHEEL], C620_MAX_SEND_CURRENT, 0, 0, 5.0f, 0.6f, 0, 0, 0, 0, 0, 1, NONE);
}

// 正运动学
void mecanum_pos_kinematics()
{
    // 计算正运动学，并转到m/s为单位
    float speed1 = chassis_controller.sensors_info.wheels_decode[LEFT_UP_MECANUM_WHEEL].speed;
    float speed2 = chassis_controller.sensors_info.wheels_decode[RIGHT_UP_MECANUM_WHEEL].speed;
    float speed3 = chassis_controller.sensors_info.wheels_decode[LEFT_DOWN_MECANUM_WHEEL].speed;
    float speed4 = chassis_controller.sensors_info.wheels_decode[RIGHT_DOWN_MECANUM_WHEEL].speed;

    // 以向前(y)，向右(x)，逆时针(yaw)
    chassis_controller.y_v = (speed1 - speed2 + speed3 - speed4) / 4 * MECANUM_DEGEREE_S_TO_MS * _DIVIDE_SQRT_2;
    chassis_controller.x_v = (speed1 + speed2 - speed3 - speed4) / 4 * MECANUM_DEGEREE_S_TO_MS * _DIVIDE_SQRT_2;
    chassis_controller.yaw_v = (-speed1 - speed2 - speed3 - speed4) / 4 / (MECANUM_WIDTH + MECANUM_LENGTH) * MECANUM_DEGEREE_S_TO_MS; // rad/s
}


// 逆运动学
void mecanum_inv_kinematics()
{
    // 逆运动学解算,统一到度/s的单位
    chassis_controller.wheels_set_v[LEFT_UP_MECANUM_WHEEL] = (chassis_controller.set_x_v * SQRT_2 + chassis_controller.set_y_v * SQRT_2 - chassis_controller.set_yaw_v * (MECANUM_WIDTH + MECANUM_LENGTH)) * MECANUM_SPEED_TO_DEGEREE_S;
    chassis_controller.wheels_set_v[RIGHT_UP_MECANUM_WHEEL] = (chassis_controller.set_x_v * SQRT_2 - chassis_controller.set_y_v * SQRT_2 - chassis_controller.set_yaw_v * (MECANUM_WIDTH + MECANUM_LENGTH)) * MECANUM_SPEED_TO_DEGEREE_S;
    chassis_controller.wheels_set_v[LEFT_DOWN_MECANUM_WHEEL] = (-chassis_controller.set_x_v * SQRT_2 + chassis_controller.set_y_v * SQRT_2 - chassis_controller.set_yaw_v * (MECANUM_WIDTH + MECANUM_LENGTH)) * MECANUM_SPEED_TO_DEGEREE_S;
    chassis_controller.wheels_set_v[RIGHT_DOWN_MECANUM_WHEEL] = (-chassis_controller.set_x_v * SQRT_2 - chassis_controller.set_y_v * SQRT_2 - chassis_controller.set_yaw_v * (MECANUM_WIDTH + MECANUM_LENGTH)) * MECANUM_SPEED_TO_DEGEREE_S;
		
}



// 仅底盘运动策略，底盘将按照规定正方向运动，通常用于测试或者小陀螺状态
void mecanum_chassis_control()
{
    // 正运动学解算
    mecanum_pos_kinematics();

    chassis_controller.set_x_v = chassis_controller.target_x_v;
    chassis_controller.set_y_v = chassis_controller.target_y_v;
    chassis_controller.set_yaw_v = chassis_controller.target_yaw_v;
		chassis_controller.target_pid_yaw_v = 1.0*(PID_Calculate(&chassis_controller.turn_pid, chassis_controller.error_angle, 0));
    chassis_controller.set_yaw_v = chassis_controller.target_pid_yaw_v;
    // 逆运动学解算
    mecanum_inv_kinematics();


    // PID 计算
    for (int i = 0; i < 4; i++)
    {
			
        chassis_controller.excute_info.wheels_set_current[i] = PID_Calculate(&chassis_controller.wheels_pid[i], chassis_controller.sensors_info.wheels_decode[i].speed, chassis_controller.wheels_set_v[i]);
    }
}
