/**
 ******************************************************************************
 * @file    ChassisSolver.c
 * @brief   底盘解算器
 ******************************************************************************
 * @attention
 ******************************************************************************
 */

#include "ChassisSolver.h"
#include <math.h>

// 外部变量声明 - 来自GimbalTask.c
extern int Is_Detected;
extern uint8_t PCInitFlag;
extern float aim_pitch, aim_yaw;
extern uint8_t system_started;
ChassisSolver chassis_solver;
static uint8_t first_start =  0 ;
int cycle_position = 0; // 周期位置

void Sentry_Vel_Preset_0(void)
{
    if(chassis_controller.preset_mode != Preset_0 || (system_started && !first_start))
    {
        chassis_controller.preset_mode = Preset_0;
        chassis_controller.Preset_Time_Point = chassis_controller.ActionTask_Time; // 记录预设模式时间点
        chassis_controller.Preset_Time = 0.0f; // 重置预设模式
							first_start = 1;
    }else{
				chassis_controller.Preset_Time = DWT_GetTimeline_s() - chassis_controller.Preset_Time_Point;
		}

    if (chassis_controller.Preset_Time < 1) {
        chassis_controller.target_x_v = 0;
        chassis_controller.target_y_v = 0;
        chassis_controller.target_yaw_v = 0;
    } 
    else if (chassis_controller.Preset_Time < 2.6) {
        chassis_controller.target_x_v = 0.0;
        chassis_controller.target_y_v = -0.99;
        chassis_controller.target_yaw_v = 0.0; 
    }else if (chassis_controller.Preset_Time < 3.6) {
        chassis_controller.target_x_v = -0.99;
        chassis_controller.target_y_v = 0.0;
        chassis_controller.target_yaw_v = 0.0; 
    }else 
    {
        // 计算在4秒周期中的位置，确保左右运动距离相等
        cycle_position = ((int)chassis_controller.Preset_Time - 4) % 4;
        chassis_controller.target_yaw_v = 0.0; 
        // 根据周期位置设置运动方向，确保左右距离相等
        if (cycle_position < 2) {
            // 左移2秒
            chassis_controller.target_x_v = -0.3;
            chassis_controller.target_y_v = 0.0;
        } else {
            // 右移2秒
            chassis_controller.target_x_v = 0.3;
            chassis_controller.target_y_v = 0.0;
        }
    }
    chassis_controller.last_preset_mode = Preset_0;
}

void setAllModeOff()
{
    setRobotState(OFFLINE_MODE);
    setControlModeAction(NOT_CONTROL_MODE);
    setShootAction(SHOOT_POWERDOWN_MODE);
    setGimbalAction(GIMBAL_POWERDOWN);
}
void Sentry_Mode_Update(ChassisSolver *infantry)
{
    int leg_len_switch = 0;
    switch (remote_controller.dji_remote.rc.s[LEFT_SW])
    {
        case Down:
            setAllModeOff();
				chassis_controller.last_preset_mode = -1;
				chassis_controller.preset_mode = -1;
            break;
        case Mid:
            setRobotState(CONTROL_MODE);
            setControlModeAction(NOT_FOLLOW_GIMBAL);
            setShootAction(SHOOT_POWERDOWN_MODE);
            setGimbalAction(GIBAL_SENTRY_MODE);
            setSuperPower(POWER_TO_BATTERY);
            Sentry_Vel_Preset_0();
            break;
        case Up:
            setRobotState(CONTROL_MODE);
            setControlModeAction(NOT_FOLLOW_GIMBAL);
            setShootAction(SHOOT_POWERDOWN_MODE);
            setGimbalAction(GIBAL_SENTRY_MODE);
            setSuperPower(POWER_TO_BATTERY);

            break;
        
    }
}
void DJIRemoteUpdate(ChassisSolver *infantry)
{
    int leg_len_switch = 0;
    if(remote_controller.control_type == DJI_REMOTE_CONTROL){
    switch (remote_controller.dji_remote.rc.s[LEFT_SW])
    {
        case Down:
            setRobotState(CONTROL_MODE);
            setControlModeAction(CHANGE_SPEED_FOLLOW);
            setShootAction(SHOOT_POWERDOWN_MODE);
            setGimbalAction(GIMBAL_ACT_MODE);
            setSuperPower(POWER_TO_BATTERY);

            // 云台控制
            gimbal_controller.target_yaw_angle -= (remote_controller.dji_remote.rc.ch[LEFT_CH_LR] - CH_MIDDLE) * MAX_SW_YAW_SPEED / CH_RANGE * infantry->delta_t;
            gimbal_controller.target_pitch_angle += (remote_controller.dji_remote.rc.ch[LEFT_CH_UD] - CH_MIDDLE) * MAX_SW_PITCH_SPEED / CH_RANGE * infantry->delta_t;
            
            if(gimbal_controller.target_yaw_angle > 360.0f)
            {
                gimbal_controller.target_yaw_angle -= 360.0f;
            }
            else if(gimbal_controller.target_yaw_angle < -0.0f)
            {
                gimbal_controller.target_yaw_angle += 360.0f;
            }
						chassis_controller.target_yaw_v = 0;

            // 底盘控制
            chassis_solver.chassis_speed_x = (remote_controller.dji_remote.rc.ch[RIGHT_CH_LR] - CH_MIDDLE) * 1.0f / CH_RANGE;
            chassis_solver.chassis_speed_y = -(remote_controller.dji_remote.rc.ch[RIGHT_CH_UD] - CH_MIDDLE) * 1.0f / CH_RANGE;
            
						chassis_controller.target_x_v = chassis_solver.chassis_speed_x;
	        chassis_controller.target_y_v = chassis_solver.chassis_speed_y;
						break;
        case Mid:
            setRobotState(CONTROL_MODE);
            setControlModeAction(CHANGE_SPEED_FOLLOW);
            setShootAction(SHOOT_POWERDOWN_MODE);
            setGimbalAction(GIMBAL_ACT_MODE);
            setSuperPower(POWER_TO_BATTERY);

            // 云台控制
            gimbal_controller.target_yaw_angle -= (remote_controller.dji_remote.rc.ch[LEFT_CH_LR] - CH_MIDDLE) * MAX_SW_YAW_SPEED / CH_RANGE * infantry->delta_t;
            gimbal_controller.target_pitch_angle += (remote_controller.dji_remote.rc.ch[LEFT_CH_UD] - CH_MIDDLE) * MAX_SW_PITCH_SPEED / CH_RANGE * infantry->delta_t;
            
            if(gimbal_controller.target_yaw_angle > 360.0f)
            {
                gimbal_controller.target_yaw_angle -= 360.0f;
            }
            else if(gimbal_controller.target_yaw_angle < -0.0f)
            {
                gimbal_controller.target_yaw_angle += 360.0f;
            }
						chassis_controller.target_yaw_v = 0;

            // 底盘控制
            chassis_solver.chassis_speed_x = (remote_controller.dji_remote.rc.ch[RIGHT_CH_LR] - CH_MIDDLE) * 1.0f / CH_RANGE;
            chassis_solver.chassis_speed_y = -(remote_controller.dji_remote.rc.ch[RIGHT_CH_UD] - CH_MIDDLE) * 1.0f / CH_RANGE;

            chassis_controller.target_x_v = chassis_solver.chassis_speed_x;
	        chassis_controller.target_y_v = chassis_solver.chassis_speed_y;

            break;
        case Up:
            setRobotState(CONTROL_MODE);
            setControlModeAction(CHANGE_SPEED_FOLLOW);
            setShootAction(SHOOT_POWERDOWN_MODE);
            setGimbalAction(GIMBAL_ACT_MODE);
            setSuperPower(POWER_TO_BATTERY);

            // 云台控制
            gimbal_controller.target_yaw_angle -= (remote_controller.dji_remote.rc.ch[LEFT_CH_LR] - CH_MIDDLE) * MAX_SW_YAW_SPEED / CH_RANGE * infantry->delta_t;
            gimbal_controller.target_pitch_angle += (remote_controller.dji_remote.rc.ch[LEFT_CH_UD] - CH_MIDDLE) * MAX_SW_PITCH_SPEED / CH_RANGE * infantry->delta_t;
            
            if(gimbal_controller.target_yaw_angle > 360.0f)
            {
                gimbal_controller.target_yaw_angle -= 360.0f;
            }
            else if(gimbal_controller.target_yaw_angle < -0.0f)
            {
                gimbal_controller.target_yaw_angle += 360.0f;
            }
						chassis_controller.target_yaw_v = 0;

            // 底盘控制
            chassis_solver.chassis_speed_x = (remote_controller.dji_remote.rc.ch[RIGHT_CH_LR] - CH_MIDDLE) * 1.0f / CH_RANGE;
            chassis_solver.chassis_speed_y = -(remote_controller.dji_remote.rc.ch[RIGHT_CH_UD] - CH_MIDDLE) * 1.0f / CH_RANGE;
            
						chassis_controller.target_x_v = chassis_solver.chassis_speed_x;
	        chassis_controller.target_y_v = chassis_solver.chassis_speed_y;
						break;
    }

    if (offline_detector.remote_state == REMOTE_OFF)
    {
        setAllModeOff();
    }
            
    }else if(remote_controller.control_type == DJI_REMOTE_ROTATE_CONTROL){
        switch (remote_controller.dji_remote.rc.s[LEFT_SW])
        {
            case Down:
                setAllModeOff();
                break;
            case Mid:
                setRobotState(CONTROL_MODE);
                setControlModeAction(CV_ROTATE);
                setShootAction(SHOOT_POWERDOWN_MODE);
                setGimbalAction(GIMBAL_ACT_MODE);
                setSuperPower(POWER_TO_BATTERY);

                // 云台控制
                gimbal_controller.target_yaw_angle -= (remote_controller.dji_remote.rc.ch[LEFT_CH_LR] - CH_MIDDLE) * MAX_SW_YAW_SPEED / CH_RANGE * infantry->delta_t;
                gimbal_controller.target_pitch_angle += (remote_controller.dji_remote.rc.ch[LEFT_CH_UD] - CH_MIDDLE) * MAX_SW_PITCH_SPEED / CH_RANGE * infantry->delta_t;
                
                if(gimbal_controller.target_yaw_angle > 360.0f)
                {
                    gimbal_controller.target_yaw_angle -= 360.0f;
                }
                else if(gimbal_controller.target_yaw_angle < -0.0f)
                {
                    gimbal_controller.target_yaw_angle += 360.0f;
                }

                // 底盘控制
                chassis_solver.chassis_speed_x = (remote_controller.dji_remote.rc.ch[RIGHT_CH_LR] - CH_MIDDLE) * 1.0f / CH_RANGE;
                chassis_solver.chassis_speed_y = -(remote_controller.dji_remote.rc.ch[RIGHT_CH_UD] - CH_MIDDLE) * 1.0f / CH_RANGE;
								chassis_controller.target_yaw_v = 1;  
                chassis_controller.target_x_v = chassis_solver.chassis_speed_x;
                chassis_controller.target_y_v = chassis_solver.chassis_speed_y;

                break;
			case Up:
                setRobotState(CONTROL_MODE);
                setControlModeAction(NOT_FOLLOW_GIMBAL);
                setShootAction(SHOOT_POWERDOWN_MODE);
                setGimbalAction(GIMBAL_ACT_MODE);
                setSuperPower(POWER_TO_BATTERY);

                // 云台控制
                gimbal_controller.target_yaw_angle -= (remote_controller.dji_remote.rc.ch[LEFT_CH_LR] - CH_MIDDLE) * MAX_SW_YAW_SPEED / CH_RANGE * infantry->delta_t;
                gimbal_controller.target_pitch_angle += (remote_controller.dji_remote.rc.ch[LEFT_CH_UD] - CH_MIDDLE) * MAX_SW_PITCH_SPEED / CH_RANGE * infantry->delta_t;
                
                if(gimbal_controller.target_yaw_angle > 360.0f)
                {
                    gimbal_controller.target_yaw_angle -= 360.0f;
                }
                else if(gimbal_controller.target_yaw_angle < -0.0f)
                {
                    gimbal_controller.target_yaw_angle += 360.0f;
                }

                // 底盘控制
                chassis_solver.chassis_speed_x = (remote_controller.dji_remote.rc.ch[RIGHT_CH_LR] - CH_MIDDLE) * 1.0f / CH_RANGE;
                chassis_solver.chassis_speed_y = -(remote_controller.dji_remote.rc.ch[RIGHT_CH_UD] - CH_MIDDLE) * 1.0f / CH_RANGE;
								chassis_controller.target_yaw_v = 1;  
                chassis_controller.target_x_v = chassis_solver.chassis_speed_x;
                chassis_controller.target_y_v = chassis_solver.chassis_speed_y;

                break;
        }
	}
}

/**
 * @brief 根据遥控器或者蓝牙更新控制状态
 * @param[in] infantry
 */

void get_control_info(ChassisSolver *infantry)
{
    if(!system_started)
    {
        switch (remote_controller.dji_remote.rc.s[RIGHT_SW])
        {
            case Down:
                setControlMode(DJI_REMOTE_CONTROL);
                break;
            case Mid:
                setControlMode(DJI_REMOTE_ROTATE_CONTROL);
                break;
            case Up:
                setControlMode(SENTRY_CONTROL);
                break;
            default:
                break;
        }

        switch (remote_controller.control_type)
        {
            case SENTRY_CONTROL:
                Sentry_Mode_Update(infantry);
                break;
            case DJI_REMOTE_ROTATE_CONTROL:
                        DJIRemoteUpdate(infantry);
                            break;
            case DJI_REMOTE_CONTROL:
                DJIRemoteUpdate(infantry);
                break;
            default:
                break;
        }
    }else if(system_started){
        remote_controller.dji_remote.rc.s[RIGHT_SW] = Up;
        remote_controller.control_type = SENTRY_CONTROL;
        remote_controller.dji_remote.rc.s[LEFT_SW] = Mid; // 设置为哨兵模式

        switch (remote_controller.dji_remote.rc.s[RIGHT_SW])
        {
            case Down:
                setControlMode(DJI_REMOTE_CONTROL);
                break;
            case Mid:
                setControlMode(DJI_REMOTE_ROTATE_CONTROL);
                break;
            case Up:
                setControlMode(SENTRY_CONTROL);
                break;
            default:
                break;
        }

        switch (remote_controller.control_type)
        {
            case SENTRY_CONTROL:
                Sentry_Mode_Update(infantry);
                break;
            case DJI_REMOTE_ROTATE_CONTROL:
                        DJIRemoteUpdate(infantry);
                            break;
            case DJI_REMOTE_CONTROL:
                DJIRemoteUpdate(infantry);
                break;
            default:
                break;
        }
    }
}
