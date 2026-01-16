/**
 ******************************************************************************
 * @file    ins_task.h
 * @author  Wang Hongxi
 * @version V2.0.0
 * @date    2022/2/23
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#ifndef __INS_TASK_H
#define __INS_TASK_H

#include "stdint.h"
#include "BMI088driver.h"
#include "QuaternionEKF.h"
#include "bsp_dwt.h"
#include <math.h>
#include <string.h>

#define X 0
#define Y 1
#define Z 2

#define INS_TASK_PERIOD 1
#define RAD_TO_ANGLE_COEF 57.295779513f
#define ANGLE_TO_RAD_COEF 0.0174532925f
#define PI_2 6.2831853072f

// 惯性导航扩展参数定义
#define NAV_GRAVITY_EARTH               9.80665f // 地球重力加速度 m/s²
#define NAV_MIN_ACCEL_THRESHOLD         0.02f   // 最小加速度阈值 m/s²
#define NAV_MAX_VELOCITY_THRESHOLD      10.0f   // 最大速度阈值 m/s
#define NAV_VELOCITY_DECAY_FACTOR       0.98f   // 速度衰减因子（模拟摩擦）
#define NAV_MAX_TRAJECTORY_POINTS       1000    // 最大轨迹点数（减少内存占用）

// 轨迹点结构体
typedef struct {
    float x;            // X坐标 (m)
    float y;            // Y坐标 (m)
    float z;            // Z坐标 (m)
    float timestamp;    // 时间戳 (s)
    uint8_t valid;      // 有效标志
} NAV_TrajectoryPoint_t;

// 惯性导航扩展结构体
typedef struct {
    // 位置信息 (m)
    float position_x;
    float position_y;
    float position_z;
    
    // 速度信息 (m/s)
    float velocity_x;
    float velocity_y;
    float velocity_z;
    
    // 世界坐标系加速度信息 (m/s²) - 去除重力后的线性加速度
    float world_accel_x;
    float world_accel_y;
    float world_accel_z;
    
    // 时间信息
    float nav_delta_time;   // 导航时间差 (s)
    float nav_total_time;   // 导航总运行时间 (s)
    uint32_t nav_last_update_time;
    
    // 轨迹记录
    NAV_TrajectoryPoint_t trajectory[NAV_MAX_TRAJECTORY_POINTS];
    uint16_t trajectory_count;
    uint16_t trajectory_index;
    
    // 统计信息
    float total_distance;       // 总行程距离 (m)
    float max_velocity;         // 最大速度 (m/s)
    float avg_velocity;         // 平均速度 (m/s)
    
    // 状态标志
    uint8_t navigation_enabled;     // 导航使能
    uint8_t trajectory_recording;   // 轨迹记录使能
    uint8_t position_reset_flag;    // 位置重置标志
    
} INS_Navigation_Extension_t;

// 惯性导航扩展参数定义
#define NAV_GRAVITY_EARTH               9.80665f // 地球重力加速度 m/s²
#define NAV_MIN_ACCEL_THRESHOLD         0.02f   // 最小加速度阈值 m/s²
#define NAV_MAX_VELOCITY_THRESHOLD      10.0f   // 最大速度阈值 m/s
#define NAV_VELOCITY_DECAY_FACTOR       0.98f   // 速度衰减因子（模拟摩擦）
#define NAV_MAX_TRAJECTORY_POINTS       1000    // 最大轨迹点数（减少内存占用）


typedef struct
{
    float q[4]; // 四元数估计值

    float Gyro[3];          // 角速度
    float Accel[3];         // 加速度
    float MotionAccel_b[3]; // 机体坐标加速度
    float MotionAccel_n[3]; // 绝对系加速度

    float AccelLPF; // 加速度低通滤波系数

    // 加速度在绝对系的向量表示
    float xn[3];
    float yn[3];
    float zn[3];

    float atanxz;
    float atanyz;

    // 位姿
    float Roll;
    float Pitch;
    float Yaw;
    float YawTotalAngle;
} INS_t;

/**
 * @brief 用于修正安装误差的参数,demo中可无视
 *
 */
typedef struct
{
    uint8_t flag;

    float scale[3];

    float Yaw;
    float Pitch;
    float Roll;
} IMU_Param_t;

extern INS_t INS;

// 惯性导航扩展对象
extern INS_Navigation_Extension_t INS_Nav;

void INS_Init(void);
void INS_Task(void);
void IMU_Temperature_Ctrl(void);

void QuaternionUpdate(float *q, float gx, float gy, float gz, float dt);
void QuaternionToEularAngle(float *q, float *Yaw, float *Pitch, float *Roll);
void EularAngleToQuaternion(float Yaw, float Pitch, float Roll, float *q);
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q);
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q);

void GimbalEstimate_task(void *pvParameters);

// 惯性导航扩展函数声明
void INS_Navigation_Extension_Init(void);
void INS_Navigation_Extension_Update(void);
void INS_Navigation_Extension_Reset_Position(void);
void INS_Navigation_Extension_Enable_Recording(uint8_t enable);
void INS_Navigation_Extension_Add_Trajectory_Point(void);
void INS_Navigation_Extension_Calculate_Statistics(void);
void INS_Navigation_Extension_Transform_Acceleration(void);
float INS_Navigation_Extension_Get_Distance_Traveled(void);
NAV_TrajectoryPoint_t* INS_Navigation_Extension_Get_Current_Position(void);
uint16_t INS_Navigation_Extension_Get_Trajectory_Count(void);

#endif
