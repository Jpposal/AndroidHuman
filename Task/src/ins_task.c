/**
 ******************************************************************************
 * @file    ins_task.c
 * @author  Wang Hongxi
 * @version V2.0.0
 * @date    2022/2/23
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#include "ins_task.h"
#include "QuaternionEKF.h"
#include "bsp_PWM.h"
#include "pid.h"

INS_t INS;
IMU_Param_t IMU_Param;
PID_t TempCtrl = {0};

// 惯性导航扩展对象
INS_Navigation_Extension_t INS_Nav;

const float xb[3] = {1, 0, 0};
const float yb[3] = {0, 1, 0};
const float zb[3] = {0, 0, 1};

uint32_t INS_DWT_Count = 0;
static float dt = 0, t = 0;
uint8_t ins_debug_mode = 0;
float RefTemp = 28.125;

static void IMU_Param_Correction(IMU_Param_t *param, float gyro[3], float accel[3]);

void INS_Init(void)
{
    IMU_Param.scale[X] = 1;
    IMU_Param.scale[Y] = 1;
    IMU_Param.scale[Z] = 1;
    IMU_Param.Yaw = 0;
    IMU_Param.Pitch = 0;
    IMU_Param.Roll = 0;
    IMU_Param.flag = 1;

    IMU_QuaternionEKF_Init(10, 0.001, 30000000, 1, 0);
    // imu heat init
    PID_Init(&TempCtrl, 2500, 300, 0, 1000, 20, 0, 0, 0, 0, 0, 0, 0);
    HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);

    INS.AccelLPF = 0.0085;
}

void INS_Task(void)
{
    static uint32_t count = 0;
    const float gravity[3] = {0, 0, 9.81f};
    dt = DWT_GetDeltaT(&INS_DWT_Count);
    t += dt;

    // ins update
    if ((count % 1) == 0)
    {
        BMI088_Read(&BMI088);
        LossUpdate(&global_debugger.imu_debugger[0], 0.0015);
        LossUpdate(&global_debugger.imu_debugger[1], 0.0015);

        INS.Accel[X] = BMI088.Accel[X];
        INS.Accel[Y] = BMI088.Accel[Y];
        INS.Accel[Z] = BMI088.Accel[Z];
        INS.Gyro[X] = BMI088.Gyro[X];
        INS.Gyro[Y] = BMI088.Gyro[Y];
        INS.Gyro[Z] = BMI088.Gyro[Z];

        // demo function,用于修正安装误差,可以不管,本demo暂时没用
        IMU_Param_Correction(&IMU_Param, INS.Gyro, INS.Accel);

        // 计算重力加速度矢量和b系的XY两轴的夹角,可用作功能扩展,本demo暂时没用
        INS.atanxz = -atan2f(INS.Accel[X], INS.Accel[Z]) * 180 / PI;
        INS.atanyz = atan2f(INS.Accel[Y], INS.Accel[Z]) * 180 / PI;

        // 核心函数,EKF更新四元数
        IMU_QuaternionEKF_Update(INS.Gyro[X], INS.Gyro[Y], INS.Gyro[Z], INS.Accel[X], INS.Accel[Y], INS.Accel[Z], dt);

        memcpy(INS.q, QEKF_INS.q, sizeof(QEKF_INS.q));

        // 机体系基向量转换到导航坐标系，本例选取惯性系为导航系
        BodyFrameToEarthFrame(xb, INS.xn, INS.q);
        BodyFrameToEarthFrame(yb, INS.yn, INS.q);
        BodyFrameToEarthFrame(zb, INS.zn, INS.q);

        // 将重力从导航坐标系n转换到机体系b,随后根据加速度计数据计算运动加速度
        float gravity_b[3];
        EarthFrameToBodyFrame(gravity, gravity_b, INS.q);
        for (uint8_t i = 0; i < 3; i++) // 同样过一个低通滤波
        {
            INS.MotionAccel_b[i] = (INS.Accel[i] - gravity_b[i]) * dt / (INS.AccelLPF + dt) + INS.MotionAccel_b[i] * INS.AccelLPF / (INS.AccelLPF + dt);
        }
        BodyFrameToEarthFrame(INS.MotionAccel_b, INS.MotionAccel_n, INS.q); // 转换回导航系n

        // 获取最终数据
        INS.Yaw = QEKF_INS.Yaw + 180.0f;
        INS.Pitch = QEKF_INS.Roll; // 对Pitch和roll进行交换（由于IMU安装问题）
        INS.Roll = QEKF_INS.Pitch;
        INS.YawTotalAngle = QEKF_INS.YawTotalAngle;
    }

    // temperature control
    if ((count % 2) == 0)
    {
        // 500hz
        IMU_Temperature_Ctrl();
    }

    if ((count % 1000) == 0)
    {
        // 200hz
    }

    count++;
}

/**
 * @brief          Transform 3dvector from BodyFrame to EarthFrame
 * @param[1]       vector in BodyFrame
 * @param[2]       vector in EarthFrame
 * @param[3]       quaternion
 */
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q)
{
    vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] +
                       (q[1] * q[2] - q[0] * q[3]) * vecBF[1] +
                       (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

    vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
                       (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

    vecEF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] +
                       (q[2] * q[3] + q[0] * q[1]) * vecBF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}

/**
 * @brief          Transform 3dvector from EarthFrame to BodyFrame
 * @param[1]       vector in EarthFrame
 * @param[2]       vector in BodyFrame
 * @param[3]       quaternion
 */
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q)
{
    vecBF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] +
                       (q[1] * q[2] + q[0] * q[3]) * vecEF[1] +
                       (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

    vecBF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
                       (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

    vecBF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] +
                       (q[2] * q[3] - q[0] * q[1]) * vecEF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}

/**
 * @brief reserved.用于修正IMU安装误差与标度因数误差,即陀螺仪轴和云台轴的安装偏移
 *
 *
 * @param param IMU参数
 * @param gyro  角速度
 * @param accel 加速度
 */
static void IMU_Param_Correction(IMU_Param_t *param, float gyro[3], float accel[3])
{
    static float lastYawOffset, lastPitchOffset, lastRollOffset;
    static float c_11, c_12, c_13, c_21, c_22, c_23, c_31, c_32, c_33;
    float cosPitch, cosYaw, cosRoll, sinPitch, sinYaw, sinRoll;

    if (fabsf(param->Yaw - lastYawOffset) > 0.001f ||
        fabsf(param->Pitch - lastPitchOffset) > 0.001f ||
        fabsf(param->Roll - lastRollOffset) > 0.001f || param->flag)
    {
        cosYaw = arm_cos_f32(param->Yaw / 57.295779513f);
        cosPitch = arm_cos_f32(param->Pitch / 57.295779513f);
        cosRoll = arm_cos_f32(param->Roll / 57.295779513f);
        sinYaw = arm_sin_f32(param->Yaw / 57.295779513f);
        sinPitch = arm_sin_f32(param->Pitch / 57.295779513f);
        sinRoll = arm_sin_f32(param->Roll / 57.295779513f);

        // 1.yaw(alpha) 2.pitch(beta) 3.roll(gamma)
        c_11 = cosYaw * cosRoll + sinYaw * sinPitch * sinRoll;
        c_12 = cosPitch * sinYaw;
        c_13 = cosYaw * sinRoll - cosRoll * sinYaw * sinPitch;
        c_21 = cosYaw * sinPitch * sinRoll - cosRoll * sinYaw;
        c_22 = cosYaw * cosPitch;
        c_23 = -sinYaw * sinRoll - cosYaw * cosRoll * sinPitch;
        c_31 = -cosPitch * sinRoll;
        c_32 = sinPitch;
        c_33 = cosPitch * cosRoll;
        param->flag = 0;
    }
    float gyro_temp[3];
    for (uint8_t i = 0; i < 3; i++)
        gyro_temp[i] = gyro[i] * param->scale[i];

    gyro[X] = c_11 * gyro_temp[X] +
              c_12 * gyro_temp[Y] +
              c_13 * gyro_temp[Z];
    gyro[Y] = c_21 * gyro_temp[X] +
              c_22 * gyro_temp[Y] +
              c_23 * gyro_temp[Z];
    gyro[Z] = c_31 * gyro_temp[X] +
              c_32 * gyro_temp[Y] +
              c_33 * gyro_temp[Z];

    float accel_temp[3];
    for (uint8_t i = 0; i < 3; i++)
        accel_temp[i] = accel[i];

    accel[X] = c_11 * accel_temp[X] +
               c_12 * accel_temp[Y] +
               c_13 * accel_temp[Z];
    accel[Y] = c_21 * accel_temp[X] +
               c_22 * accel_temp[Y] +
               c_23 * accel_temp[Z];
    accel[Z] = c_31 * accel_temp[X] +
               c_32 * accel_temp[Y] +
               c_33 * accel_temp[Z];

    lastYawOffset = param->Yaw;
    lastPitchOffset = param->Pitch;
    lastRollOffset = param->Roll;
}

/**
 * @brief 温度控制
 *
 */
void IMU_Temperature_Ctrl(void)
{
    PID_Calculate(&TempCtrl, BMI088.Temperature, RefTemp);

    TIM_Set_PWM(&htim10, TIM_CHANNEL_1, LIMIT_MAX_MIN(float_rounding(TempCtrl.Output), UINT32_MAX, 0));
}

//============================ 惯性导航扩展功能实现 ============================

/**
 * @brief 惯性导航扩展初始化
 */
void INS_Navigation_Extension_Init(void)
{
    // 清零所有数据
    memset(&INS_Nav, 0, sizeof(INS_Navigation_Extension_t));
    
    // 初始化状态
    INS_Nav.navigation_enabled = 1;
    INS_Nav.trajectory_recording = 1;
    INS_Nav.position_reset_flag = 0;
    
    // 初始化时间
    INS_Nav.nav_last_update_time = DWT_GetTimeline_ms();
    
    // 添加初始轨迹点
    INS_Navigation_Extension_Add_Trajectory_Point();
}

/**
 * @brief 变换加速度到世界坐标系（去除重力影响）
 */
void INS_Navigation_Extension_Transform_Acceleration(void)
{
    // 获取当前姿态角（弧度）
    float roll_rad = INS.Roll * ANGLE_TO_RAD_COEF;
    float pitch_rad = INS.Pitch * ANGLE_TO_RAD_COEF;
    float yaw_rad = INS.Yaw * ANGLE_TO_RAD_COEF;
    
    // 获取机体坐标系加速度（从INS获取，单位：m/s²）
    float accel_body_x = INS.Accel[X];
    float accel_body_y = INS.Accel[Y];
    float accel_body_z = INS.Accel[Z];
    
    // 旋转矩阵变换：机体坐标系 -> 世界坐标系
    float cos_roll = arm_cos_f32(roll_rad);
    float sin_roll = arm_sin_f32(roll_rad);
    float cos_pitch = arm_cos_f32(pitch_rad);
    float sin_pitch = arm_sin_f32(pitch_rad);
    float cos_yaw = arm_cos_f32(yaw_rad);
    float sin_yaw = arm_sin_f32(yaw_rad);
    
    // 旋转矩阵计算（ZYX欧拉角顺序）
    float r11 = cos_yaw * cos_pitch;
    float r12 = cos_yaw * sin_pitch * sin_roll - sin_yaw * cos_roll;
    float r13 = cos_yaw * sin_pitch * cos_roll + sin_yaw * sin_roll;
    
    float r21 = sin_yaw * cos_pitch;
    float r22 = sin_yaw * sin_pitch * sin_roll + cos_yaw * cos_roll;
    float r23 = sin_yaw * sin_pitch * cos_roll - cos_yaw * sin_roll;
    
    float r31 = -sin_pitch;
    float r32 = cos_pitch * sin_roll;
    float r33 = cos_pitch * cos_roll;
    
    // 变换到世界坐标系
    float world_accel_x = r11 * accel_body_x + r12 * accel_body_y + r13 * accel_body_z;
    float world_accel_y = r21 * accel_body_x + r22 * accel_body_y + r23 * accel_body_z;
    float world_accel_z = r31 * accel_body_x + r32 * accel_body_y + r33 * accel_body_z;
    
    // 去除重力影响（假设Z轴向上为正）
    INS_Nav.world_accel_x = world_accel_x;
    INS_Nav.world_accel_y = world_accel_y;
    INS_Nav.world_accel_z = world_accel_z - NAV_GRAVITY_EARTH;
    
    // 小加速度过滤（减少噪声影响）
    if (fabsf(INS_Nav.world_accel_x) < NAV_MIN_ACCEL_THRESHOLD) INS_Nav.world_accel_x = 0.0f;
    if (fabsf(INS_Nav.world_accel_y) < NAV_MIN_ACCEL_THRESHOLD) INS_Nav.world_accel_y = 0.0f;
    if (fabsf(INS_Nav.world_accel_z) < NAV_MIN_ACCEL_THRESHOLD) INS_Nav.world_accel_z = 0.0f;
}

/**
 * @brief 惯性导航扩展状态更新
 */
void INS_Navigation_Extension_Update(void)
{
    if (!INS_Nav.navigation_enabled) return;
    
    // 获取当前时间和计算时间差
    uint32_t current_time = DWT_GetTimeline_ms();
    INS_Nav.nav_delta_time = (current_time - INS_Nav.nav_last_update_time) / 1000.0f;
    INS_Nav.nav_last_update_time = current_time;
    INS_Nav.nav_total_time += INS_Nav.nav_delta_time;
    
    // 防止时间差过大（系统重启或长时间暂停）
    if (INS_Nav.nav_delta_time > 0.1f) {
        INS_Nav.nav_delta_time = 0.001f; // 重置为1ms
    }
    
    // 变换加速度到世界坐标系
    INS_Navigation_Extension_Transform_Acceleration();
    
    // 数值积分：加速度 -> 速度
    INS_Nav.velocity_x += INS_Nav.world_accel_x * INS_Nav.nav_delta_time;
    INS_Nav.velocity_y += INS_Nav.world_accel_y * INS_Nav.nav_delta_time;
    INS_Nav.velocity_z += INS_Nav.world_accel_z * INS_Nav.nav_delta_time;
    
    // 速度衰减（模拟摩擦和空气阻力）
    INS_Nav.velocity_x *= NAV_VELOCITY_DECAY_FACTOR;
    INS_Nav.velocity_y *= NAV_VELOCITY_DECAY_FACTOR;
    INS_Nav.velocity_z *= NAV_VELOCITY_DECAY_FACTOR;
    
    // 速度限制
    if (fabsf(INS_Nav.velocity_x) > NAV_MAX_VELOCITY_THRESHOLD) {
        INS_Nav.velocity_x = (INS_Nav.velocity_x > 0) ? NAV_MAX_VELOCITY_THRESHOLD : -NAV_MAX_VELOCITY_THRESHOLD;
    }
    if (fabsf(INS_Nav.velocity_y) > NAV_MAX_VELOCITY_THRESHOLD) {
        INS_Nav.velocity_y = (INS_Nav.velocity_y > 0) ? NAV_MAX_VELOCITY_THRESHOLD : -NAV_MAX_VELOCITY_THRESHOLD;
    }
    if (fabsf(INS_Nav.velocity_z) > NAV_MAX_VELOCITY_THRESHOLD) {
        INS_Nav.velocity_z = (INS_Nav.velocity_z > 0) ? NAV_MAX_VELOCITY_THRESHOLD : -NAV_MAX_VELOCITY_THRESHOLD;
    }
    
    // 数值积分：速度 -> 位置
    INS_Nav.position_x += INS_Nav.velocity_x * INS_Nav.nav_delta_time;
    INS_Nav.position_y += INS_Nav.velocity_y * INS_Nav.nav_delta_time;
    INS_Nav.position_z += INS_Nav.velocity_z * INS_Nav.nav_delta_time;
    
    // 计算统计信息
    INS_Navigation_Extension_Calculate_Statistics();
    
    // 检查是否需要重置位置
    if (INS_Nav.position_reset_flag) {
        INS_Navigation_Extension_Reset_Position();
        INS_Nav.position_reset_flag = 0;
    }
}

/**
 * @brief 计算统计信息
 */
void INS_Navigation_Extension_Calculate_Statistics(void)
{
    // 计算当前速度大小
    float current_velocity = sqrtf(INS_Nav.velocity_x * INS_Nav.velocity_x + 
                                  INS_Nav.velocity_y * INS_Nav.velocity_y + 
                                  INS_Nav.velocity_z * INS_Nav.velocity_z);
    
    // 更新最大速度
    if (current_velocity > INS_Nav.max_velocity) {
        INS_Nav.max_velocity = current_velocity;
    }
    
    // 计算位移增量并累加总距离
    static float last_pos_x = 0.0f, last_pos_y = 0.0f, last_pos_z = 0.0f;
    float delta_distance = sqrtf(powf(INS_Nav.position_x - last_pos_x, 2) +
                                powf(INS_Nav.position_y - last_pos_y, 2) +
                                powf(INS_Nav.position_z - last_pos_z, 2));
    
    INS_Nav.total_distance += delta_distance;
    
    // 更新上次位置
    last_pos_x = INS_Nav.position_x;
    last_pos_y = INS_Nav.position_y;
    last_pos_z = INS_Nav.position_z;
    
    // 计算平均速度
    if (INS_Nav.nav_total_time > 0) {
        INS_Nav.avg_velocity = INS_Nav.total_distance / INS_Nav.nav_total_time;
    }
}

/**
 * @brief 添加轨迹点
 */
void INS_Navigation_Extension_Add_Trajectory_Point(void)
{
    if (!INS_Nav.trajectory_recording) return;
    
    if (INS_Nav.trajectory_count < NAV_MAX_TRAJECTORY_POINTS) {
        NAV_TrajectoryPoint_t *point = &INS_Nav.trajectory[INS_Nav.trajectory_index];
        
        point->x = INS_Nav.position_x;
        point->y = INS_Nav.position_y;
        point->z = INS_Nav.position_z;
        point->timestamp = INS_Nav.nav_total_time;
        point->valid = 1;
        
        INS_Nav.trajectory_index = (INS_Nav.trajectory_index + 1) % NAV_MAX_TRAJECTORY_POINTS;
        if (INS_Nav.trajectory_count < NAV_MAX_TRAJECTORY_POINTS) {
            INS_Nav.trajectory_count++;
        }
    }
}

/**
 * @brief 重置位置信息
 */
void INS_Navigation_Extension_Reset_Position(void)
{
    INS_Nav.position_x = 0.0f;
    INS_Nav.position_y = 0.0f;
    INS_Nav.position_z = 0.0f;
    
    INS_Nav.velocity_x = 0.0f;
    INS_Nav.velocity_y = 0.0f;
    INS_Nav.velocity_z = 0.0f;
    
    INS_Nav.total_distance = 0.0f;
    INS_Nav.max_velocity = 0.0f;
    INS_Nav.avg_velocity = 0.0f;
    INS_Nav.nav_total_time = 0.0f;
    
    // 清空轨迹
    INS_Nav.trajectory_count = 0;
    INS_Nav.trajectory_index = 0;
    memset(INS_Nav.trajectory, 0, sizeof(INS_Nav.trajectory));
    
    // 添加重置后的初始点
    INS_Navigation_Extension_Add_Trajectory_Point();
}

/**
 * @brief 使能/禁用轨迹记录
 * @param enable 1-使能，0-禁用
 */
void INS_Navigation_Extension_Enable_Recording(uint8_t enable)
{
    INS_Nav.trajectory_recording = enable;
}

/**
 * @brief 获取行程距离
 * @return 总行程距离 (m)
 */
float INS_Navigation_Extension_Get_Distance_Traveled(void)
{
    return INS_Nav.total_distance;
}

/**
 * @brief 获取轨迹点数量
 * @return 轨迹点数量
 */
uint16_t INS_Navigation_Extension_Get_Trajectory_Count(void)
{
    return INS_Nav.trajectory_count;
}

/**
 * @brief 获取当前位置信息
 * @return 当前位置的轨迹点指针
 */
NAV_TrajectoryPoint_t* INS_Navigation_Extension_Get_Current_Position(void)
{
    static NAV_TrajectoryPoint_t current_pos;
    current_pos.x = INS_Nav.position_x;
    current_pos.y = INS_Nav.position_y;
    current_pos.z = INS_Nav.position_z;
    current_pos.timestamp = INS_Nav.nav_total_time;
    current_pos.valid = 1;
    return &current_pos;
}

/**
 * @brief        Update quaternion
 */
void QuaternionUpdate(float *q, float gx, float gy, float gz, float dt)
{
    float qa, qb, qc;

    gx *= 0.5f * dt;
    gy *= 0.5f * dt;
    gz *= 0.5f * dt;
    qa = q[0];
    qb = q[1];
    qc = q[2];
    q[0] += (-qb * gx - qc * gy - q[3] * gz);
    q[1] += (qa * gx + qc * gz - q[3] * gy);
    q[2] += (qa * gy - qb * gz + q[3] * gx);
    q[3] += (qa * gz + qb * gy - qc * gx);
}

/**
 * @brief        Convert quaternion to eular angle
 */
void QuaternionToEularAngle(float *q, float *Yaw, float *Pitch, float *Roll)
{
    *Yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f) * 57.295779513f;
    *Pitch = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f) * 57.295779513f;
    *Roll = asinf(2.0f * (q[0] * q[2] - q[1] * q[3])) * 57.295779513f;
}

/**
 * @brief        Convert eular angle to quaternion
 */
void EularAngleToQuaternion(float Yaw, float Pitch, float Roll, float *q)
{
    float cosPitch, cosYaw, cosRoll, sinPitch, sinYaw, sinRoll;
    Yaw /= 57.295779513f;
    Pitch /= 57.295779513f;
    Roll /= 57.295779513f;
    cosPitch = arm_cos_f32(Pitch / 2);
    cosYaw = arm_cos_f32(Yaw / 2);
    cosRoll = arm_cos_f32(Roll / 2);
    sinPitch = arm_sin_f32(Pitch / 2);
    sinYaw = arm_sin_f32(Yaw / 2);
    sinRoll = arm_sin_f32(Roll / 2);
    q[0] = cosPitch * cosRoll * cosYaw + sinPitch * sinRoll * sinYaw;
    q[1] = sinPitch * cosRoll * cosYaw - cosPitch * sinRoll * sinYaw;
    q[2] = sinPitch * cosRoll * sinYaw + cosPitch * sinRoll * cosYaw;
    q[3] = cosPitch * cosRoll * sinYaw - sinPitch * sinRoll * cosYaw;
}

/**
 * @brief 云台位姿估计任务
 * @param[in] void
 */
void GimbalEstimate_task(void *pvParameters)
{
    portTickType xLastWakeTime;
    const portTickType xFrequency = 1; // 1kHZ

    INS_Init();
    
    // 初始化惯性导航扩展功能
    INS_Navigation_Extension_Init();

    // 适当延时
    vTaskDelay(100);

    // 轨迹记录计数器
    uint16_t nav_trajectory_counter = 0;

    while (1)
    {
        xLastWakeTime = xTaskGetTickCount();

        /* 执行原有任务 */
        INS_Task();
        
        /* 执行惯性导航扩展功能 */
        INS_Navigation_Extension_Update();
        
        // 定期记录轨迹点（10Hz）
        nav_trajectory_counter++;
        if (nav_trajectory_counter >= 100) { // 1000Hz / 100 = 10Hz
            INS_Navigation_Extension_Add_Trajectory_Point();
            nav_trajectory_counter = 0;
        }

        /*  喂狗 */
        // xEventGroupSetBits(xCreatedEventGroup, GIMBAL_ESTIMATE_BIT); // 标志位置一

        /*  延时  */
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
